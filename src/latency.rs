//! Objective latency measurement with a microphone.
//!
//! Windows plays a click train. The clicks reach the speakers directly and the AirPods through airlow. A
//! microphone placed next to the AirPods records both; a matched filter finds each click, and the gap between
//! the speaker click and the AirPods click is the AirPods' delay relative to the speakers. The number is stable
//! to a fraction of a millisecond, so it can compare tweaks that ears cannot.

use anyhow::{Result, anyhow};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::sync::{Arc, Mutex};

/// The click template: a 2 ms, 2 kHz burst with a linear decay (the same shape as the generated test WAV).
pub fn click_template(fs: f64) -> Vec<f32> {
    let n = (0.002 * fs) as usize;
    (0..n).map(|i| ((std::f64::consts::TAU * 2000.0 * i as f64 / fs).sin() * (1.0 - i as f64 / n as f64)) as f32).collect()
}

/// Matched-filter detection: returns (time in seconds, strength) for each click-like event.
pub fn find_clicks(rec: &[f32], fs: f64) -> Vec<(f64, f32)> {
    let tpl = click_template(fs);
    let n = tpl.len();
    if rec.len() <= n {
        return vec![];
    }
    let norm = tpl.iter().map(|v| v * v).sum::<f32>().sqrt();
    let corr: Vec<f32> = (0..rec.len() - n).map(|i| (0..n).map(|k| rec[i + k] * tpl[k]).sum::<f32>() / norm).collect();
    let mut mags: Vec<f32> = corr.iter().map(|c| c.abs()).collect();
    let max = mags.iter().cloned().fold(0.0, f32::max);
    let mut sorted = mags.clone();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let noise = sorted[sorted.len() / 2].max(1e-9);
    // Real clicks stand far above the noise floor; pure noise has heavy tails but never reaches this.
    let p99 = sorted[sorted.len() * 99 / 100];
    let thr = (max * 0.06).max(noise * 25.0).max(p99 * 3.0);
    if max < noise * 40.0 {
        return vec![]; // nothing click-like in the recording at all
    }
    let min_sep = (0.004 * fs) as usize; // clicks closer than 4 ms are one event
    let mut out: Vec<(usize, f32)> = Vec::new();
    let mut i = 0;
    while i < mags.len() {
        if mags[i] >= thr {
            // peak within the next min_sep samples
            let end = (i + min_sep).min(mags.len());
            let (pk, val) = (i..end).map(|j| (j, mags[j])).fold((i, 0.0), |a, b| if b.1 > a.1 { b } else { a });
            out.push((pk, val));
            i = end;
        } else {
            i += 1;
        }
    }
    mags.clear();
    out.into_iter().map(|(p, v)| (p as f64 / fs, v)).collect()
}

/// Group clicks into 1-per-second events and return, for each event with two clicks, the gap in ms between the
/// first (speaker) and the strongest later one (AirPods) within 250 ms.
pub fn pair_delays(clicks: &[(f64, f32)]) -> Vec<f64> {
    let mut delays = Vec::new();
    let mut i = 0;
    while i < clicks.len() {
        let t0 = clicks[i].0;
        let mut j = i + 1;
        let mut best: Option<(f64, f32)> = None;
        while j < clicks.len() && clicks[j].0 - t0 < 0.25 {
            if clicks[j].0 - t0 > 0.0015 && best.map(|b| clicks[j].1 > b.1).unwrap_or(true) {
                best = Some(clicks[j]);
            }
            j += 1;
        }
        if let Some((t1, _)) = best {
            delays.push((t1 - t0) * 1000.0);
        }
        i = j.max(i + 1);
    }
    delays
}

pub fn summarize(delays: &[f64]) -> Option<(f64, f64, f64)> {
    if delays.is_empty() {
        return None;
    }
    let mut d = delays.to_vec();
    d.sort_by(|a, b| a.partial_cmp(b).unwrap());
    Some((d[0], d[d.len() / 2], d[d.len() - 1]))
}

/// Record from the first input device whose name contains `hint` (case-insensitive).
pub fn start_mic_recording(hint: &str) -> Result<(cpal::Stream, Arc<Mutex<Vec<f32>>>, f64)> {
    let host = cpal::default_host();
    let dev = host
        .input_devices()?
        .find(|d| d.description().map(|x| x.name().to_lowercase().contains(&hint.to_lowercase())).unwrap_or(false))
        .ok_or_else(|| anyhow!("no input device matching '{hint}'"))?;
    let c = dev.default_input_config()?;
    let (ch, fs) = (c.channels() as usize, c.sample_rate() as f64);
    println!("recording from '{}' ({ch} ch, {fs} Hz, {:?})", dev.description()?.name(), c.sample_format());
    let buf: Arc<Mutex<Vec<f32>>> = Arc::new(Mutex::new(Vec::new()));
    let b = buf.clone();
    let cfg: cpal::StreamConfig = c.clone().into();
    let err = |e| eprintln!("mic error: {e}");
    let stream = match c.sample_format() {
        cpal::SampleFormat::F32 => dev.build_input_stream(
            cfg,
            move |d: &[f32], _| {
                let mut g = b.lock().unwrap();
                g.extend(d.chunks(ch).map(|f| f[0]));
            },
            err,
            None,
        )?,
        cpal::SampleFormat::I16 => dev.build_input_stream(
            cfg,
            move |d: &[i16], _| {
                let mut g = b.lock().unwrap();
                g.extend(d.chunks(ch).map(|f| f[0] as f32 / 32768.0));
            },
            err,
            None,
        )?,
        f => return Err(anyhow!("unsupported microphone format {f:?}")),
    };
    stream.play()?;
    Ok((stream, buf, fs))
}

/// `airlow mictest [seconds]`: no Bluetooth. Record the microphone and report the clicks it hears.
pub fn mic_selftest(secs: u64) -> Result<()> {
    let hint = std::env::var("AIRLOW_MIC").unwrap_or_else(|_| "yeti".into());
    let (stream, buf, fs) = start_mic_recording(&hint)?;
    std::thread::sleep(std::time::Duration::from_secs(secs));
    drop(stream);
    let rec = buf.lock().unwrap().clone();
    let clicks = find_clicks(&rec, fs);
    let peak = rec.iter().fold(0f32, |a, b| a.max(b.abs()));
    println!("recorded {:.1} s, peak {:.3}; {} click events heard", rec.len() as f64 / fs, peak, clicks.len());
    if peak < 0.03 {
        println!("microphone level is very low (peak {peak:.3}): raise the Yeti gain knob / Windows input volume and move it closer");
    }
    for (t, s) in clicks.iter().take(12) {
        println!("  click at {t:7.3} s, strength {s:.3}");
    }
    Ok(())
}

/// Print the measurement report for a finished recording.
pub fn report(rec: &[f32], fs: f64) {
    let peak = rec.iter().fold(0f32, |a, b| a.max(b.abs()));
    println!("recorded {:.1} s, peak level {:.3}", rec.len() as f64 / fs, peak);
    if peak < 0.03 {
        println!("microphone level is very low (peak {peak:.3}): raise the Yeti gain knob / Windows input volume and move it closer");
    }
    let clicks = find_clicks(rec, fs);
    let delays = pair_delays(&clicks);
    println!("found {} click events, {} usable speaker/AirPods pairs", clicks.len(), delays.len());
    for (i, d) in delays.iter().enumerate() {
        println!("  pair {:2}: AirPods click {:6.1} ms after the speaker click", i + 1, d);
    }
    match summarize(&delays) {
        Some((lo, med, hi)) => println!("AirPods are {med:.1} ms behind the speakers (range {lo:.1} .. {hi:.1} ms)"),
        None => println!("no usable pairs: move the AirPods right next to the microphone and keep the room quiet"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A recording with a click train: speaker click at t = k + 0.20 s, AirPods click `lag_ms` later and quieter.
    fn synth(fs: f64, secs: usize, lag_ms: f64, air_gain: f32, noise: f32) -> Vec<f32> {
        let tpl = click_template(fs);
        let mut rec = vec![0f32; (fs * secs as f64) as usize];
        let mut seed = 12345u32;
        for v in rec.iter_mut() {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            *v = ((seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 2.0 * noise;
        }
        for k in 1..secs {
            let a = ((k as f64 + 0.2) * fs) as usize;
            let b = a + (lag_ms / 1000.0 * fs).round() as usize;
            for (i, t) in tpl.iter().enumerate() {
                rec[a + i] += t * 0.8;
                rec[b + i] += t * air_gain;
            }
        }
        rec
    }

    #[test]
    fn measures_a_known_delay_to_a_fraction_of_a_millisecond() {
        for lag in [12.0, 37.3, 64.0, 120.5] {
            let rec = synth(48000.0, 9, lag, 0.3, 0.01);
            let d = pair_delays(&find_clicks(&rec, 48000.0));
            let (lo, med, hi) = summarize(&d).expect("pairs");
            assert!(d.len() >= 7, "lag {lag}: only {} pairs", d.len());
            assert!(
                (med - lag).abs() < 0.3 && (lo - lag).abs() < 0.5 && (hi - lag).abs() < 0.5,
                "lag {lag}: measured {lo:.2}/{med:.2}/{hi:.2}"
            );
        }
    }

    #[test]
    fn works_at_other_sample_rates_and_when_the_airpods_click_is_louder() {
        for fs in [44100.0, 96000.0] {
            let rec = synth(fs, 7, 45.0, 1.4, 0.02);
            let (_, med, _) = summarize(&pair_delays(&find_clicks(&rec, fs))).expect("pairs");
            assert!((med - 45.0).abs() < 0.4, "fs {fs}: {med}");
        }
    }

    #[test]
    fn pure_noise_has_no_clicks() {
        let mut seed = 99u32;
        let rec: Vec<f32> = (0..480_000)
            .map(|_| {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                ((seed >> 8) as f32 / (1u32 << 24) as f32 - 0.5) * 0.012
            })
            .collect();
        assert!(find_clicks(&rec, 48000.0).is_empty(), "noise must not look like clicks");
    }

    #[test]
    fn silence_and_noise_produce_no_pairs() {
        let rec = synth(48000.0, 6, 40.0, 0.0, 0.01);
        // AirPods click absent: speaker clicks only, so no pair may be invented from noise.
        assert!(pair_delays(&find_clicks(&rec, 48000.0)).is_empty());
        assert!(summarize(&[]).is_none());
    }
}

//! Live system-audio path: WASAPI loopback capture -> SBC -> radio, event driven.

use crate::a2dp::{Link, StreamOpts, open_stream};
use crate::{proto, sbc};
use anyhow::{Result, anyhow, bail};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Interleaved s16 stereo samples, produced by the capture callback (or a test).
pub type Q = Arc<Mutex<VecDeque<i16>>>;

pub fn loopback_rate() -> Result<sbc::Rate> {
    let dev = cpal::default_host().default_output_device().ok_or_else(|| anyhow!("no default output device"))?;
    let c = dev.default_output_config()?;
    match c.sample_rate() {
        48000 => Ok(sbc::Rate::Hz48000),
        44100 => Ok(sbc::Rate::Hz44100),
        r => bail!("default output device runs at {r} Hz; set it to 44100 or 48000 in Windows sound settings"),
    }
}

/// Capture-layer glitches (WASAPI "buffer underrun or overrun") since start: each is a break in the audio.
pub static CAPTURE_XRUNS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

pub(crate) fn start_capture(q: Q) -> Result<cpal::Stream> {
    let dev = cpal::default_host().default_output_device().ok_or_else(|| anyhow!("no default output device"))?;
    let c = dev.default_output_config()?;
    let ch = c.channels() as usize;
    let cfg: cpal::StreamConfig = c.clone().into();
    println!("capturing loopback of '{}' ({} ch, {} Hz, {:?})", dev.description()?.name(), ch, c.sample_rate(), c.sample_format());
    let err = |e| {
        CAPTURE_XRUNS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        eprintln!("capture error: {e}");
    };
    let stream = match c.sample_format() {
        cpal::SampleFormat::F32 => {
            dev.build_input_stream(cfg.clone(), move |d: &[f32], _| push(&q, d, ch, |s| (s.clamp(-1.0, 1.0) * 32767.0) as i16), err, None)?
        }
        cpal::SampleFormat::I16 => dev.build_input_stream(cfg.clone(), move |d: &[i16], _| push(&q, d, ch, |s| s), err, None)?,
        f => bail!("unsupported capture format {f:?}"),
    };
    stream.play()?;
    Ok(stream)
}

fn push<T: Copy>(q: &Q, d: &[T], ch: usize, conv: impl Fn(T) -> i16) {
    let mut g = q.lock().unwrap();
    for f in d.chunks(ch) {
        let l = conv(f[0]);
        let r = if ch > 1 { conv(f[1]) } else { l };
        g.push_back(l);
        g.push_back(r);
    }
}

pub fn stream_live(l: &mut Link, o: &StreamOpts) -> Result<()> {
    let (sig_cid, seid, media_cid) = open_stream(l, o)?;
    let q: Q = Arc::new(Mutex::new(VecDeque::new()));
    let _cap = start_capture(q.clone())?;
    let s = live_loop(l, o, &q, sig_cid, seid, media_cid)?;
    println!("  total: sent {} pkts, silence {}, dropped(backlog) {}", s.sent, s.silent, s.dropped);
    Ok(())
}

#[derive(Debug, Default, Clone, Copy)]
pub struct LiveStats {
    pub sent: u64,
    pub silent: u64,
    pub dropped: u64,
    pub max_queue_ms: f64,
    pub max_frames_per_packet: usize,
}

/// Most frames bundled into one packet when the controller cannot keep up with small ones.
const MAX_FRAMES_PER_PACKET: usize = 8;

/// The event-driven core: drains `q`, encodes and sends audio as soon as a controller buffer is free.
/// Separate from capture so it can be driven by synthetic audio in tests.
///
/// Packet size adapts: the controller completes only ~300-360 ACL packets/s no matter their size, so
/// when it falls behind we wait for a free buffer *first* and then pack every ready frame (up to
/// [`MAX_FRAMES_PER_PACKET`]) into it. A sink that receives audio slower than real time starves and
/// outputs nothing, so keeping up is more important than the last millisecond of packetisation delay.
pub fn live_loop(l: &mut Link, o: &StreamOpts, q: &Q, sig_cid: u16, seid: u8, media_cid: u16) -> Result<LiveStats> {
    live_loop_ex(l, o, q, sig_cid, seid, media_cid, LoopEx::default())
}

/// Extras for A/B sessions: how long to run, audible start markers (muted gaps), and whether to Suspend at the end.
#[derive(Clone, Copy)]
pub struct LoopEx {
    pub secs: Option<u64>,
    pub start_gaps: usize,
    pub suspend_at_end: bool,
}

impl Default for LoopEx {
    fn default() -> Self {
        Self { secs: None, start_gaps: 0, suspend_at_end: true }
    }
}

/// Mute windows: `gaps` quarter-second silences half a second apart, starting now. Timing is preserved (the
/// muted audio is replaced by zeros of the same duration, never removed), so a marker adds no lag.
pub fn gap_markers(gaps: usize) -> Vec<(Instant, Instant)> {
    let now = Instant::now();
    (0..gaps)
        .map(|k| {
            let a = now + Duration::from_millis(500 * k as u64);
            (a, a + Duration::from_millis(250))
        })
        .collect()
}

#[allow(clippy::too_many_arguments)]
pub fn live_loop_ex(l: &mut Link, o: &StreamOpts, q: &Q, sig_cid: u16, seid: u8, media_cid: u16, ex: LoopEx) -> Result<LiveStats> {
    let mut media_cid = media_cid;
    let mut enc = sbc::Encoder::new(o.cfg);
    let mut spf = enc.samples_per_frame();
    let mut codesize = enc.codesize();
    let hz = if matches!(o.cfg.rate, sbc::Rate::Hz48000) { 48000.0 } else { 44100.0 };
    let mut fpp_min = o.frames_per_packet.clamp(1, MAX_FRAMES_PER_PACKET);
    let mut frame_dur = Duration::from_secs_f64(spf as f64 / hz);
    let mut flen = enc.frame_len();
    println!(
        "LIVE: {fpp_min}..{MAX_FRAMES_PER_PACKET} frame(s)/packet (adaptive), min packet {:.2} ms, flush {} ms. Play something in Windows.",
        (frame_dur * fpp_min as u32).as_secs_f64() * 1000.0,
        o.flush_ms
    );

    let end = Instant::now() + Duration::from_secs(ex.secs.unwrap_or(o.seconds as u64));
    let (mut seq, mut ts) = (0u16, 0u32);
    let mut pcm = vec![0u8; codesize];
    let mut fr = vec![0u8; 1024];
    let mut last_data = Instant::now();
    let mut silence_clock = Instant::now(); // wall time up to which silence has been sent
    let mut was_silent = false;
    let max_backlog = (hz * 0.100) as usize * 2; // capture burst limit: only a real stall (>100 ms) is trimmed
    let mut stats = LiveStats::default();
    let (mut maxq, mut maxq_total, mut max_fpp) = (0usize, 0usize, 0usize);
    let mut report = Instant::now() + Duration::from_secs(5);
    // AIRLOW_SWEEP=1: cycle the flush timeout (40 ms, 120 ms, none) every 20 s to compare what each sounds like.
    let sweep_mode = std::env::var("AIRLOW_SWEEP").unwrap_or_default();
    let sweep = sweep_mode == "flush" || sweep_mode == "1";
    let pkt_sweep = sweep_mode == "pkt"; // sending patterns: adaptive bursts, then steady 2-, 4- and 8-frame beats
    let pkt_phases: [Option<usize>; 4] = [None, Some(2), Some(4), Some(8)];
    // AIRLOW_SWEEP=cfg: reconfigure the live stream (AVDTP Suspend/Reconfigure/Start) through five SBC settings.
    let cfg_sweep = sweep_mode == "cfg";
    let cfg_phases = [
        (sbc::Mode::JointStereo, 8u8, 53u8, 3usize),
        (sbc::Mode::Stereo, 8, 53, 3),
        (sbc::Mode::JointStereo, 16, 53, 2),
        (sbc::Mode::Stereo, 16, 53, 2),
        (sbc::Mode::JointStereo, 16, 32, 2),
    ];
    let mut fixed: Option<usize> = None;
    let mut pace_next = Instant::now();
    let (t_begin, mut phase) = (Instant::now(), usize::MAX);
    // Audible phase markers: brief muted windows (1 gap = phase 2, 2 gaps = phase 3); timing is preserved.
    let mut mute: Vec<(Instant, Instant)> = gap_markers(ex.start_gaps);
    let (mut c_prev, mut f_prev) = (l.completed, l.flushed);
    while Instant::now() < end {
        if cfg_sweep {
            let p = ((t_begin.elapsed().as_secs() / 18) as usize).min(cfg_phases.len() - 1);
            if p != phase {
                phase = p;
                let (mode, blocks, bitpool, fmin) = cfg_phases[p];
                let newcfg = sbc::Config { mode, blocks, bitpool, ..o.cfg };
                if p > 0 {
                    media_cid = crate::a2dp::restart_with(l, sig_cid, seid, media_cid, &newcfg)?;
                }

                enc = sbc::Encoder::new(newcfg);
                spf = enc.samples_per_frame();
                codesize = enc.codesize();
                flen = enc.frame_len();
                frame_dur = Duration::from_secs_f64(spf as f64 / hz);
                pcm = vec![0u8; codesize];
                fpp_min = fmin;
                let now = Instant::now();
                for k in 0..p {
                    let a = now + Duration::from_millis(500 * k as u64);
                    mute.push((a, a + Duration::from_millis(250)));
                }
                println!(
                    "== PHASE {}: {:?}, {} blocks, bitpool {} -> {} B frames, ~{:.0} kbps (capture xruns so far: {})",
                    p + 1,
                    mode,
                    blocks,
                    bitpool,
                    flen,
                    flen as f64 * 8.0 / frame_dur.as_secs_f64() / 1000.0,
                    CAPTURE_XRUNS.load(std::sync::atomic::Ordering::Relaxed)
                );
            }
        }
        if pkt_sweep {
            let p = ((t_begin.elapsed().as_secs() / 15) as usize).min(pkt_phases.len() - 1);
            if p != phase {
                phase = p;
                fixed = pkt_phases[p];
                pace_next = Instant::now();
                let now = Instant::now();
                for k in 0..p {
                    let a = now + Duration::from_millis(500 * k as u64);
                    mute.push((a, a + Duration::from_millis(250)));
                }
                match fixed {
                    Some(n) => println!(
                        "== PHASE {}: steady {} frames/packet = {:.1} ms beat",
                        p + 1,
                        n,
                        (frame_dur * n as u32).as_secs_f64() * 1000.0
                    ),
                    None => println!("== PHASE {}: adaptive bursts (current default)", p + 1),
                }
            }
        }
        if sweep {
            let p = ((t_begin.elapsed().as_secs() / 20) as usize).min(2);
            if p != phase {
                phase = p;
                let ms = [40.0, 120.0, 0.0][p];
                l.set_flush_ms(ms)?;
                let now = Instant::now();
                for k in 0..p {
                    let a = now + Duration::from_millis(500 * k as u64);
                    mute.push((a, a + Duration::from_millis(250)));
                }
                println!(
                    "== PHASE {}: flush timeout {} ms  (capture xruns so far: {})",
                    p + 1,
                    if ms == 0.0 { "none".to_string() } else { ms.to_string() },
                    CAPTURE_XRUNS.load(std::sync::atomic::Ordering::Relaxed)
                );
            }
        }
        maxq_total = maxq_total.max(maxq);
        l.pump(Duration::ZERO);
        l.service()?; // keep answering the sink's AVRCP/SDP/AVDTP traffic
        if l.disconnected {
            bail!("link dropped");
        }
        // Wait for a controller buffer BEFORE choosing the packet size, so backlog becomes bigger packets.
        if !l.has_credit() {
            l.pump(Duration::from_millis(1));
            continue;
        }
        // Fixed-size mode: one packet of exactly N frames per N-frame beat.
        if fixed.is_some() && Instant::now() < pace_next {
            std::thread::yield_now();
            continue;
        }
        let (lo, hi) = match fixed {
            Some(n) => (n, n),
            None => (fpp_min, MAX_FRAMES_PER_PACKET),
        };
        let mut payload = Vec::with_capacity(flen * MAX_FRAMES_PER_PACKET);
        let mut frames = 0usize;
        {
            let mut g = q.lock().unwrap();
            maxq = maxq.max(g.len());
            while g.len() > max_backlog {
                for _ in 0..fpp_min * spf * 2 {
                    g.pop_front();
                }
                stats.dropped += 1;
            }
            let avail = g.len() / (spf * 2);
            if avail >= lo {
                frames = avail.min(hi);
                for _ in 0..frames {
                    let muted = !mute.is_empty() && {
                        let now = Instant::now();
                        mute.iter().any(|(a, b)| now >= *a && now < *b)
                    };
                    for s in 0..spf * 2 {
                        let v = g.pop_front().unwrap();
                        let v = if muted { 0 } else { v };
                        pcm[s * 2..s * 2 + 2].copy_from_slice(&v.to_le_bytes());
                    }
                    let n = enc.encode_frame(&pcm, &mut fr);
                    payload.extend_from_slice(&fr[..n]);
                }
                last_data = Instant::now();
                was_silent = false;
            }
        }
        if frames == 0 {
            // Loopback is silent when nothing plays; keep the sink fed with silence on a real-time clock.
            let now = Instant::now();
            if last_data.elapsed() > Duration::from_millis(30) {
                if !was_silent {
                    silence_clock = now; // start the clock at the moment silence begins
                    was_silent = true;
                }
                let due = (now.duration_since(silence_clock).as_secs_f64() / frame_dur.as_secs_f64()) as usize;
                if due >= lo {
                    frames = due.min(hi);
                    pcm.fill(0);
                    for _ in 0..frames {
                        let n = enc.encode_frame(&pcm, &mut fr);
                        payload.extend_from_slice(&fr[..n]);
                    }
                    silence_clock += frame_dur * frames as u32;
                    stats.silent += 1;
                }
            }
            if frames == 0 {
                // Poll hot while audio is flowing (lowest latency); back off when idle.
                if last_data.elapsed() < Duration::from_millis(100) {
                    std::thread::yield_now();
                } else {
                    std::thread::sleep(Duration::from_micros(500));
                }
                continue;
            }
        }
        max_fpp = max_fpp.max(frames);
        if fixed.is_some() {
            pace_next += frame_dur * frames as u32;
            if pace_next + Duration::from_millis(30) < Instant::now() {
                pace_next = Instant::now(); // fell far behind (e.g. idle): resync the beat
            }
        }
        let pkt = proto::rtp_sbc(seq, ts, 0xA1D10, frames as u8, &payload);
        l.send(media_cid, &pkt, true)?;
        seq = seq.wrapping_add(1);
        ts = ts.wrapping_add((frames * spf) as u32);
        stats.sent += 1;
        if Instant::now() >= report {
            println!(
                "  sent {} pkts, silence {}, dropped(backlog) {}, max capture queue {:.1} ms, max frames/packet {max_fpp}",
                stats.sent,
                stats.silent,
                stats.dropped,
                maxq as f64 / 2.0 / hz * 1000.0
            );
            println!(
                "  radio: +{} completed, +{} flushed (dropped) in 5 s; {} in flight; capture xruns {}",
                l.completed - c_prev,
                l.flushed - f_prev,
                l.outstanding(),
                CAPTURE_XRUNS.load(std::sync::atomic::Ordering::Relaxed)
            );
            println!("  link: {}", l.link_stats());
            (c_prev, f_prev) = (l.completed, l.flushed);
            maxq_total = maxq_total.max(maxq);
            maxq = 0;
            report = Instant::now() + Duration::from_secs(5);
        }
    }
    if ex.suspend_at_end {
        let _ = l.avdtp(sig_cid, 6, proto::AVDTP_SUSPEND, &[seid << 2]);
        println!("done");
    }
    stats.max_queue_ms = maxq_total as f64 / 2.0 / hz * 1000.0;
    stats.max_frames_per_packet = max_fpp;
    Ok(stats)
}
/// `airlow captest`: no Bluetooth. Capture Windows' system audio for a few seconds and report what the
/// loopback delivers (rate, burst size/period, level), so the live path's assumptions can be checked.
pub fn capture_selftest(secs: u64) -> Result<()> {
    let q: Q = Arc::new(Mutex::new(VecDeque::new()));
    let _cap = start_capture(q.clone())?;
    let end = Instant::now() + Duration::from_secs(secs);
    let mut bursts: Vec<(Instant, usize)> = Vec::new();
    let (mut last_len, mut total, mut peak) = (0usize, 0usize, 0i16);
    let mut keep: VecDeque<i16> = VecDeque::new();
    while Instant::now() < end {
        {
            let mut g = q.lock().unwrap();
            if g.len() > 0 {
                bursts.push((Instant::now(), g.len()));
                total += g.len();
                for v in g.drain(..) {
                    peak = peak.max(v.saturating_abs());
                    if keep.len() < 4 {
                        keep.push_back(v);
                    }
                }
            }
        }
        last_len = last_len.max(1);
        std::thread::sleep(Duration::from_micros(300));
    }
    let n = bursts.len();
    println!("captured {} stereo frames in {} s ({} bursts)", total / 2, secs, n);
    if n >= 2 {
        let span = bursts.last().unwrap().0.duration_since(bursts[0].0).as_secs_f64();
        let avg_frames = bursts.iter().map(|b| b.1 / 2).sum::<usize>() as f64 / n as f64;
        println!("burst period ~{:.1} ms, ~{:.0} frames/burst", span / (n - 1) as f64 * 1000.0, avg_frames);
    }
    println!(
        "peak level {} / 32767 ({:.1} dBFS){}",
        peak,
        20.0 * (peak.max(1) as f64 / 32767.0).log10(),
        if peak == 0 { "  <- SILENCE: nothing is playing in Windows" } else { "" }
    );
    Ok(())
}

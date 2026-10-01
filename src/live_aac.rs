//! Live path for the optional AAC mode: drain captured PCM, feed the Windows AAC encoder, and send each encoded
//! frame as its own LATM/RTP packet. Mirrors the SBC loop's behaviour (credit-aware, keep-alive silence on a
//! real-time clock, audible gap markers) so the two can be A/B tested by ear.

#![cfg(windows)]

use crate::a2dp::{Link, StreamOpts};
use crate::aac::{self, AacEncoder};
use crate::live::{LiveStats, LoopEx, Q, gap_markers};
use crate::{proto, sbc};
use anyhow::{Result, bail};
use std::time::{Duration, Instant};

/// `enc` must be created (ideally with [`AacEncoder::new_primed`]) BEFORE the stream is started.
pub fn live_loop_aac(l: &mut Link, o: &StreamOpts, q: &Q, sig_cid: u16, seid: u8, media_cid: u16, enc: AacEncoder) -> Result<LiveStats> {
    live_loop_aac_ex(l, o, q, sig_cid, seid, media_cid, enc, LoopEx::default())
}

/// A primed encoder for the stream described by `o` (see [`AacEncoder::new_primed`]).
pub fn make_encoder(o: &StreamOpts) -> Result<AacEncoder> {
    let rate = match o.cfg.rate {
        sbc::Rate::Hz48000 => 48000u32,
        sbc::Rate::Hz44100 => 44100,
    };
    AacEncoder::new_primed(rate, 2, o.aac_bitrate)
}

#[allow(clippy::too_many_arguments)]
pub fn live_loop_aac_ex(
    l: &mut Link,
    o: &StreamOpts,
    q: &Q,
    sig_cid: u16,
    seid: u8,
    media_cid: u16,
    mut enc: AacEncoder,
    ex: LoopEx,
) -> Result<LiveStats> {
    let rate = match o.cfg.rate {
        sbc::Rate::Hz48000 => 48000u32,
        sbc::Rate::Hz44100 => 44100,
    };
    let hz = rate as f64;
    let ri = aac::rate_index(rate).expect("A2DP rates are all valid AAC rates");
    println!(
        "LIVE (AAC): {} kbit/s, one frame ({:.1} ms) per packet, flush {} ms. Play something in Windows.",
        o.aac_bitrate / 1000,
        aac::FRAME_SAMPLES as f64 / hz * 1000.0,
        o.flush_ms
    );

    crate::live::discard_stale(q);
    let end = Instant::now() + Duration::from_secs(ex.secs.unwrap_or(o.seconds as u64));
    let (mut seq, mut ts) = (0u16, 0u32);
    let mut last_data = Instant::now();
    let mut silence_clock = Instant::now();
    let mut was_silent = false;
    let max_backlog = (hz * 0.100) as usize * 2;
    let mute = gap_markers(ex.start_gaps);
    let mut stats = LiveStats { max_frames_per_packet: 1, ..Default::default() };
    let (mut maxq, mut maxq_total) = (0usize, 0usize);
    let mut report = Instant::now() + Duration::from_secs(5);
    let (mut c_prev, mut f_prev) = (l.completed, l.flushed);
    while Instant::now() < end {
        l.pump(Duration::ZERO);
        l.service()?;
        if l.disconnected {
            bail!("link dropped");
        }
        if !l.has_credit() {
            l.pump(Duration::from_millis(1));
            continue;
        }
        // Everything the capture side has produced so far.
        let mut pcm: Vec<i16> = Vec::new();
        {
            let mut g = q.lock().unwrap();
            maxq = maxq.max(g.len());
            while g.len() > max_backlog {
                g.drain(..480 * 2);
                stats.dropped += 1;
            }
            if !g.is_empty() {
                pcm.extend(g.drain(..));
                last_data = Instant::now();
                was_silent = false;
            }
        }
        if !pcm.is_empty() {
            let now = Instant::now();
            if mute.iter().any(|(a, b)| now >= *a && now < *b) {
                pcm.fill(0);
            }
        } else if last_data.elapsed() > Duration::from_millis(30) {
            // Loopback is silent when nothing plays; keep the sink fed with silence on a real-time clock.
            let now = Instant::now();
            if !was_silent {
                silence_clock = now;
                was_silent = true;
            }
            let due = (now.duration_since(silence_clock).as_secs_f64() * hz) as usize;
            if due >= 480 {
                pcm = vec![0i16; due * 2];
                silence_clock += Duration::from_secs_f64(due as f64 / hz);
                stats.silent += 1;
            }
        }
        if pcm.is_empty() {
            if last_data.elapsed() < Duration::from_millis(100) {
                std::thread::yield_now();
            } else {
                std::thread::sleep(Duration::from_micros(500));
            }
            continue;
        }
        for frame in enc.encode(&pcm)? {
            let latm = aac::latm_mux(ri, 2, &frame);
            let pkt = proto::rtp_aac(seq, ts, 0xA1D10, &latm);
            l.send(media_cid, &pkt, true)?;
            seq = seq.wrapping_add(1);
            ts = ts.wrapping_add(aac::FRAME_SAMPLES as u32);
            stats.sent += 1;
        }
        if Instant::now() >= report {
            println!(
                "  sent {} AAC packets, silence feeds {}, dropped(backlog) {}, max capture queue {:.1} ms",
                stats.sent,
                stats.silent,
                stats.dropped,
                maxq as f64 / 2.0 / hz * 1000.0
            );
            println!(
                "  radio: +{} completed, +{} flushed (dropped) in 5 s; {} in flight",
                l.completed - c_prev,
                l.flushed - f_prev,
                l.outstanding()
            );
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
    stats.max_queue_ms = maxq_total.max(maxq) as f64 / 2.0 / hz * 1000.0;
    Ok(stats)
}

/// `AIRLOW_SWEEP=codec`: one session alternating SBC and AAC (15 s each: SBC, AAC, SBC, AAC) with audible gap
/// markers (none, 1, 2, 3 short silences at the start of each phase) so the codecs can be compared by ear.
pub fn codec_sweep(l: &mut Link, base: &StreamOpts) -> Result<()> {
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};
    let q: Q = Arc::new(Mutex::new(VecDeque::new()));
    let _cap = crate::live::start_capture(q.clone())?; // capture first: nothing slow after Start
    codec_sweep_q(l, base, &q, 15)
}

/// The sweep with an injectable audio queue and phase length (real capture in production, a tone in tests).
pub fn codec_sweep_q(l: &mut Link, base: &StreamOpts, q: &Q, phase_secs: u64) -> Result<()> {
    use crate::a2dp::{self, Codec};
    use crate::live;

    let sbc_opts = StreamOpts { codec: Codec::Sbc, cfg: base.cfg, ..*base };
    let aac_opts = StreamOpts { codec: Codec::Aac, cfg: base.cfg, ..*base };
    let (sig, mut seid, mut media) = a2dp::open_stream(l, &sbc_opts)?;
    let phases = [Codec::Sbc, Codec::Aac, Codec::Sbc, Codec::Aac];
    println!("CODEC SWEEP: 4 phases of {phase_secs} s: SBC, AAC, SBC, AAC. Gap markers at the start: none, 1, 2, 3.");
    for (i, codec) in phases.iter().enumerate() {
        let last = i == phases.len() - 1;
        let mut enc = None;
        if i > 0 {
            let o = if *codec == Codec::Aac { &aac_opts } else { &sbc_opts };
            // The AAC encoder is built right after the Suspend and before anything restarts.
            let (s2, m2) = a2dp::switch_codec(l, sig, seid, media, o, || {
                if *codec == Codec::Aac {
                    enc = Some(make_encoder(&aac_opts)?);
                }
                Ok(())
            })?;
            seid = s2;
            media = m2;
        } else if *codec == Codec::Aac {
            enc = Some(make_encoder(&aac_opts)?);
        }
        println!("== PHASE {}: {}", i + 1, if *codec == Codec::Aac { "AAC" } else { "SBC" });
        let ex = LoopEx { secs: Some(phase_secs), start_gaps: i, suspend_at_end: last };
        if *codec == Codec::Aac {
            live_loop_aac_ex(l, &aac_opts, q, sig, seid, media, enc.take().expect("encoder built"), ex)?;
        } else {
            live::live_loop_ex(l, &sbc_opts, q, sig, seid, media, ex)?;
        }
    }
    Ok(())
}

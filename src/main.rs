//! airlow command-line tool.

use airlow::hci::{Hci, fmt_addr};
use airlow::{a2dp, latency, live, lowlat, session::*, sim};
use anyhow::Result;

fn main() -> Result<()> {
    realtime_tuning();
    if std::env::args().nth(1).as_deref() == Some("simlive") {
        let secs = std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(8);
        let hz = std::env::args().nth(3).and_then(|s| s.parse().ok()).unwrap_or(1000.0);
        return sim::simlive(secs, hz);
    }
    if std::env::args().nth(1).as_deref() == Some("mictest") {
        return latency::mic_selftest(std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(8));
    }
    if std::env::args().nth(1).as_deref() == Some("audiocaps") {
        return lowlat::probe();
    }
    if std::env::args().nth(1).as_deref() == Some("captest") {
        return live::capture_selftest(std::env::args().nth(2).and_then(|s| s.parse().ok()).unwrap_or(3));
    }
    let (vid, pid) = usb_id();
    println!("controller USB id {vid:04x}:{pid:04x}");
    let mut h = Hci::open(vid, pid)?;
    match std::env::args().nth(1).as_deref() {
        Some("pair") => {
            let hd = pair(&mut h, &key_path())?;
            if std::env::args().nth(2).as_deref() == Some("exp") {
                let mut l = a2dp::Link::new(&mut h, hd)?;
                a2dp::experiments(&mut l, &tone_opts(3))?;
            } else if std::env::args().nth(2).as_deref() == Some("latency") {
                // airlow pair latency [blocks] [frames_per_packet] [flush_ms] [bitpool] [seconds=40]
                // Streams like `live` while a microphone (default: a device named "yeti", override with
                // AIRLOW_MIC) records; play a click track in Windows meanwhile (see docs/TESTING.md).
                let mut o = tone_opts(3);
                o.cfg.rate = live::loopback_rate()?;
                if std::env::args().nth(4).is_none() {
                    o.frames_per_packet = DEFAULT_LIVE_FRAMES_PER_PACKET;
                }
                if std::env::args().nth(7).is_none() {
                    o.seconds = 40;
                }
                let hint = std::env::var("AIRLOW_MIC").unwrap_or_else(|_| "yeti".into());
                let (mic, buf, fs) = latency::start_mic_recording(&hint)?;
                let mut l = a2dp::Link::new(&mut h, hd)?;
                live::stream_live(&mut l, &o)?;
                drop(mic);
                let rec = buf.lock().unwrap().clone();
                latency::report(&rec, fs);
            } else if std::env::args().nth(2).as_deref() == Some("live") {
                // airlow pair live [blocks=16] [frames_per_packet=2] [flush_ms=40] [bitpool=53] [seconds=120]
                let mut o = tone_opts(3);
                o.cfg.rate = live::loopback_rate()?;
                if std::env::args().nth(4).is_none() {
                    o.frames_per_packet = DEFAULT_LIVE_FRAMES_PER_PACKET;
                }
                if std::env::args().nth(7).is_none() {
                    o.seconds = 120;
                }
                let mut l = a2dp::Link::new(&mut h, hd)?;
                live::stream_live(&mut l, &o)?;
            } else if std::env::args().nth(2).as_deref() == Some("tone") {
                let mut l = a2dp::Link::new(&mut h, hd)?;
                a2dp::stream_tone(&mut l, &tone_opts(3))?;
            }
            Ok(())
        }
        Some("play") => {
            // The everyday command: reconnect with the saved key if possible, otherwise scan for pairing mode,
            // then stream Windows audio until the process is stopped (Ctrl+C).
            let key = key_path();
            let mut o = tone_opts(2);
            o.cfg.rate = live::loopback_rate()?;
            o.frames_per_packet = DEFAULT_LIVE_FRAMES_PER_PACKET;
            o.seconds = 30 * 86_400;
            println!("airlow: wear your AirPods. If they do not connect on their own, put them in pairing mode");
            println!("(case open, hold the back button until the light flashes white). Press Ctrl+C to stop.");
            let handle = if key.exists() {
                init(&mut h)?;
                let (addr, k) = load_key(&key)?;
                println!("Trying the saved pairing with {}...", fmt_addr(&addr));
                match a2dp::connect_with_retry(&mut h, &addr, &k, init) {
                    Ok(hd) => {
                        println!("Reconnected.");
                        hd
                    }
                    Err(e) => {
                        println!("Could not reconnect ({e}). Scanning for AirPods in pairing mode...");
                        pair(&mut h, &key)?
                    }
                }
            } else {
                pair(&mut h, &key)?
            };
            let mut l = a2dp::Link::new(&mut h, handle)?;
            live::stream_live(&mut l, &o)
        }
        Some("tone") => tone(&mut h),
        Some("exp") => {
            let o = tone_opts(2);
            let (addr, key) = load_key(&key_path())?;
            init(&mut h)?;
            println!("Connecting to {}...", fmt_addr(&addr));
            let handle = a2dp::connect_with_retry(&mut h, &addr, &key, init)?;
            let mut l = a2dp::Link::new(&mut h, handle)?;
            a2dp::experiments(&mut l, &o)
        }
        Some("live") => {
            // airlow live [blocks=8] [frames_per_packet=2] [flush_ms=40] [bitpool=53] [seconds=600]
            let mut o = tone_opts(2);
            o.cfg.rate = live::loopback_rate()?;
            if std::env::args().nth(3).is_none() {
                o.frames_per_packet = DEFAULT_LIVE_FRAMES_PER_PACKET;
            }
            if std::env::args().nth(6).is_none() {
                o.seconds = 600;
            }
            let (addr, key) = load_key(&key_path())?;
            init(&mut h)?;
            let handle = a2dp::connect_with_retry(&mut h, &addr, &key, init)?;
            let mut l = a2dp::Link::new(&mut h, handle)?;
            live::stream_live(&mut l, &o)
        }
        _ => {
            h.cmd(0x0C03, &[])?;
            info(&mut h)
        }
    }
}

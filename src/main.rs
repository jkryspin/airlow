//! airlow: low-latency user-mode Bluetooth audio stack.

mod a2dp;
mod hci;
mod latency;
mod live;
mod proto;
mod sbc;
#[allow(dead_code)]
mod sim;

use anyhow::{Result, bail};
use hci::{Addr, Hci, fmt_addr};
use std::time::{Duration, Instant};

/// Default controller: the MediaTek module on this machine. Override with AIRLOW_USB=vvvv:pppp (hex)
/// to use another WinUSB-bound Bluetooth adapter.
fn usb_id() -> (u16, u16) {
    if let Ok(s) = std::env::var("AIRLOW_USB") {
        if let Some((v, p)) = s.split_once([':', '/']) {
            if let (Ok(v), Ok(p)) = (u16::from_str_radix(v.trim(), 16), u16::from_str_radix(p.trim(), 16)) {
                return (v, p);
            }
        }
        eprintln!("ignoring malformed AIRLOW_USB={s:?} (expected e.g. 0bda:8771)");
    }
    (0x13d3, 0x3602)
}

fn init(h: &mut Hci) -> Result<()> {
    h.cmd(0x0C03, &[])?; // Reset
    h.drain();
    h.cmd(0x0C1A, &[0])?; // Write_Scan_Enable: none
    h.cmd(0x080F, &[0, 0])?; // Write_Default_Link_Policy_Settings: no role switch, no sniff
    h.cmd(0x0C01, &[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F])?; // Set_Event_Mask
    h.cmd(0x0C56, &[1])?; // Write_Simple_Pairing_Mode
    h.cmd(0x0C45, &[2])?; // Write_Inquiry_Mode: extended
    h.cmd(0x0C24, &[0x14, 0x04, 0x20])?; // CoD: audio / hi-fi
    Ok(())
}

fn info(h: &mut Hci) -> Result<()> {
    let v = h.cmd(0x1001, &[])?;
    println!("HCI version {} (13 = BT 5.4), LMP {}, manufacturer {}", v[1], v[4], u16::from_le_bytes([v[5], v[6]]));
    let a = h.cmd(0x1009, &[])?;
    println!("BD_ADDR = {}", fmt_addr(&a[1..7].try_into().unwrap()));
    Ok(())
}

struct Found {
    addr: Addr,
    cod: u32,
    name: String,
    rssi: i8,
    psrm: u8,
}

fn eir_name(eir: &[u8]) -> String {
    let mut i = 0;
    while i < eir.len() {
        let l = eir[i] as usize;
        if l == 0 || i + 1 + l > eir.len() {
            break;
        }
        if eir[i + 1] == 0x08 || eir[i + 1] == 0x09 {
            return String::from_utf8_lossy(&eir[i + 2..i + 1 + l]).into_owned();
        }
        i += 1 + l;
    }
    String::new()
}

fn inquire(h: &mut Hci, secs: u8) -> Result<Vec<Found>> {
    h.cmd(0x0401, &[0x33, 0x8B, 0x9E, (secs as f32 / 1.28).ceil() as u8, 0])?;
    let mut out: Vec<Found> = Vec::new();
    let end = Instant::now() + Duration::from_secs(secs as u64 + 3);
    while Instant::now() < end {
        let Some(e) = h.next_event(Duration::from_millis(200)) else { continue };
        match e[0] {
            0x01 => break, // Inquiry Complete
            0x22 | 0x2F => {
                // RSSI result: n x 14 bytes; extended: 1 x (14 + 240)
                let n = e[2] as usize;
                let p = &e[3..];
                let stride = if e[0] == 0x22 { 14 } else { 254 };
                for i in 0..n {
                    let Some(r) = p.get(i * stride..) else { break };
                    if r.len() < 14 {
                        break;
                    }
                    let addr: Addr = r[0..6].try_into().unwrap();
                    let f = Found {
                        addr,
                        cod: u32::from_le_bytes([r[8], r[9], r[10], 0]),
                        name: if e[0] == 0x2F { eir_name(&r[14..]) } else { String::new() },
                        rssi: r[13] as i8,
                        psrm: r[6],
                    };
                    if let Some(x) = out.iter_mut().find(|x| x.addr == addr) {
                        if !f.name.is_empty() {
                            x.name = f.name;
                        }
                    } else {
                        out.push(f);
                    }
                }
            }
            _ => {}
        }
    }
    Ok(out)
}

fn key_path() -> std::path::PathBuf {
    let d = std::path::PathBuf::from(std::env::var("APPDATA").unwrap_or_else(|_| ".".into())).join("airlow");
    let _ = std::fs::create_dir_all(&d);
    d.join("keys.txt")
}

fn save_key(path: &std::path::Path, addr: &Addr, key: &[u8]) {
    let hex: String = key.iter().map(|b| format!("{b:02x}")).collect();
    let _ = std::fs::write(path, format!("{} {}\n", fmt_addr(addr), hex));
}

fn pair(h: &mut Hci, keyfile: &std::path::Path) -> Result<u16> {
    init(h)?;
    let mut found = Vec::new();
    for round in 1..=12 {
        println!("Scanning (round {round}/12) - put AirPods in pairing mode now...");
        found = inquire(h, 8)?;
        for f in &found {
            println!("  {}  CoD {:06x}  {}", fmt_addr(&f.addr), f.cod, f.name);
        }
        if found.iter().any(|f| f.name.contains("AirPods")) {
            break;
        }
    }
    // Audio major class (0x04) devices; prefer name match.
    let target = found.iter().find(|f| f.name.contains("AirPods"));
    let Some(t) = target else { bail!("no audio device found; put AirPods in pairing mode") };
    println!("Connecting to {} ({})", fmt_addr(&t.addr), t.name);

    let mut p = t.addr.to_vec();
    p.extend_from_slice(&0xCC18u16.to_le_bytes()); // DM1/3/5 DH1/3/5
    p.push(t.psrm);
    p.push(0);
    p.extend_from_slice(&[0, 0]); // clock offset invalid
    p.push(0); // do NOT allow role switch: we stay central
    h.cmd(0x0405, &p)?;

    let mut handle: Option<u16> = None;
    let end = Instant::now() + Duration::from_secs(40);
    while Instant::now() < end {
        let Some(e) = h.next_event(Duration::from_millis(200)) else { continue };
        let a = t.addr.to_vec();
        match e[0] {
            0x03 => {
                if e[2] != 0 {
                    bail!("connection failed, status {:#04x}", e[2]);
                }
                let hd = u16::from_le_bytes([e[3], e[4]]);
                handle = Some(hd);
                println!("ACL connected, handle {hd:#05x}");
                h.cmd(0x0411, &hd.to_le_bytes())?; // Authentication_Requested
            }
            0x17 => {
                h.cmd(0x040C, &a)?; // Link key negative reply
            }
            0x16 => {
                h.cmd(0x040E, &a)?; // PIN negative reply
            }
            0x31 => {
                let mut r = a.clone();
                r.extend_from_slice(&[0x03, 0x00, 0x04]); // NoInputNoOutput, no OOB, general bonding
                h.cmd(0x042B, &r)?;
            }
            0x33 => {
                h.cmd(0x042C, &a)?; // User confirm: yes
            }
            0x18 => {
                println!("Link key received (type {})", e[24]);
                save_key(keyfile, &t.addr, &e[8..24]);
            }
            0x36 => println!("Simple pairing complete, status {:#04x}", e[2]),
            0x06 => {
                println!("Authentication complete, status {:#04x}", e[2]);
                if e[2] == 0 {
                    if let Some(hd) = handle {
                        let mut r = hd.to_le_bytes().to_vec();
                        r.push(1);
                        h.cmd(0x0413, &r)?; // Set_Connection_Encryption
                    }
                } else {
                    bail!("pairing failed");
                }
            }
            0x08 => {
                println!("Encryption change, status {:#04x}, enabled {}", e[2], e[5]);
                println!("PAIRED. Key saved to {}", keyfile.display());
                return Ok(handle.unwrap());
            }
            // A disconnect before we hold a handle is the previous process's link reporting late: ignore it.
            0x05 if handle.is_some() => bail!("disconnected, reason {:#04x}", e[5]),
            _ => {}
        }
    }
    bail!("timeout during pairing")
}

fn load_key(path: &std::path::Path) -> Result<(Addr, [u8; 16])> {
    let s = std::fs::read_to_string(path).map_err(|_| anyhow::anyhow!("no saved pairing; run `airlow pair`"))?;
    let mut it = s.split_whitespace();
    let (a, k) = (it.next().unwrap_or(""), it.next().unwrap_or(""));
    let mut addr: Addr = [0; 6];
    for (i, b) in a.split(':').rev().enumerate().take(6) {
        addr[i] = u8::from_str_radix(b, 16)?;
    }
    let mut key = [0u8; 16];
    for i in 0..16 {
        key[i] = u8::from_str_radix(&k[i * 2..i * 2 + 2], 16)?;
    }
    Ok((addr, key))
}

/// SBC blocks per frame. AirPods Pro 2 crackle badly on 8-block frames and are perfectly clean on 16 (A/B tested
/// on hardware: joint and plain stereo at 8 blocks both bad; 16 blocks good), so 16 is the default everywhere.
const DEFAULT_BLOCKS: f32 = 16.0;
/// Minimum SBC frames per packet for live audio (16-block frames are 2.67 ms, so 5.3 ms packets).
const DEFAULT_LIVE_FRAMES_PER_PACKET: usize = 2;

/// `airlow tone [blocks=16] [frames_per_packet=3] [flush_ms=40] [bitpool=53] [seconds=8]`
fn tone_opts(first: usize) -> a2dp::StreamOpts {
    let arg = |i: usize, d: f32| std::env::args().nth(first + i - 2).and_then(|s| s.parse().ok()).unwrap_or(d);
    a2dp::StreamOpts {
        cfg: sbc::Config {
            rate: sbc::Rate::Hz48000,
            mode: sbc::Mode::JointStereo,
            blocks: arg(2, DEFAULT_BLOCKS) as u8,
            subbands: 8,
            bitpool: arg(5, 53.0) as u8,
            snr: false,
        },
        frames_per_packet: arg(3, 3.0) as usize,
        flush_ms: arg(4, 40.0),
        seconds: arg(6, 8.0) as u32,
    }
}

fn tone(h: &mut Hci) -> Result<()> {
    let opts = tone_opts(2);
    let (addr, key) = load_key(&key_path())?;
    init(h)?;
    println!("Connecting to {}...", fmt_addr(&addr));
    let handle = a2dp::connect_with_retry(h, &addr, &key, init)?;
    println!("Encrypted ACL link up (handle {handle:#05x})");
    let mut l = a2dp::Link::new(h, handle)?;
    a2dp::stream_tone(&mut l, &opts)
}

#[cfg(windows)]
#[link(name = "winmm")]
unsafe extern "system" {
    fn timeBeginPeriod(period_ms: u32) -> u32;
}
#[cfg(windows)]
#[link(name = "kernel32")]
unsafe extern "system" {
    fn GetCurrentProcess() -> *mut core::ffi::c_void;
    fn SetPriorityClass(process: *mut core::ffi::c_void, class: u32) -> i32;
}

/// 1 ms timer ticks and high priority: the default 15.6 ms tick would make our pacing bursty.
fn realtime_tuning() {
    #[cfg(windows)]
    unsafe {
        timeBeginPeriod(1);
        SetPriorityClass(GetCurrentProcess(), 0x80); // HIGH_PRIORITY_CLASS
    }
}

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

#[cfg(test)]
mod default_tests {
    use super::*;

    #[test]
    fn default_sbc_layout_is_the_16_block_one_airpods_decode_cleanly() {
        assert_eq!(DEFAULT_BLOCKS, 16.0, "8-block SBC crackles on AirPods Pro 2 (hardware A/B test)");
        // tone_opts reads process args; the test runner passes none of the numeric ones we use.
        let o = tone_opts(usize::MAX / 2);
        assert_eq!(o.cfg.blocks, 16);
        assert_eq!(o.cfg.subbands, 8);
        assert_eq!(o.cfg.bitpool, 53);
        assert!(matches!(o.cfg.mode, sbc::Mode::JointStereo));
    }
}

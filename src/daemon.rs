//! Background session manager for the tray app: keeps the radio listening for the paired AirPods, accepts their
//! reconnect (or pages them), and streams Windows audio while they are connected.

use crate::a2dp::{self, Codec, Link};
use crate::hci::{Addr, Hci, fmt_addr};
use crate::live;
use crate::session::{DEFAULT_LIVE_FRAMES_PER_PACKET, init, key_path, load_key, pair, tone_opts, usb_id};
use anyhow::Result;
use std::path::{Path, PathBuf};
use std::sync::atomic::Ordering;
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::time::{Duration, Instant};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Status {
    NoController(String),
    NeedsPairing,
    Pairing,
    Waiting,
    Connecting,
    Streaming,
    Error(String),
}

impl Status {
    pub fn text(&self) -> String {
        match self {
            Status::NoController(e) => format!("Bluetooth controller not available ({e})"),
            Status::NeedsPairing => "Not paired: choose Pair AirPods".into(),
            Status::Pairing => "Pairing: put the AirPods in pairing mode".into(),
            Status::Waiting => "Waiting for AirPods (open the case or wear them)".into(),
            Status::Connecting => "Connecting...".into(),
            Status::Streaming => "Streaming".into(),
            Status::Error(e) => format!("Error: {e}"),
        }
    }
}

impl Status {
    /// Tray icon colour (RGB): grey = idle/needs action, amber = in progress, green = streaming, red = error.
    pub fn rgb(&self) -> [u8; 3] {
        match self {
            Status::NoController(_) | Status::NeedsPairing => [140, 140, 140],
            Status::Pairing | Status::Waiting | Status::Connecting => [240, 170, 30],
            Status::Streaming => [40, 190, 80],
            Status::Error(_) => [220, 60, 60],
        }
    }
}

/// A filled circle of `rgb` on a transparent `size` x `size` RGBA canvas (anti-aliased edge).
pub fn icon_rgba(size: u32, rgb: [u8; 3]) -> Vec<u8> {
    let c = (size as f32 - 1.0) / 2.0;
    let r = size as f32 / 2.0 - 1.0;
    let mut px = Vec::with_capacity((size * size * 4) as usize);
    for y in 0..size {
        for x in 0..size {
            let d = ((x as f32 - c).powi(2) + (y as f32 - c).powi(2)).sqrt();
            let a = (r + 0.5 - d).clamp(0.0, 1.0);
            px.extend_from_slice(&[rgb[0], rgb[1], rgb[2], (a * 255.0) as u8]);
        }
    }
    px
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cmd {
    Pair,
    Reconnect,
    Quit,
}

/// User settings from `%APPDATA%\airlow\config.txt` (`key = value` lines, `#` comments).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Config {
    /// Substring of the Windows output device whose loopback is streamed to the AirPods.
    pub capture_device: String,
    /// Initial AirPods volume, 0..=127 (Windows' own volume does not reach them).
    pub volume: u8,
    pub codec: Codec,
}

impl Default for Config {
    fn default() -> Self {
        Self { capture_device: "Steam Streaming Speakers".into(), volume: 0x30, codec: Codec::Sbc }
    }
}

const DEFAULT_CONFIG_FILE: &str = "# airlow settings\n\
# Windows output device whose audio is sent to the AirPods (part of its name):\n\
capture_device = Steam Streaming Speakers\n\
# Initial AirPods volume, 0-127 (Windows' volume slider does not reach them):\n\
volume = 48\n\
# sbc (default, fastest) or aac:\n\
codec = sbc\n";

impl Config {
    pub fn parse(text: &str) -> Config {
        let mut c = Config::default();
        for line in text.lines() {
            let line = line.split('#').next().unwrap_or("");
            let Some((k, v)) = line.split_once('=') else { continue };
            let v = v.trim();
            match k.trim() {
                "capture_device" if !v.is_empty() => c.capture_device = v.into(),
                "volume" => {
                    if let Ok(n) = v.parse::<u8>() {
                        c.volume = n.min(127);
                    }
                }
                "codec" => c.codec = if v.eq_ignore_ascii_case("aac") { Codec::Aac } else { Codec::Sbc },
                _ => {}
            }
        }
        c
    }

    pub fn path() -> PathBuf {
        key_path().with_file_name("config.txt")
    }

    /// Loads the config file; writes a commented default the first time so there is something to edit.
    pub fn load() -> Config {
        let p = Self::path();
        match std::fs::read_to_string(&p) {
            Ok(t) => Self::parse(&t),
            Err(_) => {
                let _ = std::fs::write(&p, DEFAULT_CONFIG_FILE);
                Config::default()
            }
        }
    }
}

pub struct Timing {
    /// How often to actively page the AirPods while waiting for them to connect on their own.
    pub page_every: Duration,
    /// Pause after an error before trying again.
    pub retry_after: Duration,
    /// Wait this long after streaming starts before opening the AirPods control channel (noise control).
    pub aacp_delay: Duration,
}

impl Default for Timing {
    fn default() -> Self {
        Self { page_every: Duration::from_secs(12), retry_after: Duration::from_secs(5), aacp_delay: Duration::from_secs(3) }
    }
}

enum Waited {
    Link(u16),
    Cmd(Cmd),
}

fn page(h: &mut Hci, addr: &Addr) -> Result<()> {
    let mut p = addr.to_vec();
    p.extend_from_slice(&0xCC18u16.to_le_bytes());
    p.extend_from_slice(&[1, 0, 0, 0, 0]); // R1 paging, clock offset unknown, never switch roles
    h.cmd(0x0405, &p).map(|_| ())
}

/// Listen with page scan on until the AirPods connect: accept their Connection_Request (we stay central) or
/// complete our own page. Returns the ACL handle, or the command that interrupted the wait.
fn wait_for_link(h: &mut Hci, addr: &Addr, rx: &Receiver<Cmd>, t: &Timing) -> Result<Waited> {
    h.cmd(0x0C1A, &[2])?; // Write_Scan_Enable: page scan only (we are not discoverable)
    let mut next_page = Instant::now() + t.page_every.min(Duration::from_secs(2));
    loop {
        match rx.try_recv() {
            Ok(Cmd::Reconnect) => next_page = Instant::now(),
            Ok(c) => return Ok(Waited::Cmd(c)),
            Err(std::sync::mpsc::TryRecvError::Disconnected) => return Ok(Waited::Cmd(Cmd::Quit)),
            Err(_) => {}
        }
        if Instant::now() >= next_page {
            page(h, addr)?;
            next_page = Instant::now() + t.page_every;
        }
        let Some(e) = h.next_event(Duration::from_millis(100)) else { continue };
        match e[0] {
            0x04 if e.len() >= 8 && e[2..8] == addr[..] => {
                let mut r = addr.to_vec();
                r.push(0); // become central
                h.cmd(0x0409, &r)?;
            }
            0x03 if e.len() >= 11 && e[2] == 0 && e[5..11] == addr[..] => return Ok(Waited::Link(u16::from_le_bytes([e[3], e[4]]))),
            _ => {}
        }
    }
}

fn do_pair(h: &mut Hci, keyfile: &Path, report: &dyn Fn(Status)) -> Result<()> {
    report(Status::Pairing);
    pair(h, keyfile)?;
    Ok(())
}

/// One cycle: wait for the AirPods, connect, stream until the link drops. `Ok(false)` means quit.
fn cycle(h: &mut Hci, keyfile: &Path, rx: &Receiver<Cmd>, report: &dyn Fn(Status), t: &Timing, cfg: &Config) -> Result<bool> {
    let (addr, key) = load_key(keyfile)?;
    report(Status::Waiting);
    init(h)?;
    let hd = match wait_for_link(h, &addr, rx, t)? {
        Waited::Cmd(Cmd::Quit) => return Ok(false),
        Waited::Cmd(Cmd::Pair) => {
            do_pair(h, keyfile, report)?;
            return Ok(true);
        }
        Waited::Cmd(Cmd::Reconnect) => return Ok(true),
        Waited::Link(hd) => hd,
    };
    report(Status::Connecting);
    println!("AirPods {} connected, authenticating", fmt_addr(&addr));
    let hd = match a2dp::finish_link(h, &addr, &key, hd) {
        Ok(hd) => hd,
        Err(e) => {
            hangup(h, hd);
            return Err(e);
        }
    };
    let r = stream_session(h, hd, report, t, cfg);
    crate::aacp::reset();
    println!("stream ended: {}", r.as_ref().map(|_| "ok".to_string()).unwrap_or_else(|e| e.to_string()));
    // Always say goodbye. Resetting the controller instead leaves the AirPods believing the old link is alive, and
    // they then refuse the next session's channels (hardware-verified: an endless connect/timeout/reconnect loop).
    hangup(h, hd);
    Ok(!live::STOP.load(Ordering::Relaxed))
}

fn stream_session(h: &mut Hci, hd: u16, report: &dyn Fn(Status), t: &Timing, cfg: &Config) -> Result<()> {
    let mut o = tone_opts(usize::MAX / 2);
    o.codec = cfg.codec;
    o.cfg.rate = live::loopback_rate()?;
    o.frames_per_packet = DEFAULT_LIVE_FRAMES_PER_PACKET;
    o.seconds = 30 * 86_400;
    let mut l = Link::new(h, hd)?;
    l.aacp_after(t.aacp_delay);
    report(Status::Streaming);
    live::stream_live(&mut l, &o)
}

/// HCI Disconnect (reason: remote user terminated) and wait for it to complete. Errors are ignored: the link may
/// already be gone.
fn hangup(h: &mut Hci, hd: u16) {
    let mut p = hd.to_le_bytes().to_vec();
    p.push(0x13);
    if h.cmd(0x0406, &p).is_err() {
        return;
    }
    let end = Instant::now() + Duration::from_millis(1500);
    while Instant::now() < end {
        if let Some(e) = h.next_event(Duration::from_millis(50)) {
            if e[0] == 0x05 {
                return;
            }
        }
    }
}

/// Run until `Cmd::Quit`. `h` is an open controller; `keyfile` holds the pairing.
pub fn serve(h: &mut Hci, keyfile: &Path, rx: &Receiver<Cmd>, report: &dyn Fn(Status), t: &Timing, cfg: &Config) -> Result<()> {
    *live::CAPTURE_NAME.lock().unwrap() = Some(cfg.capture_device.clone());
    a2dp::AVRCP_VOLUME.store(cfg.volume, Ordering::Relaxed);
    loop {
        if !keyfile.exists() {
            report(Status::NeedsPairing);
            match rx.recv_timeout(Duration::from_millis(500)) {
                Ok(Cmd::Quit) | Err(RecvTimeoutError::Disconnected) => return Ok(()),
                Ok(Cmd::Pair) => {
                    let r = init(h).and_then(|_| do_pair(h, keyfile, report));
                    if let Err(e) = r {
                        report(Status::Error(e.to_string()));
                        std::thread::sleep(t.retry_after);
                    }
                }
                _ => {}
            }
            continue;
        }
        match cycle(h, keyfile, rx, report, t, cfg) {
            Ok(true) => {}
            Ok(false) => return Ok(()),
            Err(e) => {
                println!("session error: {e}");
                report(Status::Error(e.to_string()));
                if let Ok(Cmd::Quit) | Err(RecvTimeoutError::Disconnected) = rx.recv_timeout(t.retry_after) {
                    return Ok(());
                }
            }
        }
    }
}

/// Open the controller (retrying while it is missing) and serve. The tray's worker thread runs this.
pub fn run(rx: Receiver<Cmd>, report: &dyn Fn(Status)) {
    let cfg = Config::load();
    let (vid, pid) = usb_id();
    loop {
        match Hci::open(vid, pid) {
            Ok(mut h) => {
                if let Err(e) = serve(&mut h, &key_path(), &rx, report, &Timing::default(), &cfg) {
                    println!("daemon error: {e}");
                    report(Status::Error(e.to_string()));
                }
                return;
            }
            Err(e) => {
                report(Status::NoController(e.to_string()));
                if let Ok(Cmd::Quit) | Err(RecvTimeoutError::Disconnected) = rx.recv_timeout(Duration::from_secs(5)) {
                    return;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_defaults_target_the_steam_virtual_speakers() {
        let c = Config::parse("");
        assert_eq!(c, Config::default());
        assert_eq!(c.capture_device, "Steam Streaming Speakers");
        assert_eq!(c.volume, 0x30);
        assert_eq!(c.codec, Codec::Sbc);
    }

    #[test]
    fn config_parses_values_comments_and_ignores_garbage() {
        let c = Config::parse("# hi\ncapture_device = My Cable # trailing\nvolume = 200\ncodec = AAC\nnonsense\nvolume_x = 3\n");
        assert_eq!(c.capture_device, "My Cable");
        assert_eq!(c.volume, 127, "volume is clamped to the AVRCP range");
        assert_eq!(c.codec, Codec::Aac);
        assert_eq!(Config::parse("volume = abc").volume, 0x30);
        assert_eq!(Config::parse("capture_device =").capture_device, "Steam Streaming Speakers");
    }

    #[test]
    fn the_default_config_file_parses_to_the_defaults() {
        assert_eq!(Config::parse(DEFAULT_CONFIG_FILE).capture_device, Config::default().capture_device);
        assert_eq!(Config::parse(DEFAULT_CONFIG_FILE).volume, 48);
    }

    #[test]
    fn icon_is_a_centered_disc_with_transparent_corners() {
        let px = icon_rgba(32, [1, 2, 3]);
        assert_eq!(px.len(), 32 * 32 * 4);
        let at = |x: usize, y: usize| &px[(y * 32 + x) * 4..(y * 32 + x) * 4 + 4];
        assert_eq!(at(16, 16), [1, 2, 3, 255]);
        assert_eq!(at(0, 0)[3], 0);
        assert_eq!(at(31, 31)[3], 0);
    }

    #[test]
    fn every_state_has_a_distinct_enough_colour_story() {
        assert_eq!(Status::Streaming.rgb(), [40, 190, 80]);
        assert_ne!(Status::Waiting.rgb(), Status::Streaming.rgb());
        assert_ne!(Status::Error("x".into()).rgb(), Status::NeedsPairing.rgb());
        assert!(Status::Error("boom".into()).text().contains("boom"));
    }
}

//! Simulated controller + AirPods-style A2DP sink, so the whole stack runs end to end without hardware.
//!
//! The sink models behaviour observed on a real AirPods Pro (replayed SDP queries, AVRCP exchange,
//! AVDTP state machine) and is deliberately strict: it flags any protocol error, pauses the stream
//! if media is late, drops stale/duplicate RTP, and decodes every SBC frame with libsbc so tests can
//! check the audio that would actually be heard.

use crate::hci::{Backend, Hci};
use crate::sbc;
use anyhow::Result;
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, Sender, channel};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

pub const SINK_ADDR: [u8; 6] = [0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB];
const HANDLE: u16 = 0x0032;
const R_SDP: u16 = 0x3700; // remote-side CIDs
const R_SIG: u16 = 0x3705;
const R_MEDIA: u16 = 0x3706;
const R_AVCTP: u16 = 0x3707;
const R_AACP: u16 = 0x3708;

#[derive(Clone)]
pub struct SimConfig {
    /// Controller ACL buffer size / count, as reported by Read_Buffer_Size.
    pub acl_mtu: usize,
    pub credits: usize,
    /// Radio throughput in bytes/s (0 = unlimited).
    pub air_bytes_per_sec: u64,
    /// The sink pauses (AVRCP PAUSE) if no media arrives this long after Start, or mid-stream.
    pub media_deadline: Duration,
    /// Inject stale events from a "previous process" before the first command.
    pub stale_events: bool,
    /// The sink connects to us by itself right after Reset (AirPods auto-reconnect).
    pub incoming_after_reset: bool,
    /// Link key the sink already holds (None = needs pairing).
    pub bonded_key: Option<[u8; 16]>,
    /// Probe us over SDP like the real AirPods do.
    pub sdp_probe: bool,
    /// Connection Complete status to fail with (0x04 = page timeout), 0 = succeed.
    pub connect_status: u8,
    /// The sink hangs up (reason 0x13) as soon as authentication starts.
    pub hangup_in_pairing: bool,
    /// The sink rejects Set_Configuration.
    pub reject_config: bool,
    /// A leftover Disconnection Complete from the previous process arrives well after Reset.
    pub late_stale_disconnect: bool,
    /// How long an inquiry takes before Inquiry Complete (real scans take seconds).
    pub inquiry_ms: u64,
    /// The first Create_Connection fails with 0x0B (the sink's own link already exists).
    pub exists_once: bool,
    /// Inject malformed events and ACL fragments (truncated, orphaned, over-long) during the session.
    pub garbage: bool,
    /// Controller completes at most this many ACL packets per second regardless of size (0 = unlimited).
    /// The real MediaTek controller managed ~300-360.
    pub max_pkts_per_sec: u64,
    /// Sink playout buffer prefill; the sink underruns if audio arrives slower than real time.
    pub sink_prefill_ms: u64,
    /// Reject AVDTP Reconfigure (optional in the spec). The real AirPods do, with error 0x81.
    pub reject_reconfigure: bool,
    /// Offer only the SBC endpoint (a sink without AAC).
    pub sbc_only: bool,
    /// Once page scan is enabled the sink "opens the case" and pages us: a Connection_Request event arrives
    /// and the link only comes up if we answer it with Accept_Connection_Request.
    pub request_on_scan: bool,
    /// The sink hangs up this many ms after AVDTP Start (goes back in the case).
    pub drop_after_start_ms: Option<u64>,
    /// Offer the AirPods control channel (PSM 0x1001).
    pub aacp: bool,
    /// Noise control mode the sink starts in (1 off, 2 ANC, 3 transparency, 4 adaptive).
    pub aacp_mode: u8,
}

impl Default for SimConfig {
    fn default() -> Self {
        Self {
            acl_mtu: 1021,
            credits: 8,
            air_bytes_per_sec: 0,
            media_deadline: Duration::from_millis(500),
            stale_events: false,
            incoming_after_reset: false,
            bonded_key: None,
            sdp_probe: true,
            connect_status: 0,
            hangup_in_pairing: false,
            reject_config: false,
            late_stale_disconnect: false,
            inquiry_ms: 10,
            exists_once: false,
            garbage: false,
            max_pkts_per_sec: 0,
            sink_prefill_ms: 120,
            reject_reconfigure: true,
            sbc_only: false,
            request_on_scan: false,
            drop_after_start_ms: None,
            aacp: true,
            aacp_mode: 2,
        }
    }
}

#[derive(Default, Clone)]
pub struct Report {
    pub protocol_errors: Vec<String>,
    pub sdp_requests: usize,
    pub sdp_ok: usize,
    pub avrcp_caps_ok: bool,
    pub avrcp_notif_ok: usize,
    pub play_status_ok: bool,
    pub volume: Option<u8>,
    pub volume_at: Option<Instant>,
    pub start_at: Option<Instant>,
    pub first_media_at: Option<Instant>,
    pub last_media_at: Option<Instant>,
    pub paused_by_sink: bool,
    pub pause_acked: bool,
    pub suspended: bool,
    pub configured: Option<[u8; 4]>,
    pub media_pkts: u64,
    pub media_dropped_after_pause: u64,
    pub seq_gaps: u64,
    pub seq_backwards: u64,
    pub ts_errors: u64,
    pub arrivals: Vec<(u16, u32, Instant, u8)>, // seq, ts, time, frames
    pub pcm: Vec<i16>,                          // decoded left channel
    pub flush_option_seen: bool,
    pub max_outstanding: usize,
    pub flushed: u64,
    pub link_key_issued: Option<[u8; 16]>,
    pub encrypted: bool,
    /// Accept_Connection_Request was correctly answered for an incoming link.
    pub accepted_request: bool,
    pub aacp_handshake: bool,
    pub aacp_features: bool,
    pub aacp_notify: bool,
    /// Noise control modes the host commanded, in order.
    pub aacp_sets: Vec<u8>,
    /// The host sent the Allow Off option setting.
    pub aacp_allow_off: bool,
    /// The host disconnected a live link (HCI Disconnect) instead of just resetting the controller.
    pub hangups: u32,
    pub pb_violations: u64,
    pub l2cap_config_rsp_scid_ok: bool,
    /// Times the sink's playout buffer ran dry (audible as dropouts or silence).
    pub underruns: u64,
    pub max_buffer_ms: f64,
    pub reconfigs: u64,
    pub frames_decoded: u64,
    /// Milliseconds after Start at which each underrun happened, and the largest gap between media packets.
    pub underrun_at_ms: Vec<u64>,
    pub max_arrival_gap_ms: f64,
    /// AAC: the configured codec information element and how many AAC frames were decoded.
    pub configured_aac: Option<[u8; 6]>,
    pub aac_frames: u64,
    /// Why/when the sink paused the stream (diagnostics for failing tests).
    pub pause_info: Option<String>,
}

enum Msg {
    Cmd(Vec<u8>),
    Acl(Vec<u8>),
    Stop,
}

struct SimBackend {
    tx: Mutex<Sender<Msg>>,
}

impl Backend for SimBackend {
    fn send_cmd(&self, pkt: &[u8]) -> Result<()> {
        self.tx.lock().unwrap().send(Msg::Cmd(pkt.to_vec()))?;
        Ok(())
    }
    fn send_acl(&self, pkt: &[u8]) -> Result<()> {
        self.tx.lock().unwrap().send(Msg::Acl(pkt.to_vec()))?;
        Ok(())
    }
}

pub struct SimHandle {
    air: Arc<AtomicU64>,
    report: Arc<Mutex<Report>>,
    tx: Sender<Msg>,
    join: Option<JoinHandle<()>>,
}

impl SimHandle {
    pub fn report(&self) -> Report {
        self.report.lock().unwrap().clone()
    }

    /// Change radio throughput while running (0 = unlimited): lets tests kill or throttle the air link.
    pub fn set_air(&self, bytes_per_sec: u64) {
        self.air.store(bytes_per_sec, Ordering::Relaxed);
    }
}

impl Drop for SimHandle {
    fn drop(&mut self) {
        let _ = self.tx.send(Msg::Stop);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

pub fn spawn(cfg: SimConfig) -> (Hci, SimHandle) {
    let (tx, rx) = channel();
    let (etx, erx) = channel();
    let (atx, arx) = channel();
    let report = Arc::new(Mutex::new(Report::default()));
    let hci = Hci::from_parts(Box::new(SimBackend { tx: Mutex::new(tx.clone()) }), erx, arx);
    let air = Arc::new(AtomicU64::new(cfg.air_bytes_per_sec));
    let mut sim = Sim::new(cfg, report.clone(), etx, atx, rx);
    sim.air = air.clone();
    let join = std::thread::spawn(move || sim.run());
    (hci, SimHandle { air, report, tx, join: Some(join) })
}

// ---------------------------------------------------------------------------------------------

enum Out {
    Evt(Vec<u8>),
    AclToHost(Vec<u8>), // L2CAP PDU (already framed with len+cid), fragmented on send
    Complete { flushed: bool },
    DeliverToRemote(Vec<u8>), // ACL packet
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum AvdtpState {
    Idle,
    Configured,
    Open,
    Streaming,
}

struct Chan {
    local: u16, // remote-side cid
    peer: u16,  // host-side cid
    psm: u16,
    cfg_in: bool,  // host's config accepted by us
    cfg_out: bool, // our config accepted by host
}

struct Sim {
    cfg: SimConfig,
    rep: Arc<Mutex<Report>>,
    etx: Sender<Vec<u8>>,
    atx: Sender<Vec<u8>>,
    rx: Receiver<Msg>,
    timed: VecDeque<(Instant, Out)>,
    connected: bool,
    bonded: Option<[u8; 16]>,
    pending_key: [u8; 16],
    flush_slots: u16,
    outstanding: usize,
    air_free_at: Instant,
    reasm: Vec<u8>,
    reasm_need: usize,
    chans: HashMap<u16, Chan>, // by remote-side cid
    n_avdtp: u8,
    aacp_mode: u8,
    aacp_off_allowed: bool,
    sig_id: u8,
    avdtp: AvdtpState,
    // media
    dec: sbc::Decoder,
    last_seq: Option<u16>,
    last_ts: Option<u32>,
    last_samples: u32,
    // sdp client
    sdp_next: usize,
    sdp_expect: Vec<bool>,
    avctp_txn: u8,
    air: Arc<AtomicU64>,
    delay_reporting: bool,
    play_start: Option<Instant>,
    rx_samples: u64,
    /// The endpoint the host configured (None until Set_Configuration is accepted) and its codec.
    active_seid: Option<u8>,
    codec: SimCodec,
    aac_dec: Option<crate::aac::AacDecoder>,
    aac_rate: u32,
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum SimCodec {
    Sbc,
    Aac,
}

fn le16(b: &[u8]) -> u16 {
    u16::from_le_bytes([b[0], b[1]])
}

impl Sim {
    fn new(cfg: SimConfig, rep: Arc<Mutex<Report>>, etx: Sender<Vec<u8>>, atx: Sender<Vec<u8>>, rx: Receiver<Msg>) -> Self {
        let bonded = cfg.bonded_key;
        let aacp_mode = cfg.aacp_mode;
        Self {
            cfg,
            rep,
            etx,
            atx,
            rx,
            timed: VecDeque::new(),
            connected: false,
            bonded,
            pending_key: [0xA5; 16],
            flush_slots: 0,
            outstanding: 0,
            air_free_at: Instant::now(),
            reasm: vec![],
            reasm_need: 0,
            chans: HashMap::new(),
            n_avdtp: 0,
            aacp_mode,
            aacp_off_allowed: false,
            sig_id: 0x20,
            avdtp: AvdtpState::Idle,
            dec: sbc::Decoder::new(),
            last_seq: None,
            last_ts: None,
            last_samples: 0,
            sdp_next: 0,
            sdp_expect: vec![],
            avctp_txn: 0,
            air: Arc::new(AtomicU64::new(0)),
            delay_reporting: false,
            play_start: None,
            rx_samples: 0,
            active_seid: None,
            codec: SimCodec::Sbc,
            aac_dec: None,
            aac_rate: 48000,
        }
    }

    fn err(&self, s: impl Into<String>) {
        let s = s.into();
        if crate::hci::trace() {
            eprintln!("[sim] PROTOCOL ERROR: {s}");
        }
        self.rep.lock().unwrap().protocol_errors.push(s);
    }

    fn at(&mut self, delay_ms: u64, o: Out) {
        let due = Instant::now() + Duration::from_millis(delay_ms);
        let pos = self.timed.iter().position(|(d, _)| *d > due).unwrap_or(self.timed.len());
        self.timed.insert(pos, (due, o));
    }

    fn evt(&self, code: u8, p: &[u8]) {
        let mut v = vec![code, p.len() as u8];
        v.extend_from_slice(p);
        let _ = self.etx.send(v);
    }

    fn cc(&self, op: u16, ret: &[u8]) {
        let mut p = vec![1, op.to_le_bytes()[0], op.to_le_bytes()[1]];
        p.extend_from_slice(ret);
        self.evt(0x0E, &p);
    }

    fn cs(&self, op: u16, status: u8) {
        self.evt(0x0F, &[status, 1, op.to_le_bytes()[0], op.to_le_bytes()[1]]);
    }

    fn run(&mut self) {
        if self.cfg.stale_events {
            // Leftovers of a previous process: a disconnect and a connect for the same handle.
            self.evt(0x03, &[0, 0x32, 0, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 1, 0]);
            self.evt(0x05, &[0, 0x32, 0, 0x13]);
        }
        loop {
            let now = Instant::now();
            let wait = self
                .timed
                .front()
                .map(|(d, _)| d.saturating_duration_since(now))
                .unwrap_or(Duration::from_millis(5))
                .min(Duration::from_millis(5));
            match self.rx.recv_timeout(wait) {
                Ok(Msg::Cmd(p)) => self.handle_cmd(&p),
                Ok(Msg::Acl(p)) => self.handle_acl(&p),
                Ok(Msg::Stop) | Err(RecvTimeoutError::Disconnected) => return,
                Err(RecvTimeoutError::Timeout) => {}
            }
            let now = Instant::now();
            while self.timed.front().map(|(d, _)| *d <= now).unwrap_or(false) {
                let (_, o) = self.timed.pop_front().unwrap();
                self.fire(o);
            }
            self.check_deadlines();
        }
    }

    fn fire(&mut self, o: Out) {
        match o {
            Out::Evt(e) => {
                let _ = self.etx.send(e);
            }
            Out::AclToHost(pdu) => self.send_to_host(&pdu),
            Out::Complete { flushed } => {
                self.outstanding = self.outstanding.saturating_sub(1);
                if flushed {
                    self.evt(0x11, &[0x32, 0x00]);
                    self.rep.lock().unwrap().flushed += 1;
                }
                self.evt(0x13, &[1, 0x32, 0x00, 1, 0]);
            }
            Out::DeliverToRemote(p) => self.remote_acl(&p),
        }
    }

    // ---------------- HCI commands ----------------

    fn handle_cmd(&mut self, pkt: &[u8]) {
        if pkt.len() < 3 || pkt.len() != 3 + pkt[2] as usize {
            self.err(format!("malformed HCI command {pkt:02x?}"));
            return;
        }
        let op = le16(pkt);
        let p = &pkt[3..];
        let h = HANDLE.to_le_bytes();
        match op {
            0x0C03 => {
                if self.cfg.late_stale_disconnect {
                    self.at(300, Out::Evt(vec![0x05, 4, 0, 0x32, 0, 0x13]));
                }
                self.connected = false;
                self.chans.clear();
                self.avdtp = AvdtpState::Idle;
                self.cc(op, &[0]);
                if self.cfg.incoming_after_reset {
                    self.at(30, Out::Evt(vec![0x03, 11, 0, 0x32, 0, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 1, 0]));
                    self.connected = true;
                }
            }
            0x0C1A => {
                self.cc(op, &[0]);
                if self.cfg.request_on_scan && p.first().map(|v| v & 2 != 0).unwrap_or(false) && !self.connected {
                    self.cfg.request_on_scan = false;
                    self.at(40, Out::Evt([vec![0x04, 10], SINK_ADDR.to_vec(), vec![0x14, 0x04, 0x20, 1]].concat()));
                }
            }
            0x0406 => {
                if p.len() == 3 && le16(p) == HANDLE && self.connected {
                    self.cs(op, 0);
                    self.connected = false;
                    self.chans.clear();
                    self.avdtp = AvdtpState::Idle;
                    self.rep.lock().unwrap().hangups += 1;
                    self.at(5, Out::Evt(vec![0x05, 4, 0, 0x32, 0, p[2]]));
                } else {
                    self.cs(op, 0x02); // no such connection
                }
            }
            0x0409 => {
                if p.len() != 7 || p[..6] != SINK_ADDR || p[6] != 0 {
                    self.err(format!("Accept_Connection_Request must name the requester and take the central role, got {p:02x?}"));
                    self.cs(op, 0x02);
                } else {
                    self.cs(op, 0);
                    self.connected = true;
                    self.rep.lock().unwrap().accepted_request = true;
                    self.at(10, Out::Evt(vec![0x03, 11, 0, 0x32, 0, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 1, 0]));
                }
            }
            0x0C01 | 0x0C56 | 0x0C45 | 0x0C24 | 0x080F => self.cc(op, &[0]),
            0x1001 => self.cc(op, &[0, 13, 0x04, 0x11, 13, 70, 0, 0x06, 0x26]),
            0x1009 => {
                let mut r = vec![0];
                r.extend_from_slice(&[0x66, 0x55, 0x44, 0x33, 0x22, 0x11]);
                self.cc(op, &r);
            }
            0x1005 => {
                let (m, c) = (self.cfg.acl_mtu as u16, self.cfg.credits as u16);
                self.cc(op, &[0, m.to_le_bytes()[0], m.to_le_bytes()[1], 0, c.to_le_bytes()[0], c.to_le_bytes()[1], 0, 0]);
            }
            0x080D => {
                if p.len() == 4 && le16(p) == HANDLE && le16(&p[2..]) == 0 {
                    self.cc(op, &[0, h[0], h[1]]);
                } else {
                    self.err(format!("Write_Link_Policy_Settings must forbid sniff/role switch, got {p:02x?}"));
                    self.cc(op, &[0x12]);
                }
            }
            0x0C28 => {
                if p.len() == 4 && le16(p) == HANDLE {
                    self.flush_slots = le16(&p[2..]);
                    self.cc(op, &[0, h[0], h[1]]);
                } else {
                    self.err("bad Write_Automatic_Flush_Timeout");
                    self.cc(op, &[0x02]);
                }
            }
            0x1405 => self.cc(op, &[0, h[0], h[1], 0xD7]),
            0x1403 => self.cc(op, &[0, h[0], h[1], 127]),
            0x1401 => self.cc(op, &[0, h[0], h[1], 0, 0]),
            0x0C2D => self.cc(op, &[0, h[0], h[1], 4]),
            0x1406 => self.cc(op, &[[0u8, h[0], h[1], 1].as_slice(), [0xFFu8; 9].as_slice(), [0x7Fu8].as_slice()].concat()),
            0x0401 => {
                self.cs(op, 0);
                let mut e = vec![1];
                e.extend_from_slice(&SINK_ADDR);
                e.extend_from_slice(&[1, 0, 0x18, 0x04, 0x24, 0, 0, 0xD0]);
                let mut eir = vec![0x16, 0x09];
                eir.extend_from_slice(b"AirPods Pro - Find My");
                eir.resize(240, 0);
                e.extend_from_slice(&eir);
                self.at(5, Out::Evt([vec![0x2F, e.len() as u8], e].concat()));
                let ms = self.cfg.inquiry_ms;
                self.at(ms, Out::Evt(vec![0x01, 1, 0]));
            }
            0x0405 => {
                if self.cfg.exists_once {
                    self.cfg.exists_once = false;
                    self.cs(op, 0);
                    self.at(5, Out::Evt(vec![0x03, 11, 0x0B, 0, 0, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 1, 0]));
                    return;
                }
                if self.cfg.connect_status != 0 {
                    self.cs(op, 0);
                    let st = self.cfg.connect_status;
                    self.at(20, Out::Evt(vec![0x03, 11, st, 0, 0, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 1, 0]));
                    return;
                }
                if p.len() < 13 || p[..6] != SINK_ADDR {
                    self.cs(op, 0);
                    self.at(5, Out::Evt(vec![0x03, 11, 0x04, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0]));
                } else if self.connected {
                    self.cs(op, 0);
                    self.at(5, Out::Evt(vec![0x03, 11, 0x0B, 0, 0, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 1, 0]));
                } else {
                    self.cs(op, 0);
                    self.connected = true;
                    if self.cfg.late_stale_disconnect {
                        // The previous process's link finally reports its disconnect, mid-connect.
                        self.at(2, Out::Evt(vec![0x05, 4, 0, 0x32, 0, 0x13]));
                    }
                    self.at(8, Out::Evt(vec![0x03, 11, 0, 0x32, 0, 0x56, 0x34, 0x12, 0xEF, 0xCD, 0xAB, 1, 0]));
                }
            }
            0x0411 => {
                if self.cfg.garbage {
                    for e in [
                        vec![0x05u8],
                        vec![0x05, 4, 0, 0x32, 0],
                        vec![0x03, 11, 0, 0x32],
                        vec![0x0E, 40, 1],
                        vec![0x18, 23, 1, 2, 3],
                        vec![0x13, 9, 1],
                        vec![],
                    ] {
                        let _ = self.etx.send(e);
                    }
                    let _ = self.atx.send(vec![0x32, 0x20]); // too short for an ACL header
                    let _ = self.atx.send(vec![0x32, 0x10, 3, 0, 1, 2, 3]); // orphan continuation fragment
                    let _ = self.atx.send(vec![0x32, 0x20, 1, 0, 9]); // start fragment too short for a length field
                }
                if self.cfg.hangup_in_pairing && self.connected {
                    self.cs(op, 0);
                    self.connected = false;
                    self.at(10, Out::Evt(vec![0x05, 4, 0, 0x32, 0, 0x13]));
                    return;
                }
                if p.len() == 2 && le16(p) == HANDLE && self.connected {
                    self.cs(op, 0);
                    self.at(3, Out::Evt([vec![0x17, 6], SINK_ADDR.to_vec()].concat()));
                } else {
                    self.err("Authentication_Requested for unknown handle");
                    self.cs(op, 0x02);
                }
            }
            0x040B => {
                self.cc(op, &[[0u8].as_slice(), SINK_ADDR.as_slice()].concat());
                let ok = p.len() == 22 && p[..6] == SINK_ADDR && self.bonded.map(|k| k[..] == p[6..22]).unwrap_or(false);
                if ok {
                    self.at(3, Out::Evt(vec![0x06, 3, 0, 0x32, 0]));
                } else {
                    self.at(3, Out::Evt(vec![0x06, 3, 0x06, 0x32, 0])); // PIN or key missing
                }
            }
            0x040C => {
                self.cc(op, &[[0u8].as_slice(), SINK_ADDR.as_slice()].concat());
                self.at(3, Out::Evt([vec![0x31, 6], SINK_ADDR.to_vec()].concat()));
            }
            0x042B => {
                if p.len() != 9 || p[6] != 0x03 {
                    self.err(format!("unexpected IO capability reply {p:02x?}"));
                }
                self.cc(op, &[[0u8].as_slice(), SINK_ADDR.as_slice()].concat());
                self.at(3, Out::Evt([vec![0x32, 9], SINK_ADDR.to_vec(), vec![3, 0, 4]].concat()));
                self.at(4, Out::Evt([vec![0x33, 10], SINK_ADDR.to_vec(), vec![1, 2, 3, 4]].concat()));
            }
            0x042C => {
                self.cc(op, &[[0u8].as_slice(), SINK_ADDR.as_slice()].concat());
                self.at(3, Out::Evt([vec![0x36, 7, 0], SINK_ADDR.to_vec()].concat()));
                let key = self.pending_key;
                self.bonded = Some(key);
                self.rep.lock().unwrap().link_key_issued = Some(key);
                self.at(4, Out::Evt([vec![0x18, 23], SINK_ADDR.to_vec(), key.to_vec(), vec![4]].concat()));
                self.at(5, Out::Evt(vec![0x06, 3, 0, 0x32, 0]));
            }
            0x0413 => {
                self.cs(op, 0);
                self.rep.lock().unwrap().encrypted = p.len() == 3 && p[2] == 1;
                self.at(3, Out::Evt(vec![0x08, 4, 0, 0x32, 0, 1]));
            }
            _ => {
                self.err(format!("unsupported HCI opcode {op:#06x}"));
                self.cc(op, &[0x01]);
            }
        }
    }

    // ---------------- ACL: controller model ----------------

    fn handle_acl(&mut self, pkt: &[u8]) {
        if pkt.len() < 4 || pkt.len() != 4 + le16(&pkt[2..]) as usize {
            self.err(format!("malformed ACL packet (len {})", pkt.len()));
            return;
        }
        let hf = le16(pkt);
        let (handle, pb) = (hf & 0x0FFF, (hf >> 12) & 3);
        if handle != HANDLE || !self.connected {
            self.err(format!("ACL for unknown handle {handle:#05x}"));
            return;
        }
        if pkt.len() - 4 > self.cfg.acl_mtu {
            self.err(format!("ACL payload {} exceeds controller buffer {}", pkt.len() - 4, self.cfg.acl_mtu));
        }
        if self.outstanding >= self.cfg.credits {
            self.err(format!("host exceeded controller credits ({} outstanding)", self.outstanding));
        }
        self.outstanding += 1;
        {
            let mut r = self.rep.lock().unwrap();
            r.max_outstanding = r.max_outstanding.max(self.outstanding);
        }
        // Air time model.
        let now = Instant::now();
        let start = self.air_free_at.max(now);
        let rate = self.air.load(Ordering::Relaxed);
        let mut air = if rate == 0 { Duration::ZERO } else { Duration::from_secs_f64(pkt.len() as f64 / rate as f64) };
        if self.cfg.max_pkts_per_sec > 0 {
            air = air.max(Duration::from_secs_f64(1.0 / self.cfg.max_pkts_per_sec as f64));
        }
        let done = start + air;
        let flushable = pb == 2 && self.flush_slots > 0;
        let limit = now + Duration::from_secs_f64(self.flush_slots as f64 * 0.000625);
        if flushable && done > limit {
            // Auto-flush: stale packet dropped, buffer returned, nothing delivered.
            self.air_free_at = limit.max(self.air_free_at);
            self.timed_push(limit, Out::Complete { flushed: true });
        } else {
            self.air_free_at = done;
            self.timed_push(done, Out::DeliverToRemote(pkt.to_vec()));
            self.timed_push(done, Out::Complete { flushed: false });
        }
    }

    fn timed_push(&mut self, due: Instant, o: Out) {
        let pos = self.timed.iter().position(|(d, _)| *d > due).unwrap_or(self.timed.len());
        self.timed.insert(pos, (due, o));
    }

    /// Frame a PDU for the host: ACL with auto-flushable start + continuation fragments.
    fn send_to_host(&mut self, pdu: &[u8]) {
        let mut first = true;
        for c in pdu.chunks(self.cfg.acl_mtu) {
            let pb: u16 = if first { 2 } else { 1 };
            let mut v = (HANDLE | (pb << 12)).to_le_bytes().to_vec();
            v.extend_from_slice(&(c.len() as u16).to_le_bytes());
            v.extend_from_slice(c);
            let _ = self.atx.send(v);
            first = false;
        }
    }

    /// Send on a channel identified by the sink's own (local) CID; the PDU is addressed to the HOST's CID.
    fn remote_send(&mut self, cid: u16, payload: &[u8]) {
        let cid = if cid == 1 { 1 } else { self.chans.get(&cid).map(|c| c.peer).unwrap_or(cid) };
        let mut pdu = (payload.len() as u16).to_le_bytes().to_vec();
        pdu.extend_from_slice(&cid.to_le_bytes());
        pdu.extend_from_slice(payload);
        self.send_to_host(&pdu);
    }

    // ---------------- remote side: L2CAP ----------------

    fn remote_acl(&mut self, pkt: &[u8]) {
        let pb = (le16(pkt) >> 12) & 3;
        let data = &pkt[4..];
        if pb == 1 {
            if self.reasm.is_empty() {
                self.err("continuation fragment without a start");
                self.rep.lock().unwrap().pb_violations += 1;
                return;
            }
            self.reasm.extend_from_slice(data);
        } else {
            if pb == 3 {
                self.rep.lock().unwrap().pb_violations += 1;
            }
            if !self.reasm.is_empty() {
                self.err("new start fragment before previous PDU completed");
            }
            self.reasm = data.to_vec();
            self.reasm_need = if data.len() >= 2 { le16(data) as usize + 4 } else { 0 };
        }
        if self.reasm.len() >= 4 && self.reasm.len() >= self.reasm_need {
            if self.reasm.len() > self.reasm_need {
                self.err("L2CAP PDU longer than its length field");
            }
            let pdu = std::mem::take(&mut self.reasm);
            let (cid, payload) = (le16(&pdu[2..]), pdu[4..].to_vec());
            self.remote_pdu(cid, &payload);
        }
    }

    fn remote_pdu(&mut self, cid: u16, p: &[u8]) {
        if cid == 1 {
            let mut o = 0;
            while o + 4 <= p.len() {
                let len = le16(&p[o + 2..]) as usize;
                if o + 4 + len > p.len() {
                    self.err("L2CAP signalling command overruns PDU");
                    break;
                }
                let cmd = p[o..o + 4 + len].to_vec();
                self.remote_sig(&cmd);
                o += 4 + len;
            }
            return;
        }
        let psm = self.chans.get(&cid).map(|c| c.psm);
        match psm {
            Some(0x19) if cid == R_SIG => self.avdtp_rx(p),
            Some(0x19) if cid == R_MEDIA => self.media_rx(p),
            Some(0x17) => self.avctp_rx(p),
            Some(0x1001) => self.aacp_rx(p),
            Some(0x01) => self.sdp_rx(p),
            _ => self.err(format!("data on unknown channel {cid:#06x}")),
        }
    }

    /// AirPods control channel: nothing works before the handshake; then features, notification subscription and
    /// noise control commands (answered with a notification, as the real AirPods do).
    fn aacp_rx(&mut self, p: &[u8]) {
        let seen_hs = self.rep.lock().unwrap().aacp_handshake;
        if p == crate::aacp::HANDSHAKE {
            self.rep.lock().unwrap().aacp_handshake = true;
        } else if !seen_hs {
            self.err(format!("AACP packet before the handshake: {p:02x?}"));
        } else if p == crate::aacp::FEATURES {
            self.rep.lock().unwrap().aacp_features = true;
        } else if p[..p.len().min(6)] == crate::aacp::NOTIFY[..6] && p.len() == crate::aacp::NOTIFY.len() {
            self.rep.lock().unwrap().aacp_notify = true;
            let n = crate::aacp::set_noise(crate::aacp::NoiseMode::from_byte(self.aacp_mode).unwrap());
            self.remote_send(R_AACP, &n);
            // Battery and ear detection reports as sent by the real AirPods Pro 2.
            self.remote_send(
                R_AACP,
                &[
                    0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x03, 0x04, 0x01, 0x5F, 0x02, 0x01, 0x02, 0x01, 0x64, 0x01, 0x01, 0x08, 0x01, 0x32,
                    0x02, 0x01,
                ],
            );
            self.remote_send(R_AACP, &[0x04, 0x00, 0x04, 0x00, 0x06, 0x00, 0x00, 0x01]);
        } else if p == crate::aacp::ALLOW_OFF {
            self.aacp_off_allowed = true;
            self.rep.lock().unwrap().aacp_allow_off = true;
        } else if let Some(m) = crate::aacp::parse_noise(p) {
            if !self.rep.lock().unwrap().aacp_notify {
                self.err("noise control command before subscribing to notifications");
            }
            self.rep.lock().unwrap().aacp_sets.push(m as u8);
            // Real AirPods silently keep the old mode when asked for Off while Off is not allowed.
            if m != crate::aacp::NoiseMode::Off || self.aacp_off_allowed {
                self.aacp_mode = m as u8;
            }
            let now = crate::aacp::NoiseMode::from_byte(self.aacp_mode).unwrap();
            self.remote_send(R_AACP, &crate::aacp::set_noise(now));
        } else {
            self.err(format!("unexpected AACP packet {p:02x?}"));
        }
    }

    fn next_sig_id(&mut self) -> u8 {
        self.sig_id = self.sig_id.wrapping_add(1).max(1);
        self.sig_id
    }

    fn send_config_req(&mut self, local: u16) {
        let peer = self.chans[&local].peer;
        let id = self.next_sig_id();
        let mut r = vec![0x04, id, 8, 0];
        r.extend_from_slice(&peer.to_le_bytes());
        r.extend_from_slice(&[0, 0, 1, 2, 0xFF, 0x03]); // MTU 1023
        self.remote_send(1, &r);
    }

    fn remote_sig(&mut self, c: &[u8]) {
        let (code, id, body) = (c[0], c[1], &c[4..]);
        match code {
            0x02 if body.len() == 4 => {
                let (psm, scid) = (le16(body), le16(&body[2..]));
                let local = match psm {
                    0x19 => {
                        self.n_avdtp += 1;
                        match self.n_avdtp {
                            1 => R_SIG,
                            2 => R_MEDIA,
                            _ => 0,
                        }
                    }
                    0x17 => R_AVCTP,
                    0x1001 if self.cfg.aacp => R_AACP,
                    _ => 0,
                };
                if local == 0 {
                    let mut r = vec![0x03, id, 8, 0, 0, 0];
                    r.extend_from_slice(&scid.to_le_bytes());
                    r.extend_from_slice(&[2, 0, 0, 0]);
                    self.remote_send(1, &r);
                    if psm != 0x1001 {
                        self.err(format!("host opened unsupported PSM {psm:#06x}"));
                    }
                    return;
                }
                self.chans.insert(local, Chan { local, peer: scid, psm, cfg_in: false, cfg_out: false });
                let mut r = vec![0x03, id, 8, 0];
                r.extend_from_slice(&local.to_le_bytes());
                r.extend_from_slice(&scid.to_le_bytes());
                r.extend_from_slice(&[0, 0, 0, 0]);
                self.remote_send(1, &r);
                self.send_config_req(local);
                if psm == 0x19 && local == R_SIG && self.cfg.sdp_probe {
                    // Like the real AirPods: an info request, then they open an SDP channel to us.
                    let i = self.next_sig_id();
                    self.remote_send(1, &[0x0A, i, 2, 0, 2, 0]);
                    let i = self.next_sig_id();
                    let mut r = vec![0x02, i, 4, 0, 1, 0];
                    r.extend_from_slice(&R_SDP.to_le_bytes());
                    self.chans.insert(R_SDP, Chan { local: R_SDP, peer: 0, psm: 1, cfg_in: false, cfg_out: false });
                    self.remote_send(1, &r);
                }
            }
            0x03 if body.len() == 8 => {
                // Host's response to our SDP connect request.
                let (dcid, scid, result) = (le16(body), le16(&body[2..]), le16(&body[4..]));
                if scid == R_SDP && result == 0 {
                    if let Some(c) = self.chans.get_mut(&R_SDP) {
                        c.peer = dcid;
                    }
                    self.send_config_req(R_SDP);
                } else if result != 0 && result != 1 {
                    self.err(format!("host refused our SDP channel (result {result})"));
                }
            }
            0x04 if body.len() >= 4 => {
                let dcid = le16(body);
                let Some(ch) = self.chans.get_mut(&dcid) else {
                    self.err(format!("config request for unknown cid {dcid:#06x}"));
                    return;
                };
                ch.cfg_in = true;
                let peer = ch.peer;
                let mut o = 4;
                while o + 2 <= body.len() {
                    let (t, l) = (body[o], body[o + 1] as usize);
                    if t == 2 {
                        self.rep.lock().unwrap().flush_option_seen = true;
                    }
                    o += 2 + l;
                }
                let mut r = vec![0x05, id, 6, 0];
                r.extend_from_slice(&peer.to_le_bytes()); // originator's cid, as AirPods expect
                r.extend_from_slice(&[0, 0, 0, 0]);
                self.remote_send(1, &r);
                self.maybe_chan_ready(dcid);
            }
            0x05 if body.len() >= 6 => {
                let (scid, result) = (le16(body), le16(&body[4..]));
                // Our request carried dcid=host cid, so the host's response names OUR cid in "scid".
                let found = self.chans.values().find(|c| c.local == scid).map(|c| c.local);
                let mut ok = found.is_some();
                if let Some(l) = found {
                    if result == 0 {
                        self.chans.get_mut(&l).unwrap().cfg_out = true;
                        self.maybe_chan_ready(l);
                    } else {
                        self.err(format!("host rejected our config (result {result})"));
                    }
                } else {
                    self.err(format!("config response names unknown cid {scid:#06x} (host must echo OUR cid)"));
                    ok = false;
                }
                let mut r = self.rep.lock().unwrap();
                r.l2cap_config_rsp_scid_ok = ok;
            }
            0x06 if body.len() == 4 => {
                let mut r = vec![0x07, id, 4, 0];
                r.extend_from_slice(body);
                self.remote_send(1, &r);
                // Free the channel so the host can open it again (media channel is re-opened on restart).
                let dcid = le16(body);
                if self.chans.remove(&dcid).is_some() && dcid == R_MEDIA {
                    self.n_avdtp = 1;
                }
            }
            0x0B | 0x07 => {}
            0x0A => {
                // Host info request: answer "not supported"
                let mut r = vec![0x0B, id, 4, 0];
                r.extend_from_slice(&body[..2.min(body.len())]);
                r.extend_from_slice(&[1, 0]);
                self.remote_send(1, &r);
            }
            _ => self.err(format!("unexpected L2CAP signalling code {code:#04x} len {}", body.len())),
        }
    }

    fn maybe_chan_ready(&mut self, local: u16) {
        let Some(c) = self.chans.get(&local) else { return };
        if !(c.cfg_in && c.cfg_out) {
            return;
        }
        match c.psm {
            0x01 => self.sdp_start(),
            0x17 => {
                // AVRCP controller behaviour of AirPods: ask for events, then register for notifications.
                let t = self.avctp_txn;
                self.avctp_txn = (t + 1) & 0x0F;
                let cmd = [t << 4, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x10, 0x00, 0x00, 0x01, 0x03];
                self.remote_send(R_AVCTP, &cmd);
            }
            _ => {}
        }
    }

    // ---------------- SDP client (replays the real AirPods queries) ----------------

    fn sdp_requests() -> Vec<Vec<u8>> {
        let u128a = [0x1c, 0x1f, 0xf3, 0x19, 0x36, 0x57, 0x2e, 0x4b, 0x36, 0xa2, 0xbf, 0xb2, 0x40, 0x9b, 0x1a, 0xa6, 0xf4];
        let u128b = [0x1c, 0x15, 0x19, 0x00, 0x01, 0x12, 0xf4, 0xc2, 0x26, 0x88, 0xed, 0x2a, 0xc5, 0x57, 0x9f, 0x2a, 0x85];
        let u128c = [0x1c, 0x47, 0x15, 0x65, 0x0b, 0x5e, 0x9d, 0x4a, 0xc2, 0xb8, 0x98, 0xa4, 0xfc, 0x0a, 0xa5, 0xdf, 0x78];
        let tail4 = [0x00, 0x40, 0x35, 0x03, 0x09, 0x00, 0x04, 0x00];
        let tail9 = [0x00, 0x40, 0x35, 0x03, 0x09, 0x00, 0x09, 0x00];
        let mk = |uuid: &[u8], tail: &[u8]| {
            let mut pat = vec![0x35, uuid.len() as u8];
            pat.extend_from_slice(uuid);
            let mut v = vec![0x06, 0, 0, 0, 0];
            v.extend_from_slice(&pat);
            v.extend_from_slice(tail);
            let plen = (v.len() - 5) as u16;
            v[3..5].copy_from_slice(&plen.to_be_bytes());
            v
        };
        vec![
            {
                // PnP info, attributes 0x0201.. as the real device asks
                let mut v = vec![0x06, 0, 0, 0, 0x1f, 0x35, 0x03, 0x19, 0x12, 0x00, 0x00, 0x40, 0x35, 0x15];
                v.extend_from_slice(&[
                    0x09, 0x02, 0x01, 0x09, 0x02, 0x02, 0x09, 0x02, 0x03, 0x09, 0x02, 0x05, 0x09, 0xa0, 0x00, 0x09, 0xa0, 0x01, 0x09, 0xaf,
                    0xff, 0x00,
                ]);
                v
            },
            mk(&[0x19, 0x11, 0x1f], &tail4),
            mk(&[0x19, 0x11, 0x0a], &tail4), // A2DP source: the one we must answer
            mk(&[0x19, 0x11, 0x0c], &tail9),
            mk(&u128a, &tail4),
            mk(&u128b, &tail9),
            mk(&[0x19, 0x18, 0x0a], &tail9),
            mk(&u128c, &tail9),
            mk(&[0x19, 0x18, 0x01], &tail9),
        ]
    }

    fn sdp_start(&mut self) {
        self.sdp_next = 0;
        self.sdp_expect.clear();
        self.sdp_send_next();
    }

    fn sdp_send_next(&mut self) {
        let reqs = Self::sdp_requests();
        if self.sdp_next >= reqs.len() {
            return;
        }
        let mut r = reqs[self.sdp_next].clone();
        let txn = (self.sdp_next as u16 + 1).to_be_bytes();
        r[1] = txn[0];
        r[2] = txn[1];
        let wants_a2dp = r.windows(3).any(|w| w == [0x19, 0x11, 0x0a]);
        self.sdp_expect.push(wants_a2dp);
        self.rep.lock().unwrap().sdp_requests += 1;
        self.remote_send(R_SDP, &r);
    }

    fn sdp_rx(&mut self, p: &[u8]) {
        let idx = self.sdp_next;
        let expect = self.sdp_expect.get(idx).copied().unwrap_or(false);
        let txn = (idx as u16 + 1).to_be_bytes();
        match sdp_validate(p, txn, expect) {
            Ok(()) => self.rep.lock().unwrap().sdp_ok += 1,
            Err(e) => self.err(format!("SDP response {idx} invalid: {e}")),
        }
        self.sdp_next += 1;
        self.sdp_send_next();
    }

    // ---------------- AVCTP / AVRCP ----------------

    fn avctp_rx(&mut self, p: &[u8]) {
        if p.len() < 4 || p[1..3] != [0x11, 0x0E] {
            self.err(format!("bad AVCTP header {p:02x?}"));
            return;
        }
        let is_rsp = p[0] & 2 != 0;
        let ctype = p[3] & 0x0F;
        if is_rsp {
            let pdu = p.get(9).copied().unwrap_or(0);
            let prm = if p.len() > 13 { &p[13..] } else { &[][..] };
            match (pdu, ctype) {
                (0x10, 0x0C) if prm.len() >= 4 && prm[0] == 0x03 && prm[1] as usize == prm.len() - 2 && prm[2..].contains(&0x01) => {
                    self.rep.lock().unwrap().avrcp_caps_ok = true;
                    // Next: register for playback status then track change, then ask play status.
                    let t = self.avctp_txn;
                    self.avctp_txn = (t + 1) & 0x0F;
                    self.remote_send(
                        R_AVCTP,
                        &[t << 4, 0x11, 0x0E, 0x03, 0x48, 0x00, 0x00, 0x19, 0x58, 0x31, 0x00, 0x00, 0x05, 0x01, 0, 0, 0, 0],
                    );
                }
                (0x31, 0x0F) if prm.len() >= 2 && prm[0] == 0x01 && prm[1] == 0x01 => {
                    self.rep.lock().unwrap().avrcp_notif_ok += 1;
                    let t = self.avctp_txn;
                    self.avctp_txn = (t + 1) & 0x0F;
                    self.remote_send(
                        R_AVCTP,
                        &[t << 4, 0x11, 0x0E, 0x03, 0x48, 0x00, 0x00, 0x19, 0x58, 0x31, 0x00, 0x00, 0x05, 0x02, 0, 0, 0, 0],
                    );
                }
                (0x31, 0x0F) if prm.len() == 9 && prm[0] == 0x02 => {
                    self.rep.lock().unwrap().avrcp_notif_ok += 1;
                    let t = self.avctp_txn;
                    self.avctp_txn = (t + 1) & 0x0F;
                    self.remote_send(R_AVCTP, &[t << 4, 0x11, 0x0E, 0x01, 0x48, 0x00, 0x00, 0x19, 0x58, 0x30, 0x00, 0x00, 0x00]);
                }
                (0x30, 0x0C) if prm.len() == 9 && prm[8] == 0x01 => self.rep.lock().unwrap().play_status_ok = true,
                (0x50, _) => {} // volume response handled below on the command side
                _ if p.get(5) == Some(&0x7C) => {
                    if ctype == 0x09 {
                        self.rep.lock().unwrap().pause_acked = true;
                    }
                }
                _ => self.err(format!("unexpected AVRCP response pdu {pdu:#04x} ctype {ctype:#x} {p:02x?}")),
            }
            return;
        }
        // Command from the host: only SetAbsoluteVolume is expected.
        let ok_hdr = p.len() >= 14
            && p[3] == 0x00
            && p[4] == 0x48
            && p[5] == 0x00
            && p[6..9] == [0, 0x19, 0x58]
            && p[9] == 0x50
            && p[10] == 0
            && p[11..13] == [0, 1];
        if ok_hdr {
            let vol = p[13];
            {
                let mut r = self.rep.lock().unwrap();
                r.volume = Some(vol);
                r.volume_at = Some(Instant::now());
            }
            let mut r = p.to_vec();
            r[0] |= 2;
            r[3] = 0x09;
            self.remote_send(R_AVCTP, &r);
        } else {
            self.err(format!("unexpected AVRCP command from host {p:02x?}"));
            let mut r = p.to_vec();
            r[0] |= 2;
            r[3] = 0x08;
            self.remote_send(R_AVCTP, &r);
        }
    }

    // ---------------- AVDTP sink ----------------

    fn avdtp_reply(&mut self, txn: u8, mt: u8, sig: u8, params: &[u8]) {
        let mut v = vec![(txn << 4) | (mt & 3), sig];
        v.extend_from_slice(params);
        self.remote_send(R_SIG, &v);
    }

    fn avdtp_rx(&mut self, p: &[u8]) {
        if p.len() < 2 {
            self.err("AVDTP message too short");
            return;
        }
        let (txn, pkt_type, mt, sig) = (p[0] >> 4, (p[0] >> 2) & 3, p[0] & 3, p[1] & 0x3F);
        if pkt_type != 0 {
            self.err(format!("AVDTP packet type {pkt_type} (must be single): header {:#04x}", p[0]));
            return;
        }
        if mt != 0 {
            // Response to a sink-initiated command (delay report): fine.
            return;
        }
        if crate::hci::trace() {
            eprintln!("[sim {:?}] AVDTP signal {sig:#04x} txn {txn} in state {:?}", Instant::now(), self.avdtp);
        }
        let prm = &p[2..];
        let active = self.active_seid;
        let seid_ok = move |b: &[u8]| !b.is_empty() && Some(b[0] >> 2) == active;
        let known = |b: &[u8]| !b.is_empty() && matches!(b[0] >> 2, 1..=3);
        match sig {
            0x01 if self.cfg.sbc_only => self.avdtp_reply(txn, 2, sig, &[0x04, 0x08]),
            0x01 => self.avdtp_reply(txn, 2, sig, &[0x04, 0x08, 0x08, 0x08, 0x0C, 0x08]),
            0x02 | 0x0C => {
                if !known(prm) {
                    self.avdtp_reply(txn, 3, sig, &[0x12]);
                    return;
                }
                let caps: Vec<u8> = match prm[0] >> 2 {
                    // SEID 1: SBC (as captured from the real AirPods)
                    1 => vec![0x01, 0x00, 0x07, 0x06, 0x00, 0x00, 0x3F, 0xFF, 0x02, 0x35, 0x08, 0x00],
                    // SEID 2: AAC, several object-type bits set like the AirPods do, 44.1/48 kHz, mono+stereo, VBR, 320 kbit/s
                    2 => vec![0x01, 0x00, 0x07, 0x08, 0x00, 0x02, 0xC0, 0x01, 0x8C, 0x84, 0xE2, 0x00],
                    // SEID 3: a vendor codec the host must skip
                    _ => vec![0x01, 0x00, 0x07, 0x08, 0x00, 0xFF, 0x4C, 0x00, 0x00, 0x00, 0x01, 0x00],
                };
                self.avdtp_reply(txn, 2, sig, &caps);
            }
            0x03 => {
                // ACP seid, INT seid, then capability items.
                if self.cfg.reject_config {
                    self.avdtp_reply(txn, 3, sig, &[0x07, 0x1F]);
                    return;
                }
                let seid = prm.first().map(|b| b >> 2).unwrap_or(0);
                if prm.len() < 4 || !matches!(seid, 1 | 2) || self.avdtp != AvdtpState::Idle {
                    self.err(format!("Set_Configuration in state {:?} / bad seid: {prm:02x?}", self.avdtp));
                    self.avdtp_reply(txn, 3, sig, &[0x00, 0x31]);
                    return;
                }
                let caps = &prm[2..];
                let (mut sbc_cfg, mut aac_cfg) = (None, None);
                let mut transport = false;
                let mut i = 0;
                while i + 2 <= caps.len() {
                    let (cat, len) = (caps[i], caps[i + 1] as usize);
                    if i + 2 + len > caps.len() {
                        break;
                    }
                    let d = &caps[i + 2..i + 2 + len];
                    if cat == 0x01 {
                        transport = true;
                    }
                    if cat == 0x08 {
                        self.delay_reporting = true;
                    }
                    if cat == 0x07 && len == 6 && d[0] == 0x00 && d[1] == 0x00 {
                        sbc_cfg = Some([d[2], d[3], d[4], d[5]]);
                    }
                    if cat == 0x07 && len == 8 && d[0] == 0x00 && d[1] == 0x02 {
                        aac_cfg = Some([d[2], d[3], d[4], d[5], d[6], d[7]]);
                    }
                    i += 2 + len;
                }
                if seid == 2 {
                    // AAC: exactly one object type, one frequency, one channel mode, within the advertised capabilities.
                    let valid = transport
                        && aac_cfg
                            .map(|c| {
                                let br = ((c[3] & 0x7F) as u32) << 16 | (c[4] as u32) << 8 | c[5] as u32;
                                c[0].count_ones() == 1
                                    && c[0] & 0xC0 != 0
                                    && c[1].count_ones() + (c[2] & 0xF0).count_ones() == 1
                                    && (c[1] & 0x01 != 0 || c[2] & 0x80 != 0)
                                    && (c[2] & 0x0C).count_ones() == 1
                                    && br <= 320_000
                            })
                            .unwrap_or(false);
                    if let (true, Some(c)) = (valid, aac_cfg) {
                        self.aac_rate = if c[2] & 0x80 != 0 { 48000 } else { 44100 };
                        match crate::aac::AacDecoder::new(self.aac_rate, 2) {
                            Ok(dec) => self.aac_dec = Some(dec),
                            Err(e) => self.err(format!("sim cannot create an AAC decoder: {e}")),
                        }
                        self.rep.lock().unwrap().configured_aac = Some(c);
                        self.codec = SimCodec::Aac;
                        self.active_seid = Some(2);
                        self.avdtp = AvdtpState::Configured;
                        self.avdtp_reply(txn, 2, sig, &[]);
                    } else {
                        self.err(format!("invalid AAC configuration {prm:02x?}"));
                        self.avdtp_reply(txn, 3, sig, &[0x07, 0xC1]);
                    }
                    return;
                }
                let valid = transport
                    && sbc_cfg
                        .map(|c| {
                            (c[0] & 0xF0).count_ones() == 1
                                && (c[0] & 0x0F).count_ones() == 1
                                && (c[1] & 0xF0).count_ones() == 1
                                && (c[1] & 0x0C).count_ones() == 1
                                && (c[1] & 3).count_ones() == 1
                                && c[2] >= 2
                                && c[3] <= 53
                                && c[2] <= c[3]
                        })
                        .unwrap_or(false);
                if valid {
                    self.rep.lock().unwrap().configured = sbc_cfg;
                    self.codec = SimCodec::Sbc;
                    self.active_seid = Some(1);
                    self.avdtp = AvdtpState::Configured;
                    self.avdtp_reply(txn, 2, sig, &[]);
                } else {
                    self.err(format!("invalid SBC configuration {prm:02x?}"));
                    self.avdtp_reply(txn, 3, sig, &[0x07, 0x1F]);
                }
            }
            0x05 if self.cfg.reject_reconfigure => {
                self.avdtp_reply(txn, 3, sig, &[0x00, 0x81]);
            }
            0x05 => {
                // Reconfigure: allowed only while Open (i.e. after Suspend); only the codec item may change.
                let mut sbc_cfg = None;
                let mut i = 1;
                while i + 2 <= prm.len() {
                    let (cat, len) = (prm[i], prm[i + 1] as usize);
                    if i + 2 + len > prm.len() {
                        break;
                    }
                    let d = &prm[i + 2..i + 2 + len];
                    if cat == 0x07 && len == 6 && d[0] == 0 && d[1] == 0 {
                        sbc_cfg = Some([d[2], d[3], d[4], d[5]]);
                    } else {
                        self.err(format!("Reconfigure carried a non-codec item {cat:#04x}"));
                    }
                    i += 2 + len;
                }
                let valid = sbc_cfg
                    .map(|c| {
                        (c[0] & 0xF0).count_ones() == 1
                            && (c[0] & 0x0F).count_ones() == 1
                            && (c[1] & 0xF0).count_ones() == 1
                            && (c[1] & 0x0C).count_ones() == 1
                            && (c[1] & 3).count_ones() == 1
                            && c[2] >= 2
                            && c[3] <= 53
                            && c[2] <= c[3]
                    })
                    .unwrap_or(false);
                if self.avdtp == AvdtpState::Open && seid_ok(prm) && valid {
                    self.rep.lock().unwrap().configured = sbc_cfg;
                    self.rep.lock().unwrap().reconfigs += 1;
                    self.avdtp_reply(txn, 2, sig, &[]);
                } else {
                    self.err(format!("Reconfigure rejected in state {:?}: {prm:02x?}", self.avdtp));
                    self.avdtp_reply(txn, 3, sig, &[0x07, 0x1F]);
                }
            }
            0x06 => {
                if self.avdtp == AvdtpState::Configured && seid_ok(prm) {
                    self.avdtp = AvdtpState::Open;
                    self.avdtp_reply(txn, 2, sig, &[]);
                    // AVDTP 1.3 sinks announce their buffer depth, but only if delay reporting was configured.
                    if self.delay_reporting {
                        let t = 7u8;
                        self.avdtp_reply(t, 0, 0x0D, &[0x04, 0x03, 0xE8]);
                    }
                } else {
                    self.err(format!("Open in state {:?}", self.avdtp));
                    self.avdtp_reply(txn, 3, sig, &[0x04, 0x31]);
                }
            }
            0x07 => {
                let media_open = self.chans.get(&R_MEDIA).map(|c| c.cfg_in && c.cfg_out).unwrap_or(false);
                if self.avdtp == AvdtpState::Open && seid_ok(prm) && media_open {
                    self.avdtp = AvdtpState::Streaming;
                    self.rep.lock().unwrap().start_at = Some(Instant::now());
                    if let Some(ms) = self.cfg.drop_after_start_ms {
                        self.at(ms, Out::Evt(vec![0x05, 4, 0, 0x32, 0, 0x13]));
                    }
                    // A new Start begins a fresh stream: sequence, timestamp and playout state restart, and the
                    // "media is late" clock runs from this Start, not from the previous stream's last packet.
                    self.rep.lock().unwrap().last_media_at = None;
                    self.last_seq = None;
                    self.last_ts = None;
                    self.last_samples = 0;
                    self.play_start = None;
                    self.rx_samples = 0;
                    self.avdtp_reply(txn, 2, sig, &[]);
                } else {
                    self.err(format!("Start in state {:?} (media channel open: {media_open})", self.avdtp));
                    self.avdtp_reply(txn, 3, sig, &[0x04, 0x31]);
                }
            }
            0x09 => {
                self.avdtp = AvdtpState::Open;
                self.rep.lock().unwrap().suspended = true;
                self.avdtp_reply(txn, 2, sig, &[]);
            }
            0x08 | 0x0A => {
                self.avdtp = AvdtpState::Idle;
                self.active_seid = None;
                self.aac_dec = None;
                self.avdtp_reply(txn, 2, sig, &[]);
            }
            _ => {
                self.err(format!("unsupported AVDTP signal {sig:#04x}"));
                self.avdtp_reply(txn, 3, sig, &[0x01]);
            }
        }
    }

    // ---------------- media ----------------

    fn media_rx(&mut self, p: &[u8]) {
        let now = Instant::now();
        if self.avdtp != AvdtpState::Streaming {
            self.err("media received outside the Streaming state");
            return;
        }
        if self.rep.lock().unwrap().paused_by_sink {
            self.rep.lock().unwrap().media_dropped_after_pause += 1;
            return;
        }
        let decoded = match self.codec {
            SimCodec::Sbc => self.decode_sbc_media(p),
            SimCodec::Aac => self.decode_aac_media(p),
        };
        let Some((seq, ts, samples, nframes, pcm_out)) = decoded else {
            return;
        };
        // Playout buffer model: playback starts `sink_prefill_ms` after the first packet and consumes
        // audio in real time. Audio arriving slower than that drains the buffer: an underrun.
        {
            let rate = match self.codec {
                SimCodec::Aac => self.aac_rate as f64,
                SimCodec::Sbc => {
                    if self.rep.lock().unwrap().configured.map(|c| c[0] & 0x20 != 0).unwrap_or(false) {
                        44100.0
                    } else {
                        48000.0
                    }
                }
            };
            let prefill = Duration::from_millis(self.cfg.sink_prefill_ms);
            let mut r = self.rep.lock().unwrap();
            match self.play_start {
                None => self.play_start = Some(now + prefill),
                Some(ps) if now > ps => {
                    let consumed = now.duration_since(ps).as_secs_f64() * rate;
                    if consumed > self.rx_samples as f64 {
                        r.underruns += 1;
                        let since_start = r.start_at.map(|t| now.duration_since(t).as_millis() as u64).unwrap_or(0);
                        r.underrun_at_ms.push(since_start);
                        self.play_start = Some(now + prefill);
                        self.rx_samples = 0;
                    } else {
                        r.max_buffer_ms = r.max_buffer_ms.max((self.rx_samples as f64 - consumed) / rate * 1000.0);
                    }
                }
                _ => {}
            }
            self.rx_samples += samples as u64;
        }
        // Sequence / timestamp discipline, like a real jitter buffer.
        let mut r = self.rep.lock().unwrap();
        if let Some(last) = self.last_seq {
            let d = seq.wrapping_sub(last) as i16;
            if d == 1 {
            } else if d > 1 {
                r.seq_gaps += (d - 1) as u64;
            } else {
                r.seq_backwards += 1;
            }
            if let Some(lts) = self.last_ts {
                if ts != lts.wrapping_add(self.last_samples) && d == 1 {
                    r.ts_errors += 1;
                }
            }
        }
        self.last_seq = Some(seq);
        self.last_ts = Some(ts);
        self.last_samples = samples;
        if let Some(prev) = r.last_media_at {
            r.max_arrival_gap_ms = r.max_arrival_gap_ms.max(now.duration_since(prev).as_secs_f64() * 1000.0);
        }
        r.media_pkts += 1;
        r.first_media_at.get_or_insert(now);
        r.last_media_at = Some(now);
        r.arrivals.push((seq, ts, now, nframes));
        r.pcm.extend(pcm_out);
    }

    /// Parse and decode an SBC media packet. Returns (seq, timestamp, samples per channel, frames, left-channel PCM).
    fn decode_sbc_media(&mut self, p: &[u8]) -> Option<(u16, u32, u32, u8, Vec<i16>)> {
        if p.len() < 13 || p[0] != 0x80 || (p[1] & 0x7F) != 96 {
            self.err(format!("bad RTP header {:02x?}", &p[..p.len().min(14)]));
            return None;
        }
        let (seq, ts) = (u16::from_be_bytes([p[2], p[3]]), u32::from_be_bytes([p[4], p[5], p[6], p[7]]));
        let hdr = p[12];
        if hdr & 0xF0 != 0 {
            self.err(format!("SBC payload header has fragmentation bits set: {hdr:#04x}"));
            return None;
        }
        let nframes = hdr & 0x0F;
        let mut off = 13;
        let mut samples = 0u32;
        let mut pcm_out = Vec::new();
        let mut buf = vec![0u8; 2048];
        for _ in 0..nframes {
            if let Err(e) = self.check_frame_header(&p[off..]) {
                self.err(e);
                return None;
            }
            let Some((used, wrote)) = self.dec.decode_frame(&p[off..], &mut buf) else {
                self.err("undecodable SBC frame");
                return None;
            };
            for s in buf[..wrote].chunks(4) {
                pcm_out.push(i16::from_le_bytes([s[0], s[1]]));
            }
            samples += (wrote / 4) as u32;
            off += used;
        }
        if off != p.len() {
            self.err(format!("{} trailing bytes after {nframes} SBC frames", p.len() - off));
        }
        Some((seq, ts, samples, nframes, pcm_out))
    }

    /// Parse and decode an AAC media packet: RTP (marker set) + one LATM AudioMuxElement carrying one AAC frame.
    fn decode_aac_media(&mut self, p: &[u8]) -> Option<(u16, u32, u32, u8, Vec<i16>)> {
        if p.len() < 14 || p[0] != 0x80 || p[1] != 0xE0 {
            self.err(format!("bad AAC RTP header (needs V=2, M=1, PT=96): {:02x?}", &p[..p.len().min(14)]));
            return None;
        }
        let (seq, ts) = (u16::from_be_bytes([p[2], p[3]]), u32::from_be_bytes([p[4], p[5], p[6], p[7]]));
        let (ri, ch, raw) = match crate::aac::latm_demux(&p[12..]) {
            Ok(x) => x,
            Err(e) => {
                self.err(format!("undecodable LATM payload: {e}"));
                return None;
            }
        };
        if crate::aac::index_rate(ri) != Some(self.aac_rate) || ch != 2 {
            self.err(format!("LATM stream config (rate index {ri}, {ch} ch) disagrees with the negotiated {} Hz stereo", self.aac_rate));
            return None;
        }
        let Some(dec) = self.aac_dec.as_mut() else {
            self.err("AAC media but no decoder (configuration failed?)");
            return None;
        };
        let pcm = match dec.decode(&raw) {
            Ok(v) => v,
            Err(e) => {
                self.err(format!("undecodable AAC frame: {e}"));
                return None;
            }
        };
        self.rep.lock().unwrap().aac_frames += 1;
        let left: Vec<i16> = pcm.chunks(2).map(|c| c[0]).collect();
        Some((seq, ts, crate::aac::FRAME_SAMPLES as u32, 1, left))
    }

    /// A strict sink rejects frames whose SBC header disagrees with the negotiated configuration.
    fn check_frame_header(&mut self, f: &[u8]) -> std::result::Result<(), String> {
        let Some(cfg) = self.rep.lock().unwrap().configured else { return Err("media before configuration".into()) };
        if f.len() < 4 || f[0] != 0x9C {
            return Err(format!("SBC frame without syncword: {:02x?}", &f[..f.len().min(4)]));
        }
        let freq = match cfg[0] & 0xF0 {
            0x20 => 2,
            0x10 => 3,
            0x40 => 1,
            _ => 0,
        };
        let blocks = match cfg[1] & 0xF0 {
            0x80 => 0,
            0x40 => 1,
            0x20 => 2,
            _ => 3,
        };
        let mode = match cfg[0] & 0x0F {
            0x08 => 0,
            0x04 => 1,
            0x02 => 2,
            _ => 3,
        };
        let alloc = u8::from(cfg[1] & 3 == 2);
        let sb = u8::from(cfg[1] & 0x0C == 0x04);
        let (h_freq, h_blocks, h_mode, h_alloc, h_sb) = (f[1] >> 6, (f[1] >> 4) & 3, (f[1] >> 2) & 3, (f[1] >> 1) & 1, f[1] & 1);
        if (h_freq, h_blocks, h_mode, h_alloc, h_sb) != (freq, blocks, mode, alloc, sb) {
            return Err(format!("SBC frame header {:#04x} disagrees with the negotiated config {cfg:02x?}", f[1]));
        }
        if f[2] < cfg[2] || f[2] > cfg[3] {
            return Err(format!("SBC bitpool {} outside negotiated {}..={}", f[2], cfg[2], cfg[3]));
        }
        self.rep.lock().unwrap().frames_decoded += 1;
        Ok(())
    }

    fn check_deadlines(&mut self) {
        if self.avdtp != AvdtpState::Streaming {
            return;
        }
        let (late, already) = {
            let r = self.rep.lock().unwrap();
            let now = Instant::now();
            let reference = r.last_media_at.or(r.start_at);
            (reference.map(|t| now.duration_since(t) > self.cfg.media_deadline).unwrap_or(false), r.paused_by_sink)
        };
        if late && !already {
            {
                let mut r = self.rep.lock().unwrap();
                let now = Instant::now();
                r.pause_info = Some(format!(
                    "codec {:?}, state {:?}, {:?} ms since last media, {:?} ms since Start, media packets so far {}",
                    self.codec,
                    self.avdtp,
                    r.last_media_at.map(|t| now.duration_since(t).as_millis()),
                    r.start_at.map(|t| now.duration_since(t).as_millis()),
                    r.media_pkts
                ));
            }
            self.rep.lock().unwrap().paused_by_sink = true;
            // What the real AirPods did: ask the source to pause.
            if self.chans.get(&R_AVCTP).map(|c| c.cfg_in && c.cfg_out).unwrap_or(false) {
                let t = self.avctp_txn;
                self.avctp_txn = (t + 1) & 0x0F;
                self.remote_send(R_AVCTP, &[t << 4, 0x11, 0x0E, 0x00, 0x48, 0x7C, 0x46, 0x00]);
            }
        }
    }
}

// ---------------- SDP data-element parsing (test oracle) ----------------

#[derive(Debug, Clone, PartialEq)]
pub enum De {
    Uint(u64),
    Uuid(u128),
    Text(Vec<u8>),
    Seq(Vec<De>),
}

pub fn parse_de(b: &[u8]) -> Option<(De, usize)> {
    let h = *b.first()?;
    let (ty, sz) = (h >> 3, h & 7);
    let (hdr, len) = match sz {
        0..=4 => (1, 1usize << sz),
        5 => (2, *b.get(1)? as usize),
        6 => (3, u16::from_be_bytes([*b.get(1)?, *b.get(2)?]) as usize),
        7 => (5, u32::from_be_bytes([*b.get(1)?, *b.get(2)?, *b.get(3)?, *b.get(4)?]) as usize),
        _ => return None,
    };
    // type 0 (nil) has size index 0 => len 1 would be wrong, treat as unsupported
    let body = b.get(hdr..hdr + len)?;
    let de = match ty {
        1 | 2 => De::Uint(body.iter().fold(0u64, |a, x| (a << 8) | *x as u64)),
        3 => De::Uuid(body.iter().fold(0u128, |a, x| (a << 8) | *x as u128)),
        4 => De::Text(body.to_vec()),
        6 => {
            let mut v = vec![];
            let mut o = 0;
            while o < body.len() {
                let (d, n) = parse_de(&body[o..])?;
                v.push(d);
                o += n;
            }
            De::Seq(v)
        }
        _ => return None,
    };
    Some((de, hdr + len))
}

/// Validate a ServiceSearchAttributeResponse; `expect_record` = must hold our A2DP Source record.
pub fn sdp_validate(p: &[u8], txn: [u8; 2], expect_record: bool) -> std::result::Result<(), String> {
    if p.len() < 7 || p[0] != 0x07 {
        return Err(format!("not a ServiceSearchAttributeResponse: {:02x?}", &p[..p.len().min(8)]));
    }
    if p[1..3] != txn {
        return Err("transaction id mismatch".into());
    }
    let plen = u16::from_be_bytes([p[3], p[4]]) as usize;
    if p.len() != 5 + plen {
        return Err(format!("parameter length {plen} != actual {}", p.len() - 5));
    }
    let bytecount = u16::from_be_bytes([p[5], p[6]]) as usize;
    if 7 + bytecount + 1 != p.len() || p[7 + bytecount] != 0 {
        return Err("attribute byte count / continuation state wrong".into());
    }
    let (lists, n) = parse_de(&p[7..7 + bytecount]).ok_or("attribute lists unparsable")?;
    if n != bytecount {
        return Err("trailing bytes after attribute lists".into());
    }
    let De::Seq(records) = lists else { return Err("attribute lists is not a sequence".into()) };
    if !expect_record {
        return if records.is_empty() { Ok(()) } else { Err("returned a record for a service we do not provide".into()) };
    }
    if records.len() != 1 {
        return Err(format!("expected exactly one record, got {}", records.len()));
    }
    let De::Seq(attrs) = &records[0] else { return Err("record is not a sequence".into()) };
    let mut map = HashMap::new();
    for kv in attrs.chunks(2) {
        if let [De::Uint(id), v] = kv {
            map.insert(*id, v.clone());
        } else {
            return Err("attribute id/value pairs malformed".into());
        }
    }
    let class = map.get(&0x0001).ok_or("no ServiceClassIDList")?;
    if *class != De::Seq(vec![De::Uuid(0x110A)]) {
        return Err(format!("wrong service class {class:?}"));
    }
    let proto = map.get(&0x0004).ok_or("no ProtocolDescriptorList")?;
    let want = De::Seq(vec![De::Seq(vec![De::Uuid(0x0100), De::Uint(0x0019)]), De::Seq(vec![De::Uuid(0x0019), De::Uint(0x0103)])]);
    if *proto != want {
        return Err(format!("wrong protocol descriptors {proto:?}"));
    }
    let prof = map.get(&0x0009).ok_or("no BluetoothProfileDescriptorList")?;
    if *prof != De::Seq(vec![De::Seq(vec![De::Uuid(0x110D), De::Uint(0x0103)])]) {
        return Err(format!("wrong profile descriptor {prof:?}"));
    }
    if !matches!(map.get(&0x0000), Some(De::Uint(_))) {
        return Err("no ServiceRecordHandle".into());
    }
    Ok(())
}

/// `airlow simlive [seconds] [tone_hz]`: the whole live pipeline with REAL Windows loopback capture, but a
/// simulated controller and sink instead of a radio. Play a pure tone in Windows while it runs; the sink's
/// decoded audio is compared with an ideal sine to expose capture glitches and encoding damage.
pub fn simlive(secs: u32, tone_hz: f64) -> Result<()> {
    use crate::a2dp::{Link, StreamOpts};
    let (mut h, sim) = spawn(SimConfig::default());
    let kf = std::env::temp_dir().join("airlow_simlive_key.txt");
    let hd = crate::pair(&mut h, &kf)?;
    let mut l = Link::new(&mut h, hd)?;
    let o = StreamOpts {
        codec: if std::env::var("AIRLOW_CODEC").map(|v| v.eq_ignore_ascii_case("aac")).unwrap_or(false) {
            crate::a2dp::Codec::Aac
        } else {
            crate::a2dp::Codec::Sbc
        },
        aac_bitrate: 160_000,
        cfg: sbc::Config {
            rate: crate::live::loopback_rate()?,
            mode: sbc::Mode::JointStereo,
            blocks: 16,
            subbands: 8,
            bitpool: 53,
            snr: false,
        },
        frames_per_packet: 2,
        flush_ms: 40.0,
        seconds: secs,
    };
    // The production path (capture and AAC encoder before Start, codec chosen by AIRLOW_CODEC).
    crate::live::stream_live(&mut l, &o)?;
    let stats = crate::live::LiveStats::default();
    l.pump(Duration::from_millis(50));
    let r = sim.report();
    println!("\n== simlive result ==");
    println!(
        "capture xruns: {}   dropped: {}   sink underruns: {}   seq gaps {}   protocol errors {}",
        crate::live::CAPTURE_XRUNS.load(std::sync::atomic::Ordering::Relaxed),
        stats.dropped,
        r.underruns,
        r.seq_gaps,
        r.protocol_errors.len()
    );
    let fs = if matches!(o.cfg.rate, sbc::Rate::Hz48000) { 48000.0 } else { 44100.0 };
    let win = (fs * 0.25) as usize;
    let mut shown = 0;
    for (i, w) in r.pcm.chunks_exact(win).enumerate() {
        let peak = w.iter().map(|v| v.unsigned_abs()).max().unwrap_or(0);
        if peak < 500 {
            continue; // silence
        }
        let (snr, amp) = sine_snr(w, tone_hz, fs);
        println!(
            "  t={:5.2}s  peak {:5}  tone amp {:6.0}  SNR {:5.1} dB{}",
            i as f64 * 0.25,
            peak,
            amp,
            snr,
            if snr < 25.0 { "   <-- dirty" } else { "" }
        );
        shown += 1;
    }
    if shown == 0 {
        println!("  (no audio reached the sink: play a tone in Windows while this runs)");
    }
    Ok(())
}

/// Least-squares fit of a sine at `f` Hz; returns (SNR in dB of the fit, fitted amplitude).
pub fn sine_snr(pcm: &[i16], f: f64, fs: f64) -> (f64, f64) {
    let n = pcm.len() as f64;
    let (mut c, mut s) = (0.0, 0.0);
    for (i, &v) in pcm.iter().enumerate() {
        let w = std::f64::consts::TAU * f * i as f64 / fs;
        c += v as f64 * w.cos();
        s += v as f64 * w.sin();
    }
    let (a, b) = (2.0 * c / n, 2.0 * s / n);
    let (mut sig, mut noise) = (0.0, 0.0);
    for (i, &v) in pcm.iter().enumerate() {
        let w = std::f64::consts::TAU * f * i as f64 / fs;
        let fit = a * w.cos() + b * w.sin();
        sig += fit * fit;
        noise += (v as f64 - fit).powi(2);
    }
    (10.0 * (sig / noise.max(1e-9)).log10(), (a * a + b * b).sqrt())
}

// =============================================================================================
// Tests
// =============================================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::a2dp::{self, Link, Pace, StreamOpts};
    use crate::live;
    use crate::proto;
    use std::path::PathBuf;

    fn kf(tag: &str) -> PathBuf {
        std::env::temp_dir().join(format!("airlow_t_{}_{}.txt", std::process::id(), tag))
    }

    fn opts(seconds: u32, fpp: usize) -> StreamOpts {
        StreamOpts {
            codec: a2dp::Codec::Sbc,
            aac_bitrate: 160_000,
            cfg: sbc::Config { rate: sbc::Rate::Hz48000, mode: sbc::Mode::JointStereo, blocks: 8, subbands: 8, bitpool: 53, snr: false },
            frames_per_packet: fpp,
            flush_ms: 40.0,
            seconds,
        }
    }

    /// Pair against a fresh simulator; returns the live (encrypted) link handle.
    fn paired(cfg: SimConfig, tag: &str) -> (Hci, SimHandle, u16, PathBuf) {
        crate::realtime_tuning();
        let (mut h, sim) = spawn(cfg);
        let k = kf(tag);
        let hd = crate::pair(&mut h, &k).expect("pairing against the simulator");
        (h, sim, hd, k)
    }

    /// Amplitude of a pure tone at `f` Hz inside `pcm` (single-bin DFT).
    fn amp_at(pcm: &[i16], f: f64, rate: f64) -> f64 {
        let (mut re, mut im) = (0.0, 0.0);
        for (n, &v) in pcm.iter().enumerate() {
            let w = std::f64::consts::TAU * f * n as f64 / rate;
            re += v as f64 * w.cos();
            im -= v as f64 * w.sin();
        }
        2.0 * (re * re + im * im).sqrt() / pcm.len() as f64
    }

    fn clean(r: &Report) {
        assert!(r.protocol_errors.is_empty(), "sink flagged protocol errors: {:#?}", r.protocol_errors);
        assert_eq!(r.pb_violations, 0);
        assert_eq!(r.seq_gaps, 0, "RTP sequence gaps");
        assert_eq!(r.seq_backwards, 0, "RTP sequence went backwards");
        assert_eq!(r.ts_errors, 0, "RTP timestamp discontinuities");
        assert!(!r.paused_by_sink, "sink paused the stream");
    }

    // ----- oracle self-tests: the checker must reject bad input, or passing tests mean nothing -----

    #[test]
    fn sdp_oracle_accepts_empty_and_rejects_malformed() {
        let ok_empty = [0x07, 0, 1, 0, 5, 0, 2, 0x35, 0, 0];
        assert!(sdp_validate(&ok_empty, [0, 1], false).is_ok());
        assert!(sdp_validate(&ok_empty, [0, 1], true).is_err(), "empty list must not satisfy an A2DP query");
        assert!(sdp_validate(&ok_empty, [0, 2], false).is_err(), "txn mismatch");
        let mut bad_len = ok_empty;
        bad_len[4] = 9;
        assert!(sdp_validate(&bad_len, [0, 1], false).is_err(), "bad parameter length");
        let mut bad_cont = ok_empty;
        bad_cont[9] = 1;
        assert!(sdp_validate(&bad_cont, [0, 1], false).is_err(), "continuation state must be 0");
    }

    #[test]
    fn data_element_parser_roundtrips_nested_sequences() {
        let b = [0x35, 0x08, 0x35, 0x06, 0x19, 0x11, 0x0D, 0x09, 0x01, 0x03];
        let (de, n) = parse_de(&b).unwrap();
        assert_eq!(n, b.len());
        assert_eq!(de, De::Seq(vec![De::Seq(vec![De::Uuid(0x110D), De::Uint(0x0103)])]));
    }

    // ----- pairing / connection -----

    #[test]
    fn pairing_creates_a_bond_that_reconnects() {
        let (_h, sim, _hd, k) = paired(SimConfig::default(), "pair");
        let r = sim.report();
        assert!(r.encrypted, "link must be encrypted");
        let issued = r.link_key_issued.expect("sink issued a link key");
        let (addr, key) = crate::load_key(&k).expect("key file written");
        assert_eq!(addr, SINK_ADDR);
        assert_eq!(key, issued, "saved key must match the one the sink issued");
        assert!(r.protocol_errors.is_empty(), "{:?}", r.protocol_errors);

        // A brand-new session that already holds the bond reconnects with the stored key.
        let (mut h2, sim2) = spawn(SimConfig { bonded_key: Some(key), ..Default::default() });
        crate::init(&mut h2).unwrap();
        let hd = a2dp::connect(&mut h2, &addr, &key).expect("reconnect with stored key");
        assert_eq!(hd, HANDLE);
        assert!(sim2.report().encrypted);

        // A wrong key must fail loudly (not hang).
        let (mut h3, _sim3) = spawn(SimConfig { bonded_key: Some(key), ..Default::default() });
        crate::init(&mut h3).unwrap();
        let err = a2dp::connect(&mut h3, &addr, &[0u8; 16]).unwrap_err().to_string();
        assert!(err.contains("authentication failed"), "{err}");
    }

    #[test]
    fn stale_events_from_a_previous_process_are_ignored() {
        let (_h, sim, _hd, _k) = paired(SimConfig { stale_events: true, ..Default::default() }, "stale");
        assert!(sim.report().encrypted);
        assert!(sim.report().protocol_errors.is_empty());
    }

    #[test]
    fn a_sink_that_connects_by_itself_is_adopted() {
        let key = [7u8; 16];
        let (mut h, sim) = spawn(SimConfig { incoming_after_reset: true, bonded_key: Some(key), ..Default::default() });
        crate::realtime_tuning();
        crate::init(&mut h).unwrap();
        let hd = a2dp::connect(&mut h, &SINK_ADDR, &key).expect("adopt incoming link");
        assert_eq!(hd, HANDLE);
        let mut l = Link::new(&mut h, hd).unwrap();
        a2dp::stream_tone(&mut l, &opts(1, 3)).unwrap();
        clean(&sim.report());
    }

    // ----- the full A2DP session -----

    #[test]
    fn full_session_plays_every_experiment_correctly() {
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "full");
        let mut l = Link::new(&mut h, hd).unwrap();
        a2dp::experiments(&mut l, &opts(4, 3)).unwrap(); // 1 s per variant
        l.pump(Duration::from_millis(20));
        l.service().unwrap();
        assert_eq!(l.sink_delay_ms, Some(100.0), "delay reporting must be enabled and the sink's buffer depth recorded");
        let r = sim.report();
        clean(&r);

        // Negotiation
        assert_eq!(r.configured, Some([0x11, 0x45, 53, 53]), "SBC config: 48k joint, 8 blocks, 8 sb, loudness, bitpool 53");
        // SDP: all 9 real AirPods queries answered correctly (record only for A2DP Source)
        assert_eq!(r.sdp_requests, 9);
        assert_eq!(r.sdp_ok, 9, "every SDP response must validate");
        // AVRCP
        assert!(r.avrcp_caps_ok, "GetCapabilities(events) reply rejected");
        assert_eq!(r.avrcp_notif_ok, 2, "both RegisterNotification interims");
        assert!(r.play_status_ok, "GetPlayStatus reply");
        assert_eq!(r.volume, Some(0x30));
        // The ordering that fixed "silent AirPods": volume BEFORE Start, media IMMEDIATELY after.
        let (vol, start, first) = (r.volume_at.unwrap(), r.start_at.unwrap(), r.first_media_at.unwrap());
        assert!(vol < start, "volume must be set before AVDTP Start");
        assert!(first.duration_since(start) < Duration::from_millis(120), "first media {:?} after Start", first.duration_since(start));
        assert!(!r.suspended || r.paused_by_sink == false);

        // The audio itself: each variant decodes to its own pitch at its own amplitude.
        let per = (4u32 as f32 / 4.0).max(0.25);
        let variants = [(440.0, 6000.0, 3usize), (660.0, 6000.0, 3), (880.0, 6000.0, 1), (1100.0, 12000.0, 2)];
        let freqs = [440.0, 660.0, 880.0, 1100.0];
        let mut start_idx = 0usize;
        for (i, (f, a, fpp)) in variants.iter().enumerate() {
            let pkt_samples = 64 * fpp;
            let pkt_dur = Duration::from_secs_f64(pkt_samples as f64 / 48000.0);
            let n = (per as f64 / pkt_dur.as_secs_f64()) as usize;
            let len = n * pkt_samples;
            let seg = &r.pcm[start_idx + 4000..start_idx + len - 4000];
            let amps: Vec<f64> = freqs.iter().map(|x| amp_at(seg, *x, 48000.0)).collect();
            let best = amps.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
            assert_eq!(freqs[best], *f, "variant {i}: wrong pitch, amps {amps:?}");
            assert!((amps[best] - a).abs() < 0.12 * a, "variant {i}: amplitude {:.0} vs {a}", amps[best]);
            start_idx += len;
        }
        assert_eq!(r.pcm.len(), start_idx, "every packet must arrive and decode");

        // Pacing: the 4 ms packets of variant A arrive at the right average rate.
        let a: Vec<_> = r.arrivals.iter().take(200).collect();
        let span = a.last().unwrap().2.duration_since(a[0].2).as_secs_f64() / (a.len() - 1) as f64 * 1000.0;
        println!("variant A mean inter-arrival {span:.3} ms (nominal 4.000)");
        assert!((span - 4.0).abs() < 0.3, "mean inter-arrival {span} ms");
    }

    // ----- regressions for bugs found on real hardware / in review -----

    #[test]
    fn regression_media_late_after_start_makes_the_sink_pause() {
        // The old code blocked ~2 s between AVDTP Start and the first packet. The AirPods answered
        // with AVRCP PAUSE. The simulator models that, so this proves it would catch the bug.
        let cfg = SimConfig { media_deadline: Duration::from_millis(300), ..Default::default() };
        let (mut h, sim, hd, _k) = paired(cfg, "late");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(1, 3);
        let (_s, _e, media) = a2dp::open_stream(&mut l, &o).unwrap();
        std::thread::sleep(Duration::from_millis(700)); // dead air after Start
        l.pump(Duration::from_millis(50));
        l.service().unwrap();
        let mut st = Pace { seq: 0, ts: 0, next: Instant::now() + Duration::from_millis(5), phase: 0.0 };
        a2dp::burst(&mut l, &mut st, media, o.cfg, 3, 440.0, 5000.0, 0.2, true).unwrap();
        l.pump(Duration::from_millis(50));
        l.service().unwrap();
        let r = sim.report();
        assert!(r.paused_by_sink, "sim must model the sink giving up");
        assert!(r.media_dropped_after_pause > 0);
        assert!(r.pause_acked, "we must acknowledge the sink's PAUSE key");
    }

    #[test]
    fn regression_rtp_sequence_reset_between_bursts_is_detected() {
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "seqbad");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(1, 3);
        let (_s, _e, media) = a2dp::open_stream(&mut l, &o).unwrap();
        for _ in 0..2 {
            // Old behaviour: fresh sequence/timestamp state per burst.
            let mut st = Pace { seq: 0, ts: 0, next: Instant::now() + Duration::from_millis(5), phase: 0.0 };
            a2dp::burst(&mut l, &mut st, media, o.cfg, 3, 440.0, 5000.0, 0.2, true).unwrap();
        }
        l.pump(Duration::from_millis(50));
        assert!(sim.report().seq_backwards >= 1, "the sink must notice the sequence number going backwards");
    }

    #[test]
    fn rtp_state_stays_continuous_across_bursts() {
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "seqok");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(1, 3);
        let (_s, _e, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let mut st = Pace { seq: 0, ts: 0, next: Instant::now() + Duration::from_millis(5), phase: 0.0 };
        for (f, fpp) in [(440.0, 3usize), (660.0, 1), (880.0, 2)] {
            a2dp::burst(&mut l, &mut st, media, o.cfg, fpp, f, 5000.0, 0.3, true).unwrap();
        }
        l.pump(Duration::from_millis(50));
        clean(&sim.report());
    }

    #[test]
    fn link_policy_forbids_sniff_and_role_switch() {
        // The sim flags Write_Link_Policy_Settings unless it is exactly "nothing allowed".
        let (_h, sim, _hd, _k) = paired(SimConfig::default(), "policy");
        let r = sim.report();
        assert!(r.protocol_errors.is_empty(), "{:?}", r.protocol_errors);
    }

    // ----- controller behaviour: fragmentation, credits, flush, dead radio -----

    #[test]
    fn small_controller_buffers_force_fragmentation_and_still_work() {
        let (mut h, sim, hd, _k) = paired(SimConfig { acl_mtu: 64, ..Default::default() }, "frag");
        let mut l = Link::new(&mut h, hd).unwrap();
        a2dp::stream_tone(&mut l, &opts(1, 3)).unwrap();
        let r = sim.report();
        clean(&r);
        assert_eq!(r.sdp_ok, 9);
        assert!((amp_at(&r.pcm[4000..r.pcm.len() - 4000], 440.0, 48000.0) - 6000.0).abs() < 700.0);
    }

    #[test]
    fn controller_credits_are_never_exceeded() {
        let cfg = SimConfig { credits: 2, air_bytes_per_sec: 150_000, ..Default::default() };
        let (mut h, sim, hd, _k) = paired(cfg, "credits");
        let mut l = Link::new(&mut h, hd).unwrap();
        a2dp::stream_tone(&mut l, &opts(1, 3)).unwrap();
        let r = sim.report();
        assert!(r.max_outstanding <= 2, "outstanding {}", r.max_outstanding);
        clean(&r);
    }

    #[test]
    fn auto_flush_drops_stale_audio_without_stalling() {
        // Radio slower than the 54 kB/s stream: with flushing, stale packets are dropped, never a stall.
        let cfg = SimConfig { air_bytes_per_sec: 20_000, ..Default::default() };
        let (mut h, sim, hd, _k) = paired(cfg, "flush");
        let mut l = Link::new(&mut h, hd).unwrap();
        a2dp::stream_tone(&mut l, &opts(2, 3)).unwrap();
        l.pump(Duration::from_millis(100));
        let r = sim.report();
        assert!(r.flushed > 0, "slow radio must trigger auto-flush");
        assert!(r.media_pkts < 500, "some packets must have been dropped");
        assert!(l.flushed > 0, "host must see Flush Occurred events");
        assert!(r.protocol_errors.is_empty(), "{:?}", r.protocol_errors);
    }

    #[test]
    fn dead_radio_is_reported_not_hung() {
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "dead");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(1, 3);
        let (_s, _e, media) = a2dp::open_stream(&mut l, &o).unwrap();
        sim.set_air(1); // 1 byte/s: nothing completes
        let t = Instant::now();
        let mut st = Pace { seq: 0, ts: 0, next: Instant::now(), phase: 0.0 };
        let e = a2dp::burst(&mut l, &mut st, media, o.cfg, 3, 440.0, 5000.0, 2.0, false).unwrap_err().to_string();
        assert!(e.contains("no ACL credits"), "{e}");
        assert!(t.elapsed() < Duration::from_secs(5), "must give up within seconds, took {:?}", t.elapsed());
        assert!(e.contains("rssi") && e.contains("link_quality"), "diagnostics missing: {e}");
    }

    // ----- live capture path -----

    fn run_live(chunk_ms: u64, tag: &str) -> (Report, live::LiveStats, Duration) {
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), tag);
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(3, 2);
        let (sig, seid, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let q: live::Q = Arc::new(Mutex::new(VecDeque::new()));
        let click: Arc<Mutex<Option<Instant>>> = Arc::new(Mutex::new(None));
        let (qf, cf) = (q.clone(), click.clone());
        let feeder = std::thread::spawn(move || {
            let t0 = Instant::now();
            let per_chunk = (48 * chunk_ms) as usize;
            let mut n = 0u64;
            while t0.elapsed() < Duration::from_millis(2600) {
                let due = t0 + Duration::from_millis(chunk_ms * (n + 1));
                while Instant::now() < due {
                    std::thread::sleep(Duration::from_micros(300));
                }
                let mut g = qf.lock().unwrap();
                let is_click = chunk_ms * n >= 1200 && cf.lock().unwrap().is_none();
                for i in 0..per_chunk {
                    let v = if is_click && i < 96 { ((i as f32 * 0.26).sin() * 20000.0) as i16 } else { 0 };
                    g.push_back(v);
                    g.push_back(v);
                }
                if is_click {
                    *cf.lock().unwrap() = Some(Instant::now());
                }
                n += 1;
            }
        });
        let stats = live::live_loop(&mut l, &o, &q, sig, seid, media).unwrap();
        feeder.join().unwrap();
        l.pump(Duration::from_millis(50));
        let r = sim.report();
        // Locate the click in the decoded stream and map it to the packet that carried it.
        let idx = r.pcm.iter().position(|v| v.abs() > 4000).expect("click must be audible at the sink");
        let (mut cum, mut arrival) = (0usize, None);
        for (_, _, t, frames) in &r.arrivals {
            let n = *frames as usize * 64;
            if idx < cum + n {
                arrival = Some(*t);
                break;
            }
            cum += n;
        }
        let pushed = click.lock().unwrap().expect("click pushed");
        let lat = arrival.expect("click packet").saturating_duration_since(pushed);
        (r, stats, lat)
    }

    #[test]
    fn live_path_10ms_capture_has_low_latency_and_drops_nothing() {
        let (r, s, lat) = run_live(10, "live10");
        println!("live pipeline latency (capture buffer -> sink) with 10 ms WASAPI periods: {lat:?}; stats {s:?}");
        assert_eq!(s.dropped, 0, "10 ms bursts must never trigger the backlog guard (old code dropped audio here)");
        assert!(lat < Duration::from_millis(15), "pipeline latency {lat:?}");
        assert!(r.protocol_errors.is_empty(), "{:?}", r.protocol_errors);
        assert_eq!(r.seq_gaps + r.seq_backwards + r.ts_errors, 0);
        assert!(!r.paused_by_sink);
    }

    #[test]
    fn live_path_survives_bursty_30ms_capture() {
        let (r, s, lat) = run_live(30, "live30");
        println!("live pipeline latency with 30 ms bursts: {lat:?}; stats {s:?}");
        assert_eq!(s.dropped, 0);
        assert!(lat < Duration::from_millis(20), "pipeline latency {lat:?}");
        assert_eq!(r.seq_gaps + r.seq_backwards + r.ts_errors, 0);
        assert!(!r.paused_by_sink);
    }

    // ----- failure paths: must produce a clear error quickly, never hang -----

    #[test]
    fn page_timeout_is_reported_promptly() {
        let (mut h, _sim) = spawn(SimConfig { connect_status: 0x04, ..Default::default() });
        crate::init(&mut h).unwrap();
        let t = Instant::now();
        let e = a2dp::connect(&mut h, &SINK_ADDR, &[1u8; 16]).unwrap_err().to_string();
        assert!(e.contains("status 0x04"), "{e}");
        assert!(t.elapsed() < Duration::from_secs(3), "took {:?}", t.elapsed());
    }

    #[test]
    fn sink_hanging_up_during_pairing_is_reported() {
        let (mut h, _sim) = spawn(SimConfig { hangup_in_pairing: true, ..Default::default() });
        let t = Instant::now();
        let e = crate::pair(&mut h, &kf("hangup")).unwrap_err().to_string();
        assert!(e.contains("disconnected") && e.contains("0x13"), "{e}");
        assert!(t.elapsed() < Duration::from_secs(5), "took {:?}", t.elapsed());
    }

    #[test]
    fn rejected_configuration_is_an_error_not_a_hang() {
        let (mut h, _sim, hd, _k) = paired(SimConfig { reject_config: true, ..Default::default() }, "rejcfg");
        let mut l = Link::new(&mut h, hd).unwrap();
        let t = Instant::now();
        let e = a2dp::open_stream(&mut l, &opts(1, 3)).unwrap_err().to_string();
        assert!(e.contains("rejected"), "{e}");
        assert!(t.elapsed() < Duration::from_secs(3), "took {:?}", t.elapsed());
    }

    #[test]
    fn fragmentation_works_for_awkward_buffer_sizes() {
        for mtu in [17usize, 33, 100, 251] {
            let (mut h, sim, hd, _k) = paired(SimConfig { acl_mtu: mtu, ..Default::default() }, &format!("mtu{mtu}"));
            let mut l = Link::new(&mut h, hd).unwrap();
            a2dp::stream_tone(&mut l, &opts(1, 3)).unwrap();
            let r = sim.report();
            assert!(r.protocol_errors.is_empty(), "mtu {mtu}: {:?}", r.protocol_errors);
            assert_eq!(r.sdp_ok, 9, "mtu {mtu}");
            assert!(r.media_pkts > 200, "mtu {mtu}: only {} packets", r.media_pkts);
            let a = amp_at(&r.pcm[4000..r.pcm.len() - 4000], 440.0, 48000.0);
            assert!((a - 6000.0).abs() < 700.0, "mtu {mtu}: tone amplitude {a}");
        }
    }

    #[test]
    fn idle_system_audio_keeps_the_sink_from_pausing() {
        // Nothing playing in Windows => no capture data for seconds. Silence packets must keep the
        // stream alive past the sink's 500 ms deadline, and real audio must resume cleanly.
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "idle");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(2, 2);
        let (sig, seid, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let q: live::Q = Arc::new(Mutex::new(VecDeque::new()));
        let qf = q.clone();
        let feeder = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(1400));
            let mut g = qf.lock().unwrap();
            for i in 0..4800 {
                let v = ((i as f32 * 0.0575).sin() * 8000.0) as i16;
                g.push_back(v);
                g.push_back(v);
            }
        });
        let stats = live::live_loop(&mut l, &o, &q, sig, seid, media).unwrap();
        feeder.join().unwrap();
        l.pump(Duration::from_millis(50));
        let r = sim.report();
        assert!(!r.paused_by_sink, "silence keep-alive failed");
        assert!(stats.silent > 100, "expected silence packets, got {}", stats.silent);
        assert_eq!(r.seq_gaps + r.seq_backwards + r.ts_errors, 0);
        assert!(r.pcm.iter().any(|v| v.abs() > 4000), "resumed audio must reach the sink");
    }

    #[test]
    fn late_stale_disconnect_does_not_abort_pairing() {
        // Real failure: "Error: disconnected, reason 0x13" before any connection existed.
        let (_h, sim, hd, _k) = paired(SimConfig { late_stale_disconnect: true, inquiry_ms: 700, ..Default::default() }, "latestale");
        assert_eq!(hd, HANDLE);
        assert!(sim.report().encrypted);
    }

    #[test]
    fn connection_already_exists_recovers_by_resetting_and_retrying() {
        let key = [9u8; 16];
        let (mut h, sim) = spawn(SimConfig { exists_once: true, bonded_key: Some(key), ..Default::default() });
        crate::realtime_tuning();
        crate::init(&mut h).unwrap();
        let hd = a2dp::connect_with_retry(&mut h, &SINK_ADDR, &key, crate::init).expect("recovers from 0x0B");
        assert_eq!(hd, HANDLE);
        assert!(sim.report().encrypted);
    }

    #[test]
    fn malformed_events_and_fragments_never_crash_or_corrupt_the_session() {
        let (mut h, sim, hd, _k) = paired(SimConfig { garbage: true, ..Default::default() }, "garbage");
        let mut l = Link::new(&mut h, hd).unwrap();
        a2dp::stream_tone(&mut l, &opts(1, 3)).unwrap();
        let r = sim.report();
        clean(&r);
        assert_eq!(r.sdp_ok, 9);
        assert!(r.media_pkts > 200);
    }

    // ----- real-time delivery: the sink starves if audio arrives slower than it plays -----

    /// Drive `live_loop` with a continuous tone fed in `chunk_ms` bursts; returns (report, stats, samples fed).
    fn run_live_tone(cfg: SimConfig, fpp: usize, chunk_ms: u64, secs: f32, tag: &str) -> (Report, live::LiveStats, usize) {
        run_live_tone_codec(cfg, a2dp::Codec::Sbc, fpp, chunk_ms, secs, tag)
    }

    fn run_live_tone_codec(
        cfg: SimConfig,
        codec: a2dp::Codec,
        fpp: usize,
        chunk_ms: u64,
        secs: f32,
        tag: &str,
    ) -> (Report, live::LiveStats, usize) {
        let (mut h, sim, hd, _k) = paired(cfg, tag);
        let mut l = Link::new(&mut h, hd).unwrap();
        let mut o = opts(secs.ceil() as u32 + 1, fpp);
        o.codec = codec;
        // The AAC encoder must exist (and be warm) BEFORE the stream starts, or the sink sees dead air after Start.
        let enc = (codec == a2dp::Codec::Aac).then(|| crate::live_aac::make_encoder(&o).unwrap());
        let (sig, seid, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let q: live::Q = Arc::new(Mutex::new(VecDeque::new()));
        let qf = q.clone();
        let fed = Arc::new(std::sync::atomic::AtomicUsize::new(0));
        let ff = fed.clone();
        let feeder = std::thread::spawn(move || {
            let t0 = Instant::now();
            let per = (48 * chunk_ms) as usize;
            let mut n = 0u64;
            let mut phase = 0f32;
            while t0.elapsed() < Duration::from_secs_f32(secs) {
                let due = t0 + Duration::from_millis(chunk_ms * (n + 1));
                while Instant::now() < due {
                    std::thread::sleep(Duration::from_micros(300));
                }
                let mut g = qf.lock().unwrap();
                for _ in 0..per {
                    let v = ((phase * std::f32::consts::TAU).sin() * 6000.0) as i16;
                    phase = (phase + 440.0 / 48000.0).fract();
                    g.push_back(v);
                    g.push_back(v);
                }
                ff.fetch_add(per, std::sync::atomic::Ordering::Relaxed);
                n += 1;
            }
        });
        let stats = match codec {
            a2dp::Codec::Sbc => live::live_loop(&mut l, &o, &q, sig, seid, media).unwrap(),
            a2dp::Codec::Aac => crate::live_aac::live_loop_aac(&mut l, &o, &q, sig, seid, media, enc.unwrap()).unwrap(),
        };
        feeder.join().unwrap();
        l.pump(Duration::from_millis(50));
        (sim.report(), stats, fed.load(std::sync::atomic::Ordering::Relaxed))
    }

    #[test]
    fn live_adapts_packet_size_when_the_controller_cannot_sustain_the_packet_rate() {
        // 2-frame packets need 375 pkt/s but this controller completes 300/s: the old loop fell behind,
        // dropped audio and starved the sink (silence on real AirPods). The adaptive loop must keep up.
        let cfg = SimConfig { max_pkts_per_sec: 300, ..Default::default() };
        let (r, s, fed) = run_live_tone(cfg, 2, 10, 2.2, "adapt");
        println!(
            "adaptive: {s:?}, sink underruns {}, max buffer {:.0} ms, fed {fed} samples, decoded {}",
            r.underruns,
            r.max_buffer_ms,
            r.pcm.len()
        );
        assert_eq!(s.dropped, 0, "must not drop audio");
        assert!(s.max_frames_per_packet > 2, "must have grown packets to keep up, max was {}", s.max_frames_per_packet);
        assert_eq!(r.underruns, 0, "sink starved");
        assert!(r.pcm.len() + 4000 >= fed, "audio lost: fed {fed}, decoded {}", r.pcm.len());
        clean(&r);
        let mid = &r.pcm[10_000..(fed - 10_000).min(r.pcm.len())]; // the part where the tone is playing
        assert!((amp_at(mid, 440.0, 48000.0) - 6000.0).abs() < 700.0, "tone corrupted");
    }

    #[test]
    fn idle_silence_keepalive_runs_in_real_time_even_on_a_slow_controller() {
        let cfg = SimConfig { max_pkts_per_sec: 300, ..Default::default() };
        let (mut h, sim, hd, _k) = paired(cfg, "idleslow");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(2, 2);
        let (sig, seid, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let q: live::Q = Arc::new(Mutex::new(VecDeque::new()));
        live::live_loop(&mut l, &o, &q, sig, seid, media).unwrap();
        l.pump(Duration::from_millis(50));
        let r = sim.report();
        assert_eq!(r.underruns, 0, "silence must arrive at real-time speed");
        assert!(!r.paused_by_sink);
        // ~2 s of audio must have been delivered
        assert!(r.pcm.len() > 48000 * 3 / 2, "only {} samples", r.pcm.len());
    }

    #[test]
    fn the_simulated_controller_reproduces_the_hardware_limit_for_one_frame_packets() {
        // On hardware, 1-frame packets (750/s needed) ran at ~360/s: the tone took ~10 s instead of 6 s.
        // The simulator must show the same starvation, otherwise it cannot guard against it.
        let cfg = SimConfig { max_pkts_per_sec: 330, ..Default::default() };
        let (mut h, sim, hd, _k) = paired(cfg, "onefr");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(1, 1);
        let (_s, _e, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let mut st = Pace { seq: 0, ts: 0, next: Instant::now() + Duration::from_millis(5), phase: 0.0 };
        let t = Instant::now();
        a2dp::burst(&mut l, &mut st, media, o.cfg, 1, 440.0, 5000.0, 1.0, false).unwrap();
        assert!(t.elapsed() > Duration::from_millis(1800), "1 s of audio took only {:?}", t.elapsed());
        l.pump(Duration::from_millis(50));
        assert!(sim.report().underruns > 0, "sink must starve");
    }

    #[test]
    fn three_frame_packets_keep_real_time_on_a_330_pps_controller() {
        let cfg = SimConfig { max_pkts_per_sec: 330, ..Default::default() };
        let (mut h, sim, hd, _k) = paired(cfg, "threefr");
        let mut l = Link::new(&mut h, hd).unwrap();
        a2dp::stream_tone(&mut l, &opts(2, 3)).unwrap();
        let r = sim.report();
        assert_eq!(r.underruns, 0);
        clean(&r);
    }

    // ----- audio integrity: the pipeline must not damage the signal -----

    #[test]
    fn sine_snr_oracle_separates_clean_from_damaged_audio() {
        let fs = 48000.0;
        let clean_tone: Vec<i16> = (0..12000).map(|i| ((std::f64::consts::TAU * 1000.0 * i as f64 / fs).sin() * 9000.0) as i16).collect();
        let (snr, amp) = sine_snr(&clean_tone, 1000.0, fs);
        assert!(snr > 60.0 && (amp - 9000.0).abs() < 10.0, "clean: snr {snr} amp {amp}");
        let mut damaged = clean_tone.clone();
        for i in (500..12000).step_by(2400) {
            damaged[i] = 0; // a few single-sample glitches, like capture xruns
            damaged[i + 1] = -9000;
        }
        let (snr2, _) = sine_snr(&damaged, 1000.0, fs);
        assert!(snr2 < snr - 20.0, "glitches must reduce SNR a lot: {snr2} vs {snr}");
    }

    #[test]
    fn live_pipeline_preserves_a_pure_tone_even_with_adaptive_packets() {
        let cfg = SimConfig { max_pkts_per_sec: 300, ..Default::default() }; // forces packets to grow to 8 frames
        let (r, s, fed) = run_live_tone(cfg, 2, 10, 2.2, "snr");
        assert!(s.max_frames_per_packet > 2, "adaptation must have kicked in");
        let seg = &r.pcm[12_000..(fed - 12_000).min(r.pcm.len())];
        let (snr, amp) = sine_snr(seg, 440.0, 48000.0);
        println!("pipeline SNR with adaptive packets: {snr:.1} dB, amplitude {amp:.0}");
        assert!(snr > 40.0, "audio damaged by the pipeline: SNR {snr:.1} dB");
        assert!((amp - 6000.0).abs() < 150.0);
    }

    #[test]
    fn stream_can_be_reconfigured_mid_session_and_audio_follows() {
        let (mut h, sim, hd, _k) = paired(SimConfig { reject_reconfigure: false, ..Default::default() }, "reconf");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(1, 3);
        let (sig, seid, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let mut st = Pace { seq: 0, ts: 0, next: Instant::now() + Duration::from_millis(5), phase: 0.0 };
        a2dp::burst(&mut l, &mut st, media, o.cfg, 3, 440.0, 6000.0, 0.6, true).unwrap();
        // Switch to plain stereo / 16 blocks / bitpool 35 without reopening anything.
        let c2 = sbc::Config { rate: sbc::Rate::Hz48000, mode: sbc::Mode::Stereo, blocks: 16, subbands: 8, bitpool: 35, snr: false };
        l.avdtp(sig, 11, proto::AVDTP_SUSPEND, &[seid << 2]).unwrap();
        l.avdtp(sig, 12, proto::AVDTP_RECONFIGURE, &proto::reconfigure_sbc(seid, &c2)).unwrap();
        l.avdtp(sig, 13, proto::AVDTP_START, &[seid << 2]).unwrap();
        st.next = Instant::now() + Duration::from_millis(5);
        a2dp::burst(&mut l, &mut st, media, c2, 2, 660.0, 6000.0, 0.6, true).unwrap();
        l.pump(Duration::from_millis(50));
        let r = sim.report();
        clean(&r);
        assert_eq!(r.reconfigs, 1);
        assert_eq!(r.configured.map(|c| c[3]), Some(35), "sink must hold the new configuration");
        let n1 = (0.6f64 / (192.0 / 48000.0)) as usize * 192; // samples in segment one
        let a = &r.pcm[3000..n1 - 3000];
        let b = &r.pcm[n1 + 3000..r.pcm.len() - 3000];
        assert!((amp_at(a, 440.0, 48000.0) - 6000.0).abs() < 700.0, "segment 1 wrong");
        assert!((amp_at(b, 660.0, 48000.0) - 6000.0).abs() < 700.0, "segment 2 wrong");
    }

    #[test]
    fn sink_rejects_media_whose_frame_header_disagrees_with_the_negotiated_config() {
        // Strict-decoder model: negotiate joint/8-blocks but send stereo/16-block frames.
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "hdrmismatch");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(1, 3);
        let (_s, _e, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let wrong = sbc::Config { rate: sbc::Rate::Hz48000, mode: sbc::Mode::Stereo, blocks: 16, subbands: 8, bitpool: 35, snr: false };
        let mut st = Pace { seq: 0, ts: 0, next: Instant::now() + Duration::from_millis(5), phase: 0.0 };
        a2dp::burst(&mut l, &mut st, media, wrong, 2, 440.0, 5000.0, 0.1, true).unwrap();
        l.pump(Duration::from_millis(50));
        assert!(
            sim.report().protocol_errors.iter().any(|e| e.contains("disagrees with the negotiated config")),
            "mismatch must be flagged"
        );
    }

    #[test]
    fn reconfigure_is_rejected_like_the_real_airpods_do() {
        let (mut h, _sim, hd, _k) = paired(SimConfig::default(), "reconfrej");
        let mut l = Link::new(&mut h, hd).unwrap();
        let (sig, seid, _m) = a2dp::open_stream(&mut l, &opts(1, 3)).unwrap();
        let c2 = sbc::Config { rate: sbc::Rate::Hz48000, mode: sbc::Mode::Stereo, blocks: 16, subbands: 8, bitpool: 35, snr: false };
        l.avdtp(sig, 11, proto::AVDTP_SUSPEND, &[seid << 2]).unwrap();
        let e = l.avdtp(sig, 12, proto::AVDTP_RECONFIGURE, &proto::reconfigure_sbc(seid, &c2)).unwrap_err().to_string();
        assert!(e.contains("rejected") && e.contains("81"), "{e}");
    }

    #[test]
    fn stream_restart_changes_configuration_without_reconfigure() {
        // Suspend, Close, drop the media channel, Set Configuration, Open, reopen, Start: what AirPods allow.
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "restart");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o = opts(1, 3);
        let (sig, seid, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let mut st = Pace { seq: 0, ts: 0, next: Instant::now() + Duration::from_millis(5), phase: 0.0 };
        a2dp::burst(&mut l, &mut st, media, o.cfg, 3, 440.0, 6000.0, 0.6, true).unwrap();
        let c2 = sbc::Config { rate: sbc::Rate::Hz48000, mode: sbc::Mode::Stereo, blocks: 16, subbands: 8, bitpool: 35, snr: false };
        let media2 = a2dp::restart_with(&mut l, sig, seid, media, &c2).unwrap();
        st.next = Instant::now() + Duration::from_millis(5);
        a2dp::burst(&mut l, &mut st, media2, c2, 2, 660.0, 6000.0, 0.6, true).unwrap();
        l.pump(Duration::from_millis(50));
        let r = sim.report();
        assert!(r.protocol_errors.is_empty(), "{:?}", r.protocol_errors);
        assert_eq!(r.configured.map(|c| (c[0], c[1], c[3])), Some((0x12, 0x15, 35)), "new config must be active");
        assert_eq!(r.seq_backwards, 0);
        assert!(!r.paused_by_sink);
        let n1 = (0.6f64 / (192.0 / 48000.0)) as usize * 192;
        assert!((amp_at(&r.pcm[3000..n1 - 3000], 440.0, 48000.0) - 6000.0).abs() < 700.0);
        assert!((amp_at(&r.pcm[n1 + 3000..r.pcm.len() - 3000], 660.0, 48000.0) - 6000.0).abs() < 700.0);
    }

    // ----- optional AAC mode: the real Windows AAC encoder/decoder against the strict simulated sink -----

    #[test]
    fn aac_stream_plays_a_tone_through_the_real_windows_codec() {
        let (r, s, fed) = run_live_tone_codec(SimConfig::default(), a2dp::Codec::Aac, 2, 10, 2.4, "aac1");
        println!("AAC live: {s:?}, aac frames {}, decoded {} samples (fed {fed}), underruns {}", r.aac_frames, r.pcm.len(), r.underruns);
        assert!(r.protocol_errors.is_empty(), "{:?}", r.protocol_errors);
        assert!(r.configured_aac.is_some(), "an AAC configuration must have been negotiated");
        assert!(r.configured.is_none(), "the SBC endpoint must not have been configured");
        assert!(r.aac_frames > 90, "only {} AAC frames reached the sink", r.aac_frames);
        assert_eq!(r.underruns, 0, "sink starved");
        assert_eq!(r.seq_gaps + r.seq_backwards + r.ts_errors, 0);
        assert!(!r.paused_by_sink);
        let mid = &r.pcm[20_000..(fed - 40_000).min(r.pcm.len())];
        let (snr, amp) = sine_snr(mid, 440.0, 48000.0);
        println!("AAC tone: amplitude {amp:.0}, SNR {snr:.1} dB");
        assert!((amp - 6000.0).abs() < 600.0 && snr > 25.0, "AAC audio damaged: amp {amp:.0}, SNR {snr:.1} dB");
    }

    #[test]
    fn aac_idle_silence_keeps_the_sink_alive() {
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "aacidle");
        let mut l = Link::new(&mut h, hd).unwrap();
        let mut o = opts(2, 2);
        o.codec = a2dp::Codec::Aac;
        let enc = crate::live_aac::make_encoder(&o).unwrap();
        let (sig, seid, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let q: live::Q = Arc::new(Mutex::new(VecDeque::new()));
        crate::live_aac::live_loop_aac(&mut l, &o, &q, sig, seid, media, enc).unwrap();
        l.pump(Duration::from_millis(50));
        let r = sim.report();
        assert!(!r.paused_by_sink, "silence keep-alive failed for AAC");
        assert_eq!(r.underruns, 0);
        assert!(r.aac_frames > 60, "only {} frames in 2 s of idle", r.aac_frames);
        assert!(r.protocol_errors.is_empty(), "{:?}", r.protocol_errors);
    }

    #[test]
    fn aac_negotiates_44100_hz_too() {
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "aac441");
        let mut l = Link::new(&mut h, hd).unwrap();
        let mut o = opts(1, 2);
        o.codec = a2dp::Codec::Aac;
        o.cfg.rate = sbc::Rate::Hz44100;
        let enc = crate::live_aac::make_encoder(&o).unwrap();
        let (sig, seid, media) = a2dp::open_stream(&mut l, &o).unwrap();
        let q: live::Q = Arc::new(Mutex::new(VecDeque::new()));
        crate::live_aac::live_loop_aac(&mut l, &o, &q, sig, seid, media, enc).unwrap();
        l.pump(Duration::from_millis(50));
        let r = sim.report();
        assert!(r.protocol_errors.is_empty(), "{:?}", r.protocol_errors);
        let c = r.configured_aac.expect("AAC configured");
        assert_eq!((c[1], c[2] & 0xF0), (0x01, 0x00), "44.1 kHz is signalled in byte 1");
        assert!(r.aac_frames > 20);
    }

    #[test]
    fn requesting_aac_from_a_sink_without_it_is_a_clear_error() {
        let (mut h, _sim, hd, _k) = paired(SimConfig { sbc_only: true, ..Default::default() }, "noaac");
        let mut l = Link::new(&mut h, hd).unwrap();
        let mut o = opts(1, 2);
        o.codec = a2dp::Codec::Aac;
        let e = a2dp::open_stream(&mut l, &o).unwrap_err().to_string();
        assert!(e.contains("offers no Aac endpoint") && e.contains("SEID 1"), "{e}");
    }

    #[test]
    fn aac_endpoint_discovery_skips_vendor_endpoints_and_picks_the_right_one() {
        let (mut h, _sim, hd, _k) = paired(SimConfig::default(), "seps");
        let mut l = Link::new(&mut h, hd).unwrap();
        let mut o = opts(1, 2);
        o.codec = a2dp::Codec::Aac;
        let (_sig, seid, _media) = a2dp::open_stream(&mut l, &o).unwrap();
        assert_eq!(seid, 2, "the AAC endpoint, not the SBC (1) or the vendor one (3)");
    }

    #[test]
    fn codec_can_be_switched_live_sbc_to_aac_and_back() {
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "switch");
        let mut l = Link::new(&mut h, hd).unwrap();
        let o_sbc = opts(1, 2);
        let mut o_aac = opts(1, 2);
        o_aac.codec = a2dp::Codec::Aac;
        let (sig, seid, media) = a2dp::open_stream(&mut l, &o_sbc).unwrap();
        let q: live::Q = Arc::new(Mutex::new(VecDeque::new()));
        let qf = q.clone();
        let feeder = std::thread::spawn(move || {
            let t0 = Instant::now();
            let mut phase = 0f32;
            let mut n = 0u64;
            while t0.elapsed() < Duration::from_millis(4600) {
                let due = t0 + Duration::from_millis(10 * (n + 1));
                while Instant::now() < due {
                    std::thread::sleep(Duration::from_micros(300));
                }
                let mut g = qf.lock().unwrap();
                for _ in 0..480 {
                    let v = ((phase * std::f32::consts::TAU).sin() * 6000.0) as i16;
                    phase = (phase + 440.0 / 48000.0).fract();
                    g.push_back(v);
                    g.push_back(v);
                }
                n += 1;
            }
        });
        let ex = live::LoopEx { secs: Some(1), start_gaps: 0, suspend_at_end: false };
        live::live_loop_ex(&mut l, &o_sbc, &q, sig, seid, media, ex).unwrap();
        let mut enc = None;
        let (seid2, media2) = a2dp::switch_codec(&mut l, sig, seid, media, &o_aac, || {
            enc = Some(crate::live_aac::make_encoder(&o_aac)?); // after the Suspend, before anything restarts
            Ok(())
        })
        .unwrap();
        assert_eq!(seid2, 2);
        crate::live_aac::live_loop_aac_ex(&mut l, &o_aac, &q, sig, seid2, media2, enc.take().unwrap(), ex).unwrap();
        let (seid3, media3) = a2dp::switch_codec(&mut l, sig, seid2, media2, &o_sbc, || Ok(())).unwrap();
        assert_eq!(seid3, 1);
        // The final phase Suspends: a stream left Streaming with no media is (rightly) paused by a strict sink.
        live::live_loop_ex(&mut l, &o_sbc, &q, sig, seid3, media3, live::LoopEx { suspend_at_end: true, ..ex }).unwrap();
        feeder.join().unwrap();
        l.pump(Duration::from_millis(50));
        let r = sim.report();
        assert!(r.protocol_errors.is_empty(), "{:?}", r.protocol_errors);
        assert!(r.frames_decoded > 100, "SBC frames: {}", r.frames_decoded);
        assert!(r.aac_frames > 30, "AAC frames: {}", r.aac_frames);
        assert!(r.configured.is_some() && r.configured_aac.is_some(), "both codecs must have been configured at some point");
        assert!(!r.paused_by_sink, "sink paused: {:?}", r.pause_info);
        assert_eq!(r.seq_backwards, 0, "a codec switch is a new stream: no stale sequence numbers");
    }

    #[test]
    fn codec_sweep_alternates_sbc_and_aac_and_marks_each_phase_audibly() {
        let (mut h, sim, hd, _k) = paired(SimConfig::default(), "sweep");
        let mut l = Link::new(&mut h, hd).unwrap();
        let q: live::Q = Arc::new(Mutex::new(VecDeque::new()));
        let qf = q.clone();
        let feeder = std::thread::spawn(move || {
            let t0 = Instant::now();
            let (mut phase, mut n) = (0f32, 0u64);
            while t0.elapsed() < Duration::from_millis(9600) {
                let due = t0 + Duration::from_millis(10 * (n + 1));
                while Instant::now() < due {
                    std::thread::sleep(Duration::from_micros(300));
                }
                let mut g = qf.lock().unwrap();
                for _ in 0..480 {
                    let v = ((phase * std::f32::consts::TAU).sin() * 9000.0) as i16;
                    phase = (phase + 440.0 / 48000.0).fract();
                    g.push_back(v);
                    g.push_back(v);
                }
                n += 1;
            }
        });
        let mut o = opts(1, 2);
        o.cfg.blocks = 16;
        crate::live_aac::codec_sweep_q(&mut l, &o, &q, 2).unwrap(); // 4 phases of 2 s: room for every marker
        feeder.join().unwrap();
        l.pump(Duration::from_millis(50));
        let r = sim.report();
        assert!(r.protocol_errors.is_empty(), "{:?}", r.protocol_errors);
        assert!(!r.paused_by_sink, "sink paused: {:?}", r.pause_info);
        assert!(r.frames_decoded > 100 && r.aac_frames > 60, "SBC frames {}, AAC frames {}", r.frames_decoded, r.aac_frames);
        // Gap markers: 1 + 2 + 3 quarter-second silences must be visible in what the sink decoded
        // (a silent run of at least ~150 ms in the middle of a loud tone).
        let win = 480usize; // 10 ms
        let silent: Vec<bool> = r.pcm.chunks(win).map(|c| c.iter().map(|v| v.unsigned_abs() as u32).max().unwrap_or(0) < 400).collect();
        let (mut runs, mut cur) = (0, 0);
        for (i, &s) in silent.iter().enumerate() {
            if s {
                cur += 1;
            }
            if (!s || i + 1 == silent.len()) && cur > 0 {
                if cur >= 15 && i > 20 && i + 20 < silent.len() {
                    runs += 1;
                }
                cur = 0;
            }
        }
        println!("silent runs >=150 ms inside the tone: {runs} (expected 6 markers)");
        assert!(runs >= 5, "expected 6 audible gap markers (1+2+3), found {runs}");
    }

    // ----- the tray's background session manager -----

    use crate::daemon::{self, Cmd, Config, Status, Timing};
    use std::sync::mpsc;
    use std::sync::{Arc, Mutex};

    /// Run the daemon against a simulated sink until `done(statuses)` holds, then quit it.
    fn run_daemon(
        cfg: SimConfig,
        tag: &str,
        key: Option<[u8; 16]>,
        mut script: impl FnMut(&mpsc::Sender<Cmd>, &[Status]) -> bool,
    ) -> (Vec<Status>, Report) {
        // The daemon uses process-wide state (capture device, AACP mode), so daemon tests take turns.
        static DAEMON_LOCK: Mutex<()> = Mutex::new(());
        let _turn = DAEMON_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        crate::aacp::set_current(None);
        let (mut h, sim) = spawn(cfg);
        let keyfile = kf(tag);
        let _ = std::fs::remove_file(&keyfile);
        if let Some(k) = key {
            crate::save_key(&keyfile, &SINK_ADDR, &k);
        }
        let statuses = Arc::new(Mutex::new(Vec::<Status>::new()));
        let (tx, rx) = mpsc::channel();
        let st = statuses.clone();
        let worker = std::thread::spawn(move || {
            let t = Timing {
                page_every: Duration::from_millis(300),
                retry_after: Duration::from_millis(200),
                aacp_delay: Duration::from_millis(800),
            };
            let c = Config { capture_device: String::new(), ..Config::default() };
            let report = move |s: Status| {
                let mut g = st.lock().unwrap();
                if g.last() != Some(&s) {
                    g.push(s);
                }
            };
            daemon::serve(&mut h, &keyfile, &rx, &report, &t, &c).unwrap();
        });
        let end = Instant::now() + Duration::from_secs(40);
        loop {
            let snapshot = statuses.lock().unwrap().clone();
            if script(&tx, &snapshot) || Instant::now() > end {
                break;
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        let _ = tx.send(Cmd::Quit);
        worker.join().unwrap();
        let s = statuses.lock().unwrap().clone();
        (s, sim.report())
    }

    fn streamed_then_waiting_again(s: &[Status]) -> bool {
        s.iter().position(|x| *x == Status::Streaming).map(|i| s[i..].contains(&Status::Waiting)).unwrap_or(false)
    }

    #[test]
    fn daemon_accepts_the_airpods_reconnect_streams_and_goes_back_to_waiting() {
        crate::realtime_tuning();
        let key = [9u8; 16];
        let cfg =
            SimConfig { aacp: false, request_on_scan: true, bonded_key: Some(key), drop_after_start_ms: Some(1500), ..Default::default() };
        let (s, r) = run_daemon(cfg, "daemon_incoming", Some(key), |_, s| streamed_then_waiting_again(s));
        assert_eq!(s[..3], [Status::Waiting, Status::Connecting, Status::Streaming], "statuses: {s:?}");
        assert!(streamed_then_waiting_again(&s), "after the AirPods leave the daemon must wait again: {s:?}");
        assert!(r.protocol_errors.is_empty(), "protocol errors: {:?}", r.protocol_errors);
        assert!(r.encrypted && r.first_media_at.is_some());
        assert!(r.accepted_request, "the incoming Connection_Request must be accepted, not left to our own page");
    }

    #[test]
    fn daemon_pages_the_airpods_when_they_do_not_connect_themselves() {
        crate::realtime_tuning();
        let key = [4u8; 16];
        let cfg = SimConfig { aacp: false, bonded_key: Some(key), drop_after_start_ms: Some(1000), ..Default::default() };
        let (s, r) = run_daemon(cfg, "daemon_page", Some(key), |_, s| s.contains(&Status::Streaming));
        assert!(s.contains(&Status::Streaming), "statuses: {s:?}");
        assert!(r.protocol_errors.is_empty(), "protocol errors: {:?}", r.protocol_errors);
    }

    #[test]
    fn daemon_without_a_pairing_asks_for_one_and_pairs_on_request() {
        crate::realtime_tuning();
        let cfg = SimConfig { aacp: false, drop_after_start_ms: Some(1000), ..Default::default() };
        let (s, r) = run_daemon(cfg, "daemon_pair", None, |tx, s| {
            if s == [Status::NeedsPairing] {
                tx.send(Cmd::Pair).unwrap();
            }
            s.contains(&Status::Streaming)
        });
        assert_eq!(s[..2], [Status::NeedsPairing, Status::Pairing], "statuses: {s:?}");
        assert!(s.contains(&Status::Waiting) && s.contains(&Status::Streaming), "statuses: {s:?}");
        assert!(r.link_key_issued.is_some(), "pairing must have bonded");
        assert!(r.protocol_errors.is_empty(), "protocol errors: {:?}", r.protocol_errors);
    }

    #[test]
    fn daemon_hangs_up_a_failed_session_instead_of_leaving_the_link_dangling() {
        crate::realtime_tuning();
        let key = [5u8; 16];
        let cfg = SimConfig { aacp: false, reject_config: true, bonded_key: Some(key), ..Default::default() };
        let (s, r) = run_daemon(cfg, "daemon_hangup", Some(key), |_, s| streamed_then_waiting_again(s));
        assert!(streamed_then_waiting_again(&s), "statuses: {s:?}");
        assert_eq!(r.hangups, 1, "a session that failed while the link was up must end with an HCI Disconnect");
    }

    #[test]
    fn daemon_survives_a_controller_error_and_keeps_listening() {
        crate::realtime_tuning();
        // Wrong key on the sink side: authentication fails, the daemon reports it and retries instead of dying.
        let cfg = SimConfig { aacp: false, bonded_key: Some([1u8; 16]), ..Default::default() };
        let (s, _) =
            run_daemon(cfg, "daemon_badkey", Some([2u8; 16]), |_, s| s.iter().filter(|x| matches!(x, Status::Error(_))).count() >= 1);
        assert!(s.iter().any(|x| matches!(x, Status::Error(_))), "statuses: {s:?}");
    }

    #[test]
    fn daemon_reads_and_sets_noise_control_over_the_airpods_control_channel() {
        use crate::aacp::{self, NoiseMode};
        crate::realtime_tuning();
        let key = [6u8; 16];
        let cfg = SimConfig { aacp: true, aacp_mode: 2, bonded_key: Some(key), drop_after_start_ms: Some(6000), ..Default::default() };
        let mut asked = false;
        let mut asked_off = false;
        let mut pods = aacp::Pods::default();
        let (s, r) = run_daemon(cfg, "daemon_anc", Some(key), |_, s| {
            if s.contains(&Status::Streaming) && aacp::current() == Some(NoiseMode::Anc) && !asked {
                asked = true;
                aacp::request(NoiseMode::Transparency);
            }
            if asked && aacp::current() == Some(NoiseMode::Transparency) && !asked_off {
                asked_off = true;
                aacp::request(NoiseMode::Off);
            }
            if aacp::current() == Some(NoiseMode::Off) {
                pods = aacp::pods();
            }
            asked_off && aacp::current() == Some(NoiseMode::Off)
        });
        assert!(s.contains(&Status::Streaming), "statuses: {s:?}");
        assert!(r.aacp_handshake && r.aacp_features && r.aacp_notify, "handshake, features and notification subscription required");
        assert_eq!(r.aacp_sets, vec![3, 1], "transparency, then off");
        assert!(r.aacp_allow_off, "Off is ignored by the AirPods unless it is allowed first");
        assert_eq!(pods.left.map(|b| (b.percent, b.charging)), Some((95, false)));
        assert_eq!(pods.right.map(|b| (b.percent, b.charging)), Some((100, true)));
        assert_eq!(pods.case.map(|b| b.percent), Some(50));
        assert_eq!(pods.ears_text(), "Ears: one in ear");
        assert!(r.protocol_errors.is_empty(), "protocol errors: {:?}", r.protocol_errors);
    }
}

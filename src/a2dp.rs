//! Link layer glue: ACL fragmentation/credits, L2CAP channels, AVDTP, and a paced A2DP stream.

use crate::hci::{Addr, Hci, fmt_addr};
use crate::{proto, sbc};
use anyhow::{Result, anyhow, bail};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

pub struct Link<'a> {
    pub h: &'a mut Hci,
    pub handle: u16,
    acl_mtu: usize,
    credits: usize,
    credits_total: usize,
    rx: Vec<u8>,
    rx_need: usize,
    inbox: VecDeque<(u16, Vec<u8>)>,
    sig_id: u8,
    pub disconnected: bool,
    pub sig_dcid: u16,
    sdp_remote: u16,
    avrcp_dcid: u16,
    pub completed: u64,
    pub flushed: u64,
    volume_acked: bool,
    pub sink_delay_ms: Option<f32>,
    avdtp_txn: u8,
    aacp_dcid: u16,
    aacp_due: Option<Instant>,
}

const SDP_CID: u16 = 0x50;
const AVRCP_CID: u16 = 0x42;
const AACP_CID: u16 = 0x43;

impl<'a> Link<'a> {
    pub fn new(h: &'a mut Hci, handle: u16) -> Result<Self> {
        let b = h.cmd(0x1005, &[])?; // Read_Buffer_Size
        if b.len() < 6 {
            bail!("short Read_Buffer_Size reply {b:02x?}");
        }
        let acl_mtu = u16::from_le_bytes([b[1], b[2]]) as usize;
        let credits = u16::from_le_bytes([b[4], b[5]]) as usize;
        println!("controller ACL buffers: {credits} x {acl_mtu} bytes");
        h.cmd(0x080D, &[handle.to_le_bytes()[0], handle.to_le_bytes()[1], 0, 0])?; // Write_Link_Policy_Settings: forbid sniff/role switch
        Ok(Self {
            h,
            handle,
            acl_mtu,
            credits,
            credits_total: credits,
            rx: vec![],
            rx_need: 0,
            inbox: VecDeque::new(),
            sig_id: 1,
            disconnected: false,
            sig_dcid: 0,
            sdp_remote: 0,
            avrcp_dcid: 0,
            completed: 0,
            flushed: 0,
            volume_acked: false,
            sink_delay_ms: None,
            avdtp_txn: 8,
            aacp_dcid: 0,
            aacp_due: None,
        })
    }

    /// Next AVDTP transaction label (0..15), for signals we do not number by hand.
    pub fn next_txn(&mut self) -> u8 {
        self.avdtp_txn = (self.avdtp_txn + 1) & 0x0F;
        self.avdtp_txn
    }

    /// Tear down an L2CAP channel (`dcid` = the sink's cid, `scid` = ours).
    pub fn close_channel(&mut self, dcid: u16, scid: u16) -> Result<()> {
        self.sig_send(|id| {
            let mut r = vec![0x06, id, 4, 0];
            r.extend_from_slice(&dcid.to_le_bytes());
            r.extend_from_slice(&scid.to_le_bytes());
            r
        })?;
        let end = Instant::now() + Duration::from_secs(5);
        while Instant::now() < end {
            if let Some((cid, d)) = self.recv(Duration::from_millis(50)) {
                if cid == 1 && d.first() == Some(&0x07) {
                    return Ok(());
                }
                self.other(cid, &d)?;
            }
        }
        bail!("L2CAP disconnect timed out")
    }

    /// Set the automatic flush timeout (ms; 0 = never flush) for this link.
    pub fn set_flush_ms(&mut self, ms: f32) -> Result<()> {
        let slots = (ms / 0.625) as u16;
        let mut p = self.handle.to_le_bytes().to_vec();
        p.extend_from_slice(&slots.to_le_bytes());
        self.h.cmd(0x0C28, &p)?;
        Ok(())
    }

    /// Is a controller ACL buffer free right now?
    pub fn has_credit(&self) -> bool {
        self.credits > 0
    }

    /// ACL buffers currently in flight (sent, not yet reported complete).
    pub fn outstanding(&self) -> usize {
        self.credits_total.saturating_sub(self.credits)
    }

    /// One-line radio health snapshot: RSSI, link quality, failed contacts, TX power.
    pub fn link_stats(&mut self) -> String {
        let h = self.handle.to_le_bytes();
        let rssi = self.h.cmd(0x1405, &h).map(|r| r[3] as i8);
        let lq = self.h.cmd(0x1403, &h).map(|r| r[3]);
        let fc = self.h.cmd(0x1401, &h).map(|r| u16::from_le_bytes([r[3], r[4]]));
        let tx = self.h.cmd(0x0C2D, &[h[0], h[1], 0]).map(|r| r[3] as i8);
        let txmax = self.h.cmd(0x0C2D, &[h[0], h[1], 1]).map(|r| r[3] as i8);
        // Adaptive frequency hopping: how many of the 79 channels the controller currently uses.
        let afh = self.h.cmd(0x1406, &h).map(|r| {
            let used: u32 = r[4..14].iter().map(|b| b.count_ones()).sum::<u32>().min(79);
            format!("mode {} channels_in_use {used}/79", r[3])
        });
        format!("rssi {rssi:?} lq {lq:?} failed_contacts {fc:?} tx_power {tx:?} (max {txmax:?}) dBm, AFH {afh:?}")
    }

    /// Process pending HCI events and ACL data without blocking longer than `wait`.
    pub fn pump(&mut self, wait: Duration) {
        while let Some(e) = self.h.next_event(Duration::ZERO) {
            self.on_event(&e);
        }
        if wait > Duration::ZERO {
            if let Some(e) = self.h.next_event(wait) {
                self.on_event(&e);
            }
        }
        while let Ok(p) = self.h.acl_rx.try_recv() {
            self.on_acl(&p);
        }
    }

    fn on_event(&mut self, e: &[u8]) {
        match e[0] {
            0x13 => {
                let n = e[2] as usize;
                for i in 0..n {
                    let o = 3 + n * 2 + i * 2;
                    if let Some(c) = e.get(o..o + 2) {
                        self.completed += u16::from_le_bytes([c[0], c[1]]) as u64;
                        self.credits += u16::from_le_bytes([c[0], c[1]]) as usize;
                    }
                }
            }
            0x11 => self.flushed += 1,
            0x14 => println!(
                "  [HCI] Mode Change: status {:#04x} mode {} (0=active 2=sniff) interval {} slots",
                e[2],
                e[5],
                u16::from_le_bytes([e[6], e[7]])
            ),
            0x12 => println!("  [HCI] Role Change: status {:#04x} new role {} (0=central)", e[2], e[9]),
            0x10 => println!("  [HCI] HARDWARE ERROR {:02x?}", e),
            0x1A => println!("  [HCI] Sniff Subrating {:02x?}", e),
            0x05 => {
                self.disconnected = true;
                println!("link disconnected, reason {:#04x}", e[5]);
            }
            0x1B | 0x0E | 0x0F | 0x08 | 0x06 | 0x18 => {}
            c => println!("  [HCI] event {c:#04x} {:02x?}", &e[..e.len().min(16)]),
        }
    }

    fn on_acl(&mut self, p: &[u8]) {
        if crate::hci::trace() {
            eprintln!("acl rx {:02x?}", &p[..p.len().min(48)]);
        }
        if p.len() < 4 {
            return;
        }
        let pb = (u16::from_le_bytes([p[0], p[1]]) >> 12) & 3;
        let data = &p[4..];
        if pb == 1 {
            // Continuation fragment: meaningless without a start fragment in progress.
            if self.rx.is_empty() {
                return;
            }
            self.rx.extend_from_slice(data);
        } else {
            self.rx.clear();
            self.rx.extend_from_slice(data);
            self.rx_need = 0;
        }
        if self.rx_need == 0 && self.rx.len() >= 2 {
            self.rx_need = u16::from_le_bytes([self.rx[0], self.rx[1]]) as usize + 4;
        }
        if self.rx_need >= 4 && self.rx.len() >= self.rx_need {
            let cid = u16::from_le_bytes([self.rx[2], self.rx[3]]);
            self.inbox.push_back((cid, self.rx[4..self.rx_need].to_vec())); // drop any bytes past the PDU
            self.rx.clear();
            self.rx_need = 0;
        }
    }
    /// Next L2CAP PDU, waiting up to `timeout`.
    pub fn recv(&mut self, timeout: Duration) -> Option<(u16, Vec<u8>)> {
        let end = Instant::now() + timeout;
        loop {
            if let Some(x) = self.inbox.pop_front() {
                return Some(x);
            }
            let left = end.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return None;
            }
            self.pump(left.min(Duration::from_millis(20)));
        }
    }

    /// Send one L2CAP PDU on `cid` (auto-flushable first fragment if `flushable`).
    pub fn send(&mut self, cid: u16, payload: &[u8], flushable: bool) -> Result<()> {
        let pdu = proto::l2cap(cid, payload);
        let mut first = true;
        for chunk in pdu.chunks(self.acl_mtu) {
            let stall = Instant::now();
            while self.credits == 0 {
                self.pump(Duration::from_millis(5));
                if self.disconnected {
                    bail!("disconnected");
                }
                if stall.elapsed() > Duration::from_millis(1500) {
                    let h = self.handle.to_le_bytes();
                    let rssi = self.h.cmd(0x1405, &h).map(|r| r[3] as i8);
                    let lq = self.h.cmd(0x1403, &h).map(|r| r[3]);
                    bail!("controller returned no ACL credits for 1.5 s (radio not delivering). rssi {rssi:?} link_quality {lq:?}");
                }
            }
            let pb = if !first {
                1
            } else if flushable {
                2
            } else {
                0
            };
            if crate::hci::trace() {
                eprintln!("acl tx pb={pb} {:02x?}", &chunk[..chunk.len().min(48)]);
            }
            self.h.send_acl(&proto::acl(self.handle, pb, chunk))?;
            self.credits -= 1;
            first = false;
        }
        Ok(())
    }

    fn sig_send(&mut self, build: impl FnOnce(u8) -> Vec<u8>) -> Result<u8> {
        let id = self.sig_id;
        self.sig_id = self.sig_id.wrapping_add(1).max(1);
        let p = build(id);
        self.send(1, &p, false)?;
        Ok(id)
    }

    /// Open an L2CAP channel; returns (remote cid, remote mtu).
    pub fn open_channel(&mut self, psm: u16, scid: u16) -> Result<(u16, usize)> {
        self.sig_send(|id| proto::l2cap_connect_req(id, psm, scid))?;
        let mut dcid = 0u16;
        let (mut got_rsp, mut sent_cfg) = (false, false);
        let (mut our_cfg_ok, mut their_cfg_done) = (false, false);
        let mut rmtu = 672usize;
        let end = Instant::now() + Duration::from_secs(5);
        while Instant::now() < end && !(our_cfg_ok && their_cfg_done) {
            let Some((cid, d)) = self.recv(Duration::from_millis(100)) else { continue };
            if cid != 1 {
                self.other(cid, &d)?; // never drop traffic we are not waiting for
                continue;
            }
            if d.len() < 4 {
                continue;
            }
            let (code, id) = (d[0], d[1]);
            let body = &d[4..];
            match code {
                0x03 if body.len() >= 8 && u16::from_le_bytes([body[2], body[3]]) == scid => {
                    let result = u16::from_le_bytes([body[4], body[5]]);
                    if result == 1 {
                        continue;
                    }
                    if result != 0 {
                        bail!("L2CAP connect refused, result {result}");
                    }
                    dcid = u16::from_le_bytes([body[0], body[1]]);
                    got_rsp = true;
                }
                0x04 if body.len() >= 2 && u16::from_le_bytes([body[0], body[1]]) != SDP_CID => {
                    // Remote config request: parse MTU, accept.
                    let mut o = 4;
                    while o + 2 <= body.len() {
                        let (t, l) = (body[o], body[o + 1] as usize);
                        if t == 1 && l == 2 && o + 4 <= body.len() {
                            rmtu = u16::from_le_bytes([body[o + 2], body[o + 3]]) as usize;
                        }
                        o += 2 + l;
                    }
                    let rsp = proto::l2cap_config_rsp(id, dcid, 0);
                    self.send(1, &rsp, false)?;
                    their_cfg_done = true;
                }
                0x05 if body.len() >= 6 => {
                    let result = u16::from_le_bytes([body[4], body[5]]);
                    if result != 0 {
                        bail!("L2CAP config rejected, result {result}");
                    }
                    our_cfg_ok = true;
                }
                0x02 | 0x04 | 0x06 | 0x0A => self.sig_misc(&d)?,
                _ => {}
            }
            if got_rsp && !sent_cfg {
                sent_cfg = true;
                self.sig_send(|id| proto::l2cap_config_req(id, dcid, Some(895), None))?;
            }
        }
        if !(our_cfg_ok && their_cfg_done) {
            bail!("L2CAP channel setup timed out (psm {psm:#x})");
        }
        Ok((dcid, rmtu))
    }

    /// Answer and log everything the remote sent while we were busy streaming.
    pub fn service(&mut self) -> Result<()> {
        while let Some((cid, d)) = self.inbox.pop_front() {
            if cid == 1 {
                println!("  [rx L2CAP sig] code {:#04x} {:02x?}", d.first().copied().unwrap_or(0), &d[..d.len().min(16)]);
            }
            self.other(cid, &d)?;
        }
        if self.aacp_due.is_some_and(|t| Instant::now() >= t) {
            self.aacp_due = None;
            if let Err(e) = self.aacp_open() {
                println!("AirPods control channel unavailable (noise control disabled): {e}");
            }
        }
        // Only a link that has the control channel may consume a pending request.
        if self.aacp_dcid != 0 {
            if let Some(m) = crate::aacp::take_request() {
                println!("  [AACP] setting noise control: {}", m.label());
                if m == crate::aacp::NoiseMode::Off {
                    // The AirPods ignore Off until it is allowed.
                    self.send(self.aacp_dcid, &crate::aacp::ALLOW_OFF, false)?;
                }
                self.send(self.aacp_dcid, &crate::aacp::set_noise(m), false)?;
            }
        }
        Ok(())
    }

    /// Open the control channel `delay` from now, from `service()`. Opening it before the audio channels makes the
    /// AirPods abandon the A2DP setup (hardware-verified), so it must wait until audio is flowing.
    pub fn aacp_after(&mut self, delay: Duration) {
        self.aacp_due = Some(Instant::now() + delay);
    }

    /// Open the AirPods' control channel (PSM 0x1001): handshake, enable features, subscribe to notifications.
    /// Best effort: audio does not depend on it, so the caller may ignore an error.
    pub fn aacp_open(&mut self) -> Result<()> {
        use crate::aacp;
        let (dcid, _) = self.open_channel(aacp::PSM, AACP_CID)?;
        self.aacp_dcid = dcid;
        self.send(dcid, &aacp::HANDSHAKE, false)?;
        self.pump(Duration::from_millis(30));
        self.send(dcid, &aacp::FEATURES, false)?;
        self.send(dcid, &aacp::NOTIFY, false)?;
        println!("  [AACP] control channel open");
        Ok(()) // the initial mode report arrives later and is picked up by service()
    }

    fn aacp_rx(&mut self, d: &[u8]) -> Result<()> {
        if let Some(m) = crate::aacp::parse_noise(d) {
            println!("  [AACP] noise control is now {}", m.label());
            crate::aacp::set_current(Some(m));
        } else if let Some(p) = crate::aacp::parse_battery(d) {
            crate::aacp::update_battery(p);
            println!("  [AACP] {}", p.battery_text());
        } else if let Some(e) = crate::aacp::parse_ears(d) {
            crate::aacp::update_ears(e);
            println!("  [AACP] ear detection {e:?}");
        } else {
            println!("  [AACP rx] {:02x?}", &d[..d.len().min(24)]);
        }
        Ok(())
    }

    /// Anything we are not explicitly waiting for: answer it so the sink never stalls on us.
    /// Used by every receive loop (channel setup, AVDTP waits, AVRCP waits, streaming).
    fn other(&mut self, cid: u16, d: &[u8]) -> Result<()> {
        match cid {
            1 => self.sig_misc(d),
            SDP_CID => self.sdp_handle(d),
            AVRCP_CID => self.avrcp_rx(d),
            AACP_CID => self.aacp_rx(d),
            0x40 if d.len() >= 2 && d[0] & 3 == 0 => self.avdtp_cmd(d),
            0x40 => Ok(()), // a late response nobody is waiting for
            _ => {
                println!("  [rx unexpected cid {cid:#06x}] {:02x?}", &d[..d.len().min(16)]);
                Ok(())
            }
        }
    }

    /// A command the sink sent us on the AVDTP signalling channel.
    fn avdtp_cmd(&mut self, d: &[u8]) -> Result<()> {
        let (t, sig) = (d[0] >> 4, d[1] & 0x3F);
        if sig == 0x0D && d.len() >= 5 {
            let delay = u16::from_be_bytes([d[3], d[4]]) as f32 / 10.0;
            println!("  [AVDTP] sink Delay Report: {delay:.1} ms");
            self.sink_delay_ms = Some(delay);
            self.send(self.sig_dcid, &proto::avdtp(t, 2, sig, &[]), false)
        } else {
            println!("  [AVDTP] sink command {sig:#04x} {:02x?} -> rejected", &d[..d.len().min(12)]);
            self.send(self.sig_dcid, &proto::avdtp(t, 3, sig, &[0x01]), false)
        }
    }

    /// Answer remote-initiated L2CAP signalling: SDP channel (we serve it), info, disconnects.
    fn sig_misc(&mut self, d: &[u8]) -> Result<()> {
        if d.len() < 4 {
            return Ok(());
        }
        let (code, id, body) = (d[0], d[1], &d[4..]);
        match code {
            0x02 if body.len() >= 4 => {
                let (psm, rcid) = (u16::from_le_bytes([body[0], body[1]]), u16::from_le_bytes([body[2], body[3]]));
                let mut r = vec![0x03, id, 8, 0];
                if psm == 1 {
                    r.extend_from_slice(&SDP_CID.to_le_bytes());
                    r.extend_from_slice(&rcid.to_le_bytes());
                    r.extend_from_slice(&[0, 0, 0, 0]);
                    self.send(1, &r, false)?;
                    self.sdp_remote = rcid;
                    println!("  [SDP] remote opened SDP channel, serving A2DP source record");
                    self.sig_send(|i| proto::l2cap_config_req(i, rcid, Some(672), None))?;
                } else {
                    r.extend_from_slice(&[0, 0]);
                    r.extend_from_slice(&rcid.to_le_bytes());
                    r.extend_from_slice(&[2, 0, 0, 0]);
                    self.send(1, &r, false)?;
                    println!("  [L2CAP] refused remote channel to PSM {psm:#06x}");
                }
            }
            0x04 if body.len() >= 4 && u16::from_le_bytes([body[0], body[1]]) == SDP_CID => {
                let rsp = proto::l2cap_config_rsp(id, self.sdp_remote, 0);
                self.send(1, &rsp, false)?;
            }
            0x06 if body.len() >= 4 => {
                // Disconnect request: dcid (ours), scid (theirs)
                let mut r = vec![0x07, id, 4, 0];
                r.extend_from_slice(&body[..4]);
                self.send(1, &r, false)?;
                if u16::from_le_bytes([body[0], body[1]]) == SDP_CID {
                    self.sdp_remote = 0;
                }
            }
            0x0A if body.len() >= 2 => {
                let mut r = vec![0x0B, id, 4, 0];
                r.extend_from_slice(&body[..2]);
                r.extend_from_slice(&[1, 0]);
                self.send(1, &r, false)?;
            }
            _ => {}
        }
        Ok(())
    }

    /// Open AVRCP and set the sink's absolute volume (0..=0x7F). Sinks keep a per-source volume;
    /// a fresh bond can sit at zero until the source sets it, as Windows does on connect.
    pub fn avrcp_set_volume(&mut self, vol: u8) -> Result<()> {
        let (dcid, _) = self.open_channel(0x17, AVRCP_CID)?;
        self.avrcp_dcid = dcid;
        // AVCTP(txn 1, command, PID 0x110E) + AV/C CONTROL, panel, VENDOR_DEPENDENT, BT SIG 00 19 58,
        // PDU 0x50 SetAbsoluteVolume, single packet, 1 param byte.
        let cmd = [0x10, 0x11, 0x0E, 0x00, 0x48, 0x00, 0x00, 0x19, 0x58, 0x50, 0x00, 0x00, 0x01, vol & 0x7F];
        self.send(dcid, &cmd, false)?;
        println!("  [AVRCP] SetAbsoluteVolume {} / 127 sent", vol & 0x7F);
        let end = Instant::now() + Duration::from_millis(1500);
        while Instant::now() < end && !self.volume_acked {
            if let Some((cid, d)) = self.recv(Duration::from_millis(50)) {
                self.other(cid, &d)?;
            }
        }
        Ok(())
    }

    /// AVRCP/AVCTP traffic: log responses; answer any command from the sink with NOT_IMPLEMENTED.
    fn avrcp_rx(&mut self, d: &[u8]) -> Result<()> {
        if d.len() < 4 {
            return Ok(());
        }
        let is_rsp = d[0] & 2 != 0;
        let ctype = d[3] & 0x0F;
        println!("  [AVRCP rx] {} ctype {ctype:#x} {:02x?}", if is_rsp { "response" } else { "command" }, &d[..d.len().min(16)]);
        if is_rsp && d.len() >= 14 && d[9] == 0x50 && ctype == 0x09 {
            self.volume_acked = true;
        }
        if !is_rsp && self.avrcp_dcid != 0 {
            let mut r = d.to_vec();
            r[0] |= 0x02; // response
            let vendor = d.len() >= 13 && d[5] == 0x00 && d[6..9] == [0x00, 0x19, 0x58];
            let pdu = if vendor { d[9] } else { 0 };
            let params = if d.len() > 13 { &d[13..] } else { &[][..] };
            let reply: Option<(u8, Vec<u8>)> = match (pdu, params.first().copied()) {
                (0x10, Some(0x03)) => Some((0x0C, vec![0x03, 2, 0x01, 0x02])), // events: playback status, track changed
                (0x10, Some(0x02)) => Some((0x0C, vec![0x02, 1, 0x00, 0x19, 0x58])),
                (0x30, _) => Some((0x0C, vec![0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01])), // GetPlayStatus: playing
                (0x31, Some(0x01)) => Some((0x0F, vec![0x01, 0x01])),                                  // INTERIM: PLAYING
                (0x31, Some(0x02)) => Some((0x0F, vec![0x02, 0, 0, 0, 0, 0, 0, 0, 0])),                // INTERIM: track 0
                _ => None,
            };
            if d.len() >= 8 && d[5] == 0x7C {
                // Pass-through (play/pause/...): acknowledge it, and say so loudly.
                println!(
                    "  [AVRCP] sink pressed key {:#04x} ({}), acknowledging",
                    d[6] & 0x7F,
                    match d[6] & 0x7F {
                        0x44 => "PLAY",
                        0x45 => "STOP",
                        0x46 => "PAUSE",
                        0x4B => "NEXT",
                        0x4C => "PREV",
                        _ => "other",
                    }
                );
                r[3] = 0x09;
                return self.send(self.avrcp_dcid, &r, false);
            }
            match reply {
                Some((ct, p)) => {
                    r.truncate(9);
                    r[3] = ct;
                    r.push(pdu);
                    r.push(0);
                    r.extend_from_slice(&(p.len() as u16).to_be_bytes());
                    r.extend_from_slice(&p);
                    println!("  [AVRCP] answered pdu {pdu:#04x} as player (ctype {ct:#x})");
                }
                None => r[3] = 0x08, // NOT_IMPLEMENTED
            }
            self.send(self.avrcp_dcid, &r, false)?;
        }
        Ok(())
    }

    /// Minimal SDP server: one A2DP Source record (0x110A), AVDTP 1.3, A2DP 1.3.
    fn sdp_handle(&mut self, d: &[u8]) -> Result<()> {
        if d.len() < 5 || self.sdp_remote == 0 {
            return Ok(());
        }
        let (pdu, txn) = (d[0], [d[1], d[2]]);
        println!("  [SDP] req {:02x?}", &d[..d.len().min(48)]);
        // Which service is being asked for? UUIDs sit in the pattern sequence right after the header.
        // Ours only if the search pattern names A2DP Source (0x110A) or the Public Browse Root (0x1002).
        let wants_ours =
            pdu == 0x04 || d[5..d.len().min(5 + 2 + 40)].windows(3).any(|w| w == [0x19, 0x11, 0x0A] || w == [0x19, 0x10, 0x02]);
        let rec = if wants_ours { sdp_record() } else { Vec::new() };
        let mut params = Vec::new();
        let rsp_pdu = match pdu {
            0x02 => {
                if wants_ours {
                    params.extend_from_slice(&[0, 1, 0, 1, 0, 1, 0, 1, 0]); // total, current, handle 0x00010001, no continuation
                } else {
                    params.extend_from_slice(&[0, 0, 0, 0, 0]); // no matching records
                }
                0x03
            }
            0x04 => {
                params.extend_from_slice(&(rec.len() as u16).to_be_bytes());
                params.extend_from_slice(&rec);
                params.push(0);
                0x05
            }
            0x06 => {
                let lists = sdp_seq(&rec);
                params.extend_from_slice(&(lists.len() as u16).to_be_bytes());
                params.extend_from_slice(&lists);
                params.push(0);
                0x07
            }
            _ => return Ok(()),
        };
        println!("  [SDP] request pdu {pdu:#04x} -> response {rsp_pdu:#04x} ({} B)", params.len());
        let mut r = vec![rsp_pdu, txn[0], txn[1]];
        r.extend_from_slice(&(params.len() as u16).to_be_bytes());
        r.extend_from_slice(&params);
        self.send(self.sdp_remote, &r, false)
    }
    /// AVDTP request/response on the signalling channel. Returns accept params.
    pub fn avdtp(&mut self, cid: u16, txn: u8, signal: u8, params: &[u8]) -> Result<Vec<u8>> {
        self.send(cid, &proto::avdtp(txn, 0, signal, params), false)?;
        let end = Instant::now() + Duration::from_secs(5);
        while Instant::now() < end {
            let Some((c, d)) = self.recv(Duration::from_millis(100)) else { continue };
            if c != 0x40 || d.len() < 2 || d[0] & 3 == 0 {
                self.other(c, &d)?; // commands, SDP, AVRCP, signalling: answer, then keep waiting
                continue;
            }
            let (t, mt, sig) = (d[0] >> 4, d[0] & 3, d[1]);
            if t == txn && sig == signal {
                if mt == 2 {
                    return Ok(d[2..].to_vec());
                }
                bail!("AVDTP signal {signal:#04x} rejected: {:02x?}", &d[2..]);
            }
        }
        Err(anyhow!("AVDTP signal {signal:#04x} timed out"))
    }
}

/// Initial AVRCP absolute volume (0..=127) sent to the sink before Start.
pub static AVRCP_VOLUME: std::sync::atomic::AtomicU8 = std::sync::atomic::AtomicU8::new(0x30);

/// Connect (with stored link key), authenticate, encrypt. Returns ACL handle.
pub fn connect(h: &mut Hci, addr: &Addr, key: &[u8; 16]) -> Result<u16> {
    // Adopt a link the AirPods opened themselves (they auto-reconnect to the last host).
    let mut incoming = None;
    let settle = Instant::now() + Duration::from_millis(400);
    while Instant::now() < settle {
        if let Some(e) = h.next_event(Duration::from_millis(50)) {
            if e[0] == 0x03 && e[2] == 0 && e[5..11] == addr[..] {
                incoming = Some(u16::from_le_bytes([e[3], e[4]]));
                break;
            }
        }
    }
    if let Some(hd) = incoming {
        println!("AirPods connected to us (handle {hd:#05x}), authenticating");
        return finish_link(h, addr, key, hd);
    }
    let mut p = addr.to_vec();
    p.extend_from_slice(&0xCC18u16.to_le_bytes());
    p.extend_from_slice(&[1, 0, 0, 0, 0]); // psrm R1, reserved, clock offset invalid, NO role switch
    h.cmd(0x0405, &p)?;
    let mut handle = None;
    let end = Instant::now() + Duration::from_secs(20);
    // (outgoing path)
    while Instant::now() < end {
        let Some(e) = h.next_event(Duration::from_millis(100)) else { continue };
        match e[0] {
            0x03 => {
                if e[2] != 0 {
                    bail!("connect failed, status {:#04x} (are the AirPods out of the case / not on another device?)", e[2]);
                }
                let hd = u16::from_le_bytes([e[3], e[4]]);
                handle = Some(hd);
                h.cmd(0x0411, &hd.to_le_bytes())?;
            }
            0x17 => {
                let mut r = addr.to_vec();
                r.extend_from_slice(key);
                h.cmd(0x040B, &r)?;
            }
            0x06 => {
                if e[2] != 0 {
                    bail!("authentication failed {:#04x} (re-pair)", e[2]);
                }
                let mut r = handle.unwrap().to_le_bytes().to_vec();
                r.push(1);
                h.cmd(0x0413, &r)?;
            }
            0x08 if e[2] == 0 => return Ok(handle.unwrap()),
            0x05 if handle == Some(u16::from_le_bytes([e[3], e[4]])) => bail!("disconnected during setup, reason {:#04x}", e[5]),
            _ => {}
        }
    }
    bail!("timeout connecting to {}", fmt_addr(addr))
}

/// Which audio codec to stream with. SBC is mandatory for every sink; AAC is optional and, on AirPods,
/// reportedly uses a smaller playback buffer (at the price of ~70 ms of encoder delay on our side).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Codec {
    Sbc,
    Aac,
}

pub struct StreamOpts {
    pub codec: Codec,
    pub cfg: sbc::Config,
    /// AAC bit rate in bits/s (the Windows encoder supports 96/128/160/192 kbit/s).
    pub aac_bitrate: u32,
    pub frames_per_packet: usize,
    pub flush_ms: f32,
    pub seconds: u32,
}

/// Does the capability list contain service category `cat`?
fn caps_has(caps: &[u8], cat: u8) -> bool {
    let mut i = 0;
    while i + 2 <= caps.len() {
        if caps[i] == cat {
            return true;
        }
        i += 2 + caps[i + 1] as usize;
    }
    false
}

/// The Media Codec capability item (category 0x07): (codec type, codec information element).
fn media_codec_item(caps: &[u8]) -> Option<(u8, &[u8])> {
    let mut i = 0;
    while i + 2 <= caps.len() {
        let (cat, len) = (caps[i], caps[i + 1] as usize);
        if cat == 0x07 && len >= 2 && i + 2 + len <= caps.len() {
            return Some((caps[i + 3], &caps[i + 4..i + 2 + len]));
        }
        i += 2 + len;
    }
    None
}

fn parse_sbc_caps(caps: &[u8]) -> Option<[u8; 4]> {
    match media_codec_item(caps) {
        Some((proto::CODEC_SBC, info)) if info.len() >= 4 => info[..4].try_into().ok(),
        _ => None,
    }
}

/// A stream endpoint of the sink with its advertised capabilities.
#[derive(Clone, Debug)]
pub struct Sep {
    pub seid: u8,
    pub caps: Vec<u8>,
    /// Codec type from the Media Codec item (0x00 SBC, 0x02 AAC, 0xFF vendor), if present.
    pub codec: Option<u8>,
}

/// Discover every free audio sink endpoint and fetch its capabilities.
pub fn discover_seps(l: &mut Link, sig_cid: u16) -> Result<Vec<Sep>> {
    let t = l.next_txn();
    let d = l.avdtp(sig_cid, t, proto::AVDTP_DISCOVER, &[])?;
    let mut seps = Vec::new();
    for e in d.chunks(2).filter(|e| e.len() == 2 && (e[0] & 2) == 0 && (e[1] >> 4) == 0 && (e[1] >> 3) & 1 == 1) {
        let seid = e[0] >> 2;
        let t = l.next_txn();
        match l.avdtp(sig_cid, t, proto::AVDTP_GET_CAPABILITIES, &[seid << 2]) {
            Ok(caps) => {
                let codec = media_codec_item(&caps).map(|(c, _)| c);
                println!("sink endpoint SEID {seid}: codec {codec:02x?}, capabilities {caps:02x?}");
                seps.push(Sep { seid, caps, codec });
            }
            Err(err) => println!("sink endpoint SEID {seid}: capabilities unavailable ({err})"),
        }
    }
    if seps.is_empty() {
        bail!("no free audio sink endpoint: {d:02x?}");
    }
    Ok(seps)
}

/// Build the Set_Configuration parameters for `sep` and the requested codec, validating against its capabilities.
fn configuration_for(sep: &Sep, o: &StreamOpts) -> Result<Vec<u8>> {
    let seid = sep.seid;
    match o.codec {
        Codec::Sbc => {
            let sc = parse_sbc_caps(&sep.caps).ok_or_else(|| anyhow!("endpoint {seid} has no SBC: {:02x?}", sep.caps))?;
            println!("sink SBC caps: {sc:02x?}");
            let mut cfgp = proto::set_sbc_configuration(seid, 1, &o.cfg);
            // AVDTP 1.3 Delay Reporting: if the sink offers it, enable it so it tells us its playout buffer depth.
            if caps_has(&sep.caps, 0x08) {
                cfgp.extend_from_slice(&[0x08, 0x00]);
            }
            // Capability sanity: refuse configs the sink did not advertise.
            let (freq, chm, blk, sb) = (cfgp[8] & 0xF0, cfgp[8] & 0x0F, cfgp[9] & 0xF0, cfgp[9] & 0x0C);
            if sc[0] & freq == 0 || sc[0] & chm == 0 || sc[1] & blk == 0 || sc[1] & sb == 0 {
                bail!("sink does not support requested SBC config; caps {sc:02x?}");
            }
            let bp = o.cfg.bitpool.clamp(sc[2], sc[3]);
            if bp != o.cfg.bitpool {
                bail!("bitpool {} outside sink range {}..={}", o.cfg.bitpool, sc[2], sc[3]);
            }
            Ok(cfgp)
        }
        Codec::Aac => {
            let (_, info) = media_codec_item(&sep.caps).ok_or_else(|| anyhow!("endpoint {seid} has no codec item"))?;
            let c = proto::parse_aac_caps(info).ok_or_else(|| anyhow!("endpoint {seid}: unparsable AAC capabilities {info:02x?}"))?;
            println!("sink AAC caps: {c:?}");
            // AirPods set several object-type bits; MPEG-2 AAC-LC is the mandatory one, MPEG-4 LC the fallback.
            let obj = if c.object_types & proto::AAC_MPEG2_LC != 0 {
                proto::AAC_MPEG2_LC
            } else if c.object_types & proto::AAC_MPEG4_LC != 0 {
                proto::AAC_MPEG4_LC
            } else {
                bail!("sink AAC has no LC object type (bits {:#04x})", c.object_types);
            };
            let rate = match o.cfg.rate {
                sbc::Rate::Hz44100 => 44100,
                sbc::Rate::Hz48000 => 48000,
            };
            let (f1, f2) = proto::aac_freq_bits(rate).unwrap();
            if c.freq_byte1 & f1 == 0 && c.freq_byte2 & f2 == 0 {
                bail!("sink AAC does not support {rate} Hz (caps {:#04x}/{:#04x})", c.freq_byte1, c.freq_byte2);
            }
            if c.channels & 0x04 == 0 {
                bail!("sink AAC does not support stereo");
            }
            if c.bitrate != 0 && o.aac_bitrate > c.bitrate {
                bail!("requested AAC bit rate {} exceeds the sink's {}", o.aac_bitrate, c.bitrate);
            }
            proto::set_aac_configuration(seid, 1, obj, rate, false, o.aac_bitrate).ok_or_else(|| anyhow!("bad AAC configuration"))
        }
    }
}

/// Configure and start one endpoint: Set_Configuration, Open, media channel, volume, Start.
/// Returns the media channel id. Everything that can block happens BEFORE Start: sinks expect media immediately after it.
fn start_endpoint(l: &mut Link, sig_cid: u16, sep: &Sep, o: &StreamOpts, with_volume: bool) -> Result<u16> {
    let seid = sep.seid;
    let cfgp = configuration_for(sep, o)?;
    let t = l.next_txn();
    l.avdtp(sig_cid, t, proto::AVDTP_SET_CONFIGURATION, &cfgp)?;
    let t = l.next_txn();
    l.avdtp(sig_cid, t, proto::AVDTP_OPEN, &[seid << 2])?;
    let (media_cid, mtu) = l.open_channel(proto::AVDTP_PSM, 0x41)?;
    println!("media channel open (remote mtu {mtu})");
    if with_volume {
        // Windows' volume slider does not reach the AirPods (loopback is pre-volume): set their volume here.
        let env = std::env::var("AIRLOW_VOLUME").ok().and_then(|v| v.trim().parse::<u8>().ok());
        let vol = env.unwrap_or_else(|| AVRCP_VOLUME.load(std::sync::atomic::Ordering::Relaxed)).min(0x7F);
        if let Err(e) = l.avrcp_set_volume(vol) {
            println!("  [AVRCP] volume not set: {e}");
        }
    }
    let t = l.next_txn();
    l.avdtp(sig_cid, t, proto::AVDTP_START, &[seid << 2])?;
    Ok(media_cid)
}

fn pick_sep<'a>(seps: &'a [Sep], codec: Codec) -> Result<&'a Sep> {
    let want = match codec {
        Codec::Sbc => proto::CODEC_SBC,
        Codec::Aac => proto::CODEC_AAC,
    };
    seps.iter().find(|s| s.codec == Some(want)).ok_or_else(|| {
        let have: Vec<String> = seps.iter().map(|s| format!("SEID {} codec {:02x?}", s.seid, s.codec)).collect();
        anyhow!("the sink offers no {codec:?} endpoint (it offers: {})", have.join("; "))
    })
}

/// Full A2DP bring-up. Returns (signalling cid, sink seid, media cid) with the stream started.
pub fn open_stream(l: &mut Link, o: &StreamOpts) -> Result<(u16, u8, u16)> {
    // Auto-flush: drop audio the radio could not deliver in time instead of retransmitting.
    if o.flush_ms > 0.0 {
        l.set_flush_ms(o.flush_ms)?;
        println!("ACL auto-flush timeout = {} ms", o.flush_ms);
    }
    let (sig_cid, _) = l.open_channel(proto::AVDTP_PSM, 0x40)?;
    l.sig_dcid = sig_cid;
    println!("AVDTP signalling channel open");
    let seps = discover_seps(l, sig_cid)?;
    let sep = pick_sep(&seps, o.codec)?.clone();
    println!("using {:?} endpoint SEID {}", o.codec, sep.seid);
    let media_cid = start_endpoint(l, sig_cid, &sep, o, true)?;
    Ok((sig_cid, sep.seid, media_cid))
}

/// Switch a live stream to the other codec: Suspend, Close, drop the media channel, then configure the endpoint
/// that speaks `o.codec`. Returns (seid, media cid).
///
/// `prepare` runs right after the Suspend and before anything is started again: build the new codec's encoder
/// there. Slow first-time setup must never happen while a stream is Streaming with no media flowing (the sink
/// would pause) or after Start (dead air).
pub fn switch_codec(
    l: &mut Link,
    sig_cid: u16,
    old_seid: u8,
    old_media: u16,
    o: &StreamOpts,
    prepare: impl FnOnce() -> Result<()>,
) -> Result<(u8, u16)> {
    let t = l.next_txn();
    l.avdtp(sig_cid, t, proto::AVDTP_SUSPEND, &[old_seid << 2])?;
    prepare()?;
    let t = l.next_txn();
    l.avdtp(sig_cid, t, proto::AVDTP_CLOSE, &[old_seid << 2])?;
    l.close_channel(old_media, 0x41)?;
    let seps = discover_seps(l, sig_cid)?;
    let sep = pick_sep(&seps, o.codec)?.clone();
    let media = start_endpoint(l, sig_cid, &sep, o, false)?;
    Ok((sep.seid, media))
}
/// A2DP bring-up + paced test tone.
pub fn stream_tone(l: &mut Link, o: &StreamOpts) -> Result<()> {
    let (sig_cid, seid, media_cid) = open_stream(l, o)?;
    println!("streaming {} s test tone...", o.seconds);

    let mut enc = sbc::Encoder::new(o.cfg);
    let codesize = enc.codesize();
    let spf = enc.samples_per_frame();
    let hz = match o.cfg.rate {
        sbc::Rate::Hz44100 => 44100.0f32,
        sbc::Rate::Hz48000 => 48000.0,
    };
    let pkt_samples = spf * o.frames_per_packet;
    let pkt_dur = Duration::from_secs_f64(pkt_samples as f64 / hz as f64);
    let flen = enc.frame_len();
    println!(
        "frame {} B / {:.2} ms, {} frame(s)/packet = {:.2} ms, ~{:.0} kbps",
        flen,
        enc.frame_duration_us() / 1000.0,
        o.frames_per_packet,
        pkt_dur.as_secs_f64() * 1000.0,
        (flen * o.frames_per_packet * 8) as f64 / pkt_dur.as_secs_f64() / 1000.0
    );

    let total_pkts = (o.seconds as f64 / pkt_dur.as_secs_f64()) as u64;
    let mut phase = 0.0f32;
    let mut pcm = vec![0u8; codesize];
    let mut fr = vec![0u8; 1024];
    let t0 = Instant::now() + Duration::from_millis(20);
    let (mut seq, mut ts) = (0u16, 0u32);
    for k in 0..total_pkts {
        let due = t0 + pkt_dur * k as u32;
        loop {
            let now = Instant::now();
            if now >= due {
                break;
            }
            if due - now > Duration::from_micros(2500) {
                l.pump(Duration::from_millis(1));
            } else {
                std::hint::spin_loop();
            }
        }
        let mut payload = Vec::with_capacity(flen * o.frames_per_packet);
        for _ in 0..o.frames_per_packet {
            for s in 0..spf {
                let v = ((phase * std::f32::consts::TAU).sin() * 6000.0) as i16;
                phase = (phase + 440.0 / hz).fract();
                pcm[s * 4..s * 4 + 2].copy_from_slice(&v.to_le_bytes());
                pcm[s * 4 + 2..s * 4 + 4].copy_from_slice(&v.to_le_bytes());
            }
            let n = enc.encode_frame(&pcm, &mut fr);
            payload.extend_from_slice(&fr[..n]);
        }
        l.service()?;
        let pkt = proto::rtp_sbc(seq, ts, 0xA1D10, o.frames_per_packet as u8, &payload);
        l.send(media_cid, &pkt, true)?;
        seq = seq.wrapping_add(1);
        ts = ts.wrapping_add(pkt_samples as u32);
        if l.disconnected {
            bail!("link dropped mid-stream");
        }
    }
    let _ = l.avdtp(sig_cid, 6, proto::AVDTP_SUSPEND, &[seid << 2]);
    println!("done");
    Ok(())
}

pub fn finish_link(h: &mut Hci, addr: &Addr, key: &[u8; 16], hd: u16) -> Result<u16> {
    h.cmd(0x0411, &hd.to_le_bytes())?;
    let end = Instant::now() + Duration::from_secs(15);
    while Instant::now() < end {
        let Some(e) = h.next_event(Duration::from_millis(100)) else { continue };
        match e[0] {
            0x17 => {
                let mut r = addr.to_vec();
                r.extend_from_slice(key);
                h.cmd(0x040B, &r)?;
            }
            0x06 if e[2] == 0 => {
                let mut r = hd.to_le_bytes().to_vec();
                r.push(1);
                h.cmd(0x0413, &r)?;
            }
            0x06 => bail!("authentication failed {:#04x} (re-pair)", e[2]),
            0x08 if e[2] == 0 => return Ok(hd),
            0x05 => bail!("disconnected during setup, reason {:#04x}", e[5]),
            _ => {}
        }
    }
    bail!("timeout authenticating")
}
/// RTP and pacing state that must stay continuous across bursts: a sink treats a sequence
/// number or timestamp that jumps backwards as stale/duplicate media and drops it.
pub(crate) struct Pace {
    pub(crate) seq: u16,
    pub(crate) ts: u32,
    pub(crate) next: Instant,
    pub(crate) phase: f32,
}

/// One paced tone burst, continuing the stream described by `st`. Returns packets sent.
pub(crate) fn burst(
    l: &mut Link,
    st: &mut Pace,
    media_cid: u16,
    cfg: sbc::Config,
    fpp: usize,
    freq: f32,
    amp: f32,
    secs: f32,
    flushable: bool,
) -> Result<u64> {
    let mut enc = sbc::Encoder::new(cfg);
    let (spf, hz) = (enc.samples_per_frame(), if matches!(cfg.rate, sbc::Rate::Hz48000) { 48000.0f32 } else { 44100.0 });
    let pkt_samples = spf * fpp;
    let pkt_dur = Duration::from_secs_f64(pkt_samples as f64 / hz as f64);
    let (mut pcm, mut fr) = (vec![0u8; enc.codesize()], vec![0u8; 1024]);
    let total = (secs as f64 / pkt_dur.as_secs_f64()) as u64;
    let t_start = Instant::now();
    let (mut next_tick, mut next_stats) = (t_start + Duration::from_millis(500), t_start + Duration::from_secs(1));
    let (mut c_prev, mut f_prev) = (l.completed, l.flushed);
    for _ in 0..total {
        let due = st.next;
        st.next += pkt_dur;
        while Instant::now() < due {
            if due - Instant::now() > Duration::from_micros(2500) {
                l.pump(Duration::from_millis(1));
            } else {
                std::hint::spin_loop();
            }
        }
        l.service()?;
        let mut payload = Vec::new();
        for _ in 0..fpp {
            for s in 0..spf {
                let v = ((st.phase * std::f32::consts::TAU).sin() * amp) as i16;
                st.phase = (st.phase + freq / hz).fract();
                pcm[s * 4..s * 4 + 2].copy_from_slice(&v.to_le_bytes());
                pcm[s * 4 + 2..s * 4 + 4].copy_from_slice(&v.to_le_bytes());
            }
            let n = enc.encode_frame(&pcm, &mut fr);
            payload.extend_from_slice(&fr[..n]);
        }
        l.send(media_cid, &proto::rtp_sbc(st.seq, st.ts, 0xA1D10, fpp as u8, &payload), flushable)?;
        st.seq = st.seq.wrapping_add(1);
        st.ts = st.ts.wrapping_add(pkt_samples as u32);
        let now = Instant::now();
        if now >= next_tick {
            println!(
                "    t={:.1}s  +{} completed, +{} flushed, {} in flight",
                now.duration_since(t_start).as_secs_f32(),
                l.completed - c_prev,
                l.flushed - f_prev,
                l.outstanding()
            );
            (c_prev, f_prev) = (l.completed, l.flushed);
            next_tick = now + Duration::from_millis(500);
        }
        if now >= next_stats {
            println!("    link: {}", l.link_stats());
            next_stats = now + Duration::from_secs(1);
        }
        if l.disconnected {
            bail!("link dropped mid-stream");
        }
    }
    Ok(total)
}

/// Several packet strategies in one connection, each a distinct pitch.
pub fn experiments(l: &mut Link, o: &StreamOpts) -> Result<()> {
    let (sig_cid, seid, media_cid) = open_stream(l, o)?;
    // (label, freq, amp, flush_ms, flushable, frames_per_packet)
    let runs = [
        ("A: 440 Hz  flush 40ms, auto-flushable, 3 frames/pkt", 440.0, 6000.0, 40.0, true, 3),
        ("B: 660 Hz  no flush, non-flushable,    3 frames/pkt", 660.0, 6000.0, 0.0, false, 3),
        ("C: 880 Hz  no flush, non-flushable,    1 frame/pkt ", 880.0, 6000.0, 0.0, false, 1),
        ("D: 1100 Hz no flush, non-flushable,    2 frames/pkt, louder", 1100.0, 12000.0, 0.0, false, 2),
    ];
    let per = (o.seconds as f32 / runs.len() as f32).max(0.25); // seconds per variant
    let mut st = Pace { seq: 0, ts: 0, next: Instant::now() + Duration::from_millis(10), phase: 0.0 };
    for (label, freq, amp, flush, flushable, fpp) in runs {
        let slots = (flush / 0.625) as u16;
        let mut p = l.handle.to_le_bytes().to_vec();
        p.extend_from_slice(&slots.to_le_bytes());
        l.h.cmd(0x0C28, &p)?;
        println!("== {label}");
        let (c0, f0) = (l.completed, l.flushed);
        let n = burst(l, &mut st, media_cid, o.cfg, fpp, freq, amp, per, flushable)?;
        l.pump(Duration::ZERO);
        println!(
            "   sent {n} packets; controller: {} completed, {} flushed (flushed = dropped undelivered)",
            l.completed - c0,
            l.flushed - f0
        );
    }
    let _ = l.avdtp(sig_cid, 6, proto::AVDTP_SUSPEND, &[seid << 2]);
    println!("done");
    Ok(())
}
fn sdp_seq(content: &[u8]) -> Vec<u8> {
    let mut v =
        if content.len() < 256 { vec![0x35, content.len() as u8] } else { vec![0x36, (content.len() >> 8) as u8, content.len() as u8] };
    v.extend_from_slice(content);
    v
}

/// The attribute list (a data-element sequence) of our A2DP Source service record.
fn sdp_record() -> Vec<u8> {
    let mut a = Vec::new();
    let mut attr = |id: u16, val: &[u8]| {
        a.push(0x09);
        a.extend_from_slice(&id.to_be_bytes());
        a.extend_from_slice(val);
    };
    attr(0x0000, &[0x0A, 0x00, 0x01, 0x00, 0x01]); // ServiceRecordHandle
    attr(0x0001, &[0x35, 0x03, 0x19, 0x11, 0x0A]); // ServiceClassIDList: A2DP Source
    attr(0x0004, &[0x35, 0x10, 0x35, 0x06, 0x19, 0x01, 0x00, 0x09, 0x00, 0x19, 0x35, 0x06, 0x19, 0x00, 0x19, 0x09, 0x01, 0x03]); // L2CAP psm 0x19, AVDTP 1.3
    attr(0x0009, &[0x35, 0x08, 0x35, 0x06, 0x19, 0x11, 0x0D, 0x09, 0x01, 0x03]); // A2DP 1.3
    attr(0x0100, &[0x25, 0x06, b'a', b'i', b'r', b'l', b'o', b'w']); // ServiceName
    attr(0x0311, &[0x09, 0x00, 0x01]); // SupportedFeatures: Player
    sdp_seq(&a)
}

/// `connect`, and if the controller says a link to the sink already exists (0x0B: it reconnected on
/// its own between our Reset and our connect), reset via `reinit` and adopt/redo once.
pub fn connect_with_retry(h: &mut Hci, addr: &Addr, key: &[u8; 16], reinit: fn(&mut Hci) -> Result<()>) -> Result<u16> {
    match connect(h, addr, key) {
        Err(e) if e.to_string().contains("status 0x0b") => {
            println!("link to the sink already existed (0x0B); resetting the controller and retrying once");
            reinit(h)?;
            connect(h, addr, key)
        }
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::{SimConfig, spawn};

    /// A Link on a throwaway simulated controller, for exercising reassembly directly.
    fn with_link(f: impl FnOnce(&mut Link)) {
        let (mut h, _sim) = spawn(SimConfig::default());
        let mut l = Link::new(&mut h, 0x32).unwrap();
        f(&mut l);
    }

    fn acl(pb: u16, data: &[u8]) -> Vec<u8> {
        let mut v = (0x32u16 | (pb << 12)).to_le_bytes().to_vec();
        v.extend_from_slice(&(data.len() as u16).to_le_bytes());
        v.extend_from_slice(data);
        v
    }

    fn pdu(cid: u16, payload: &[u8]) -> Vec<u8> {
        proto::l2cap(cid, payload)
    }

    #[test]
    fn single_fragment_pdu_is_delivered() {
        with_link(|l| {
            l.on_acl(&acl(2, &pdu(0x40, &[1, 2, 3])));
            assert_eq!(l.inbox.pop_front(), Some((0x40, vec![1, 2, 3])));
            assert!(l.inbox.is_empty());
        });
    }

    #[test]
    fn orphan_continuation_is_ignored_and_does_not_poison_the_next_pdu() {
        with_link(|l| {
            l.on_acl(&acl(1, &[9, 9, 9, 9, 9, 9]));
            assert!(l.inbox.is_empty());
            assert!(l.rx.is_empty(), "orphan bytes must not be buffered");
            l.on_acl(&acl(2, &pdu(0x41, &[7, 7])));
            assert_eq!(l.inbox.pop_front(), Some((0x41, vec![7, 7])));
            assert!(l.inbox.is_empty());
        });
    }

    #[test]
    fn bytes_past_the_pdu_length_are_dropped_not_delivered() {
        with_link(|l| {
            let mut p = pdu(0x40, &[1, 2, 3]);
            p.extend_from_slice(&[0xEE, 0xEE, 0xEE]); // controller glitch: extra bytes after the PDU
            l.on_acl(&acl(2, &p));
            assert_eq!(l.inbox.pop_front(), Some((0x40, vec![1, 2, 3])), "payload must be exactly the declared length");
        });
    }

    #[test]
    fn start_fragment_too_short_for_the_length_field_still_reassembles() {
        with_link(|l| {
            let p = pdu(0x42, &[5, 6, 7, 8]);
            l.on_acl(&acl(2, &p[..1]));
            assert!(l.inbox.is_empty());
            l.on_acl(&acl(1, &p[1..]));
            assert_eq!(l.inbox.pop_front(), Some((0x42, vec![5, 6, 7, 8])));
        });
    }

    #[test]
    fn a_new_start_discards_an_unfinished_pdu() {
        with_link(|l| {
            let p = pdu(0x40, &[1; 20]);
            l.on_acl(&acl(2, &p[..10])); // never completed
            l.on_acl(&acl(2, &pdu(0x41, &[2, 2])));
            assert_eq!(l.inbox.pop_front(), Some((0x41, vec![2, 2])));
            assert!(l.inbox.is_empty());
        });
    }

    #[test]
    fn random_fragmentation_always_reassembles_exactly() {
        with_link(|l| {
            let mut seed = 0x1234_5678u32;
            let mut rnd = |n: usize| {
                seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                (seed >> 8) as usize % n
            };
            for round in 0..300 {
                let len = rnd(600) + 1;
                let payload: Vec<u8> = (0..len).map(|i| (i * 7 + round) as u8).collect();
                let cid = 0x40 + rnd(4) as u16;
                let full = pdu(cid, &payload);
                let mut o = 0;
                let mut first = true;
                while o < full.len() {
                    let n = (rnd(40) + 1).min(full.len() - o);
                    l.on_acl(&acl(if first { 2 } else { 1 }, &full[o..o + n]));
                    first = false;
                    o += n;
                }
                assert_eq!(l.inbox.pop_front(), Some((cid, payload)), "round {round}");
                assert!(l.inbox.is_empty(), "round {round}: spurious extra PDU");
            }
        });
    }
}

/// Change the SBC configuration of a live stream the way every sink supports: Suspend, Close, drop the
/// media channel, Set Configuration, Open, reopen the media channel, Start. (AVDTP Reconfigure is
/// optional and AirPods reject it.) Returns the new media channel id.
pub fn restart_with(l: &mut Link, sig_cid: u16, seid: u8, old_media: u16, cfg: &sbc::Config) -> Result<u16> {
    let t = l.next_txn();
    l.avdtp(sig_cid, t, proto::AVDTP_SUSPEND, &[seid << 2])?;
    let t = l.next_txn();
    l.avdtp(sig_cid, t, proto::AVDTP_CLOSE, &[seid << 2])?;
    l.close_channel(old_media, 0x41)?;
    let t = l.next_txn();
    l.avdtp(sig_cid, t, proto::AVDTP_SET_CONFIGURATION, &proto::set_sbc_configuration(seid, 1, cfg))?;
    let t = l.next_txn();
    l.avdtp(sig_cid, t, proto::AVDTP_OPEN, &[seid << 2])?;
    let (media, _mtu) = l.open_channel(proto::AVDTP_PSM, 0x41)?;
    let t = l.next_txn();
    l.avdtp(sig_cid, t, proto::AVDTP_START, &[seid << 2])?;
    Ok(media)
}

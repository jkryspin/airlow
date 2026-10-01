//! HCI transport + command layer.
//!
//! The wire side is a [`Backend`]: [`UsbBackend`] talks WinUSB (events on 0x81 interrupt, ACL on
//! 0x82/0x02 bulk, commands as class control transfers); the simulator in `sim.rs` implements the
//! same trait so the whole stack can be tested without hardware.

use anyhow::{Context, Result, bail};
use rusb::{DeviceHandle, GlobalContext};
use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, RecvTimeoutError, channel};
use std::time::{Duration, Instant};

const EP_EVT: u8 = 0x81;
const EP_ACL_IN: u8 = 0x82;
const EP_ACL_OUT: u8 = 0x02;

pub type Addr = [u8; 6]; // little-endian as on the wire

pub fn fmt_addr(a: &Addr) -> String {
    a.iter().rev().map(|b| format!("{b:02X}")).collect::<Vec<_>>().join(":")
}

pub fn trace() -> bool {
    std::env::var_os("AIRLOW_TRACE").is_some()
}

/// Is this a well-formed HCI event? Handlers index fixed offsets, so every event must be at least as
/// long as the minimum for its code, and no shorter than its own length field claims.
pub fn valid_event(e: &[u8]) -> bool {
    let Some(&code) = e.first() else { return false };
    if e.len() < 2 || e.len() < e[1] as usize + 2 {
        return false;
    }
    let min = match code {
        0x01 => 3,
        0x03 => 13,
        0x05 => 6,
        0x06 => 5,
        0x08 => 6,
        0x0E | 0x0F => 6,
        0x11 => 4,
        0x12 => 10,
        0x13 => 3,
        0x14 => 8,
        0x16 | 0x17 | 0x31 | 0x32 | 0x33 => 8,
        0x18 => 25,
        0x22 | 0x2F => 3,
        0x36 => 9,
        _ => 2,
    };
    e.len() >= min
}

/// Host -> controller path. Receive paths are channels handed to [`Hci::from_parts`].
pub trait Backend: Send + Sync {
    /// A complete HCI command packet: opcode(2) + length(1) + params.
    fn send_cmd(&self, pkt: &[u8]) -> Result<()>;
    /// A complete HCI ACL packet: handle/flags(2) + length(2) + data.
    fn send_acl(&self, pkt: &[u8]) -> Result<()>;
}

/// USB bulk transfers whose length is a multiple of the endpoint packet size must be terminated
/// by a zero-length packet, or the controller never sees the end of the transfer.
pub fn bulk_out_chunks(pkt: &[u8]) -> Vec<&[u8]> {
    if !pkt.is_empty() && pkt.len() % 64 == 0 { vec![pkt, &[]] } else { vec![pkt] }
}

struct UsbBackend {
    h: Arc<DeviceHandle<GlobalContext>>,
}

impl Backend for UsbBackend {
    fn send_cmd(&self, pkt: &[u8]) -> Result<()> {
        self.h.write_control(0x20, 0, 0, 0, pkt, Duration::from_millis(1000))?;
        Ok(())
    }

    fn send_acl(&self, pkt: &[u8]) -> Result<()> {
        for c in bulk_out_chunks(pkt) {
            self.h.write_bulk(EP_ACL_OUT, c, Duration::from_millis(1000))?;
        }
        Ok(())
    }
}

pub struct Hci {
    backend: Box<dyn Backend>,
    evt_rx: Receiver<Vec<u8>>,
    pub acl_rx: Receiver<Vec<u8>>,
    backlog: VecDeque<Vec<u8>>,
    stop: Arc<AtomicBool>,
}

impl Hci {
    pub fn open(vid: u16, pid: u16) -> Result<Self> {
        let h = rusb::open_device_with_vid_pid(vid, pid).context("controller not found or not bound to WinUSB (see docs/setup.md)")?;
        h.claim_interface(0).context("claim interface 0 (HCI)")?;
        let h = Arc::new(h);
        let stop = Arc::new(AtomicBool::new(false));
        let (etx, evt_rx) = channel();
        let (atx, acl_rx) = channel();
        {
            let (h, stop) = (h.clone(), stop.clone());
            std::thread::spawn(move || {
                let mut b = [0u8; 512];
                while !stop.load(Ordering::Relaxed) {
                    match h.read_interrupt(EP_EVT, &mut b, Duration::from_millis(200)) {
                        Ok(n) => {
                            let _ = etx.send(b[..n].to_vec());
                        }
                        Err(rusb::Error::Timeout) => {}
                        Err(_) => std::thread::sleep(Duration::from_millis(20)), // device gone: do not spin
                    }
                }
            });
        }
        {
            let (h, stop) = (h.clone(), stop.clone());
            std::thread::spawn(move || {
                let mut b = [0u8; 2048];
                while !stop.load(Ordering::Relaxed) {
                    match h.read_bulk(EP_ACL_IN, &mut b, Duration::from_millis(200)) {
                        Ok(n) => {
                            let _ = atx.send(b[..n].to_vec());
                        }
                        Err(rusb::Error::Timeout) => {}
                        Err(_) => std::thread::sleep(Duration::from_millis(20)),
                    }
                }
            });
        }
        let mut s = Self::from_parts(Box::new(UsbBackend { h }), evt_rx, acl_rx);
        s.stop = stop;
        Ok(s)
    }

    pub fn from_parts(backend: Box<dyn Backend>, evt_rx: Receiver<Vec<u8>>, acl_rx: Receiver<Vec<u8>>) -> Self {
        Self { backend, evt_rx, acl_rx, backlog: VecDeque::new(), stop: Arc::new(AtomicBool::new(false)) }
    }

    pub fn send_cmd(&self, opcode: u16, params: &[u8]) -> Result<()> {
        let mut pkt = Vec::with_capacity(3 + params.len());
        pkt.extend_from_slice(&opcode.to_le_bytes());
        pkt.push(params.len() as u8);
        pkt.extend_from_slice(params);
        self.backend.send_cmd(&pkt)
    }

    /// Next event (from backlog first), or None on timeout.
    pub fn next_event(&mut self, timeout: Duration) -> Option<Vec<u8>> {
        if let Some(e) = self.backlog.pop_front() {
            return Some(e);
        }
        let deadline = Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match self.evt_rx.recv_timeout(left) {
                Ok(e) if valid_event(&e) => {
                    if trace() {
                        eprintln!("evt {:02x?}", e);
                    }
                    return Some(e);
                }
                Ok(e) => {
                    if trace() {
                        eprintln!("dropped malformed event {:02x?}", e);
                    }
                }
                Err(RecvTimeoutError::Timeout) | Err(RecvTimeoutError::Disconnected) => return None,
            }
        }
    }

    /// Send a command and wait for its Command Complete; returns return parameters (status first).
    /// For commands answered with Command Status, returns [status].
    pub fn cmd(&mut self, opcode: u16, params: &[u8]) -> Result<Vec<u8>> {
        self.send_cmd(opcode, params)?;
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut stash = Vec::new();
        let res = loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let Some(e) = self.evt_rx_direct(left) else { bail!("timeout opcode {opcode:#06x}") };
            match e[0] {
                0x0E if e.len() >= 6 && u16::from_le_bytes([e[3], e[4]]) == opcode => break e[5..].to_vec(),
                0x0F if e.len() >= 6 && u16::from_le_bytes([e[4], e[5]]) == opcode => break vec![e[2]],
                _ => stash.push(e),
            }
        };
        for e in stash.into_iter().rev() {
            self.backlog.push_front(e);
        }
        if res[0] != 0 {
            bail!("opcode {opcode:#06x} failed, status {:#04x}", res[0]);
        }
        Ok(res)
    }

    fn evt_rx_direct(&mut self, timeout: Duration) -> Option<Vec<u8>> {
        // Bypass backlog so a command waits only for fresh events.
        let deadline = Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            match self.evt_rx.recv_timeout(left) {
                Ok(e) if valid_event(&e) => {
                    if trace() {
                        eprintln!("evt {:02x?}", e);
                    }
                    return Some(e);
                }
                Ok(_) => {}
                Err(_) => return None,
            }
        }
    }

    /// Discard everything queued so far (stale events from before a Reset).
    pub fn drain(&mut self) {
        self.backlog.clear();
        while self.evt_rx.try_recv().is_ok() {}
        while self.acl_rx.try_recv().is_ok() {}
    }

    pub fn send_acl(&self, pkt: &[u8]) -> Result<()> {
        self.backend.send_acl(pkt)
    }
}

impl Drop for Hci {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn malformed_events_are_rejected_and_wellformed_ones_pass() {
        assert!(!valid_event(&[]));
        assert!(!valid_event(&[0x05]));
        assert!(!valid_event(&[0x05, 4, 0, 0x32, 0]), "disconnect missing its reason byte");
        assert!(valid_event(&[0x05, 4, 0, 0x32, 0, 0x13]));
        assert!(!valid_event(&[0x03, 11, 0, 0x32, 0]), "truncated connection complete");
        assert!(!valid_event(&[0x0E, 10, 1, 3, 12, 0]), "length field claims more than present");
        assert!(valid_event(&[0x0E, 4, 1, 3, 12, 0]));
        let mut key = vec![0x18, 23];
        key.extend_from_slice(&[0u8; 23]);
        assert!(valid_event(&key));
        key.truncate(20);
        assert!(!valid_event(&key));
        assert!(valid_event(&[0xFF, 0]), "unknown events of valid framing pass through");
    }

    #[test]
    fn zlp_only_for_exact_multiples_of_the_usb_packet_size() {
        assert_eq!(bulk_out_chunks(&[0u8; 63]).len(), 1);
        assert_eq!(bulk_out_chunks(&[0u8; 65]).len(), 1);
        let c = bulk_out_chunks(&[0u8; 128]);
        assert_eq!(c.len(), 2);
        assert!(c[1].is_empty());
        assert_eq!(bulk_out_chunks(&[0u8; 512]).len(), 2);
    }
}

//! Wire formats: HCI ACL, L2CAP signalling, AVDTP signalling, RTP/A2DP media packets.
//! Pure functions, no I/O.

// ---------- HCI ACL ----------

/// ACL packet header + payload. `pb`: 0x00 start non-auto-flushable, 0x02 start auto-flushable.
/// Auto-flushable (0x02) lets the controller drop stale audio at the flush timeout instead of
/// retransmitting it forever: that is the core latency lever.
pub fn acl(handle: u16, pb: u8, payload: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(4 + payload.len());
    v.extend_from_slice(&((handle & 0x0FFF) | ((pb as u16 & 3) << 12)).to_le_bytes());
    v.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    v.extend_from_slice(payload);
    v
}

pub fn l2cap(cid: u16, payload: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(4 + payload.len());
    v.extend_from_slice(&(payload.len() as u16).to_le_bytes());
    v.extend_from_slice(&cid.to_le_bytes());
    v.extend_from_slice(payload);
    v
}

// ---------- L2CAP signalling (CID 0x0001) ----------

fn sig(code: u8, id: u8, data: &[u8]) -> Vec<u8> {
    let mut v = vec![code, id];
    v.extend_from_slice(&(data.len() as u16).to_le_bytes());
    v.extend_from_slice(data);
    v
}

pub fn l2cap_connect_req(id: u8, psm: u16, scid: u16) -> Vec<u8> {
    let mut d = psm.to_le_bytes().to_vec();
    d.extend_from_slice(&scid.to_le_bytes());
    sig(0x02, id, &d)
}

/// Configuration request with optional MTU and flush-timeout options.
/// `flush_ms`: 0xFFFF = reliable, 1 = best effort / no retransmission, else ms.
pub fn l2cap_config_req(id: u8, dcid: u16, mtu: Option<u16>, flush_ms: Option<u16>) -> Vec<u8> {
    let mut d = dcid.to_le_bytes().to_vec();
    d.extend_from_slice(&0u16.to_le_bytes()); // flags
    if let Some(m) = mtu {
        d.extend_from_slice(&[0x01, 0x02]);
        d.extend_from_slice(&m.to_le_bytes());
    }
    if let Some(f) = flush_ms {
        d.extend_from_slice(&[0x02, 0x02]);
        d.extend_from_slice(&f.to_le_bytes());
    }
    sig(0x04, id, &d)
}

pub fn l2cap_config_rsp(id: u8, scid: u16, result: u16) -> Vec<u8> {
    let mut d = scid.to_le_bytes().to_vec();
    d.extend_from_slice(&0u16.to_le_bytes());
    d.extend_from_slice(&result.to_le_bytes());
    sig(0x05, id, &d)
}

// ---------- AVDTP signalling ----------

pub const AVDTP_PSM: u16 = 0x0019;
pub const AVDTP_DISCOVER: u8 = 0x01;
pub const AVDTP_GET_CAPABILITIES: u8 = 0x02;
pub const AVDTP_SET_CONFIGURATION: u8 = 0x03;
pub const AVDTP_RECONFIGURE: u8 = 0x05;
pub const AVDTP_CLOSE: u8 = 0x08;
pub const AVDTP_OPEN: u8 = 0x06;
pub const AVDTP_START: u8 = 0x07;
pub const AVDTP_SUSPEND: u8 = 0x09;

/// message type: 0 command, 2 response accept, 3 response reject
pub fn avdtp(txn: u8, msg_type: u8, signal: u8, params: &[u8]) -> Vec<u8> {
    let mut v = vec![(txn << 4) | (msg_type & 3), signal];
    v.extend_from_slice(params);
    v
}

/// Set_Configuration params for SBC. Returns the full parameter block.
pub fn set_sbc_configuration(acp_seid: u8, int_seid: u8, cfg: &crate::sbc::Config) -> Vec<u8> {
    use crate::sbc::{Mode, Rate};
    let freq = match cfg.rate {
        Rate::Hz44100 => 0x20,
        Rate::Hz48000 => 0x10,
    };
    let chmode = match cfg.mode {
        Mode::Stereo => 0x02,
        Mode::JointStereo => 0x01,
    };
    let blocks = match cfg.blocks {
        4 => 0x80,
        8 => 0x40,
        12 => 0x20,
        _ => 0x10,
    };
    let sb = if cfg.subbands == 4 { 0x08 } else { 0x04 };
    let alloc = if cfg.snr { 0x02 } else { 0x01 };
    vec![
        acp_seid << 2,
        int_seid << 2,
        0x01,
        0x00, // Media Transport
        0x07,
        0x06,
        0x00,
        0x00, // Media Codec, audio, SBC
        freq | chmode,
        blocks | sb | alloc,
        cfg.bitpool, // min
        cfg.bitpool, // max
    ]
}

/// Reconfigure params (allowed while Open, i.e. after Suspend): ACP seid + the SBC codec capability only.
pub fn reconfigure_sbc(acp_seid: u8, cfg: &crate::sbc::Config) -> Vec<u8> {
    let full = set_sbc_configuration(acp_seid, 1, cfg);
    let mut v = vec![acp_seid << 2];
    v.extend_from_slice(&full[4..]); // drop seids and the media-transport item
    v
}

// ---------- RTP / A2DP media ----------

/// One media packet carrying `nframes` whole SBC frames.
pub fn rtp_sbc(seq: u16, timestamp: u32, ssrc: u32, nframes: u8, frames: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(13 + frames.len());
    v.extend_from_slice(&[0x80, 0x60]); // V=2, PT=96 (dynamic)
    v.extend_from_slice(&seq.to_be_bytes());
    v.extend_from_slice(&timestamp.to_be_bytes());
    v.extend_from_slice(&ssrc.to_be_bytes());
    v.push(nframes & 0x0F); // SBC payload header: not fragmented
    v.extend_from_slice(frames);
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn acl_header_auto_flushable() {
        let p = acl(0x0040, 0x02, &[1, 2, 3]);
        assert_eq!(p, [0x40, 0x20, 3, 0, 1, 2, 3]);
    }

    #[test]
    fn l2cap_config_with_flush() {
        let r = l2cap_config_req(7, 0x0041, Some(895), Some(20));
        assert_eq!(r[0], 0x04);
        assert_eq!(u16::from_le_bytes([r[2], r[3]]) as usize, r.len() - 4);
        assert_eq!(&r[8..], [0x01, 0x02, 0x7F, 0x03, 0x02, 0x02, 20, 0]);
    }

    #[test]
    fn avdtp_header() {
        assert_eq!(avdtp(3, 0, AVDTP_DISCOVER, &[]), [0x30, 0x01]);
    }

    #[test]
    fn rtp_layout() {
        let p = rtp_sbc(1, 2, 3, 2, &[0xAA]);
        assert_eq!(p.len(), 14);
        assert_eq!(p[12], 2);
        assert_eq!(p[13], 0xAA);
    }

    #[test]
    fn reconfigure_carries_only_the_codec_item() {
        let cfg = crate::sbc::Config {
            rate: crate::sbc::Rate::Hz44100,
            mode: crate::sbc::Mode::Stereo,
            blocks: 16,
            subbands: 8,
            bitpool: 35,
            snr: false,
        };
        let r = reconfigure_sbc(1, &cfg);
        assert_eq!(r, [0x04, 0x07, 0x06, 0x00, 0x00, 0x22, 0x15, 35, 35]);
    }

    #[test]
    fn sbc_config_bytes() {
        let cfg = crate::sbc::Config {
            rate: crate::sbc::Rate::Hz48000,
            mode: crate::sbc::Mode::JointStereo,
            blocks: 8,
            subbands: 8,
            bitpool: 53,
            snr: false,
        };
        let p = set_sbc_configuration(1, 1, &cfg);
        assert_eq!(&p[8..], [0x11, 0x45, 53, 53]);
    }
}

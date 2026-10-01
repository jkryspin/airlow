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

// ---------- AAC (MPEG-2,4 AAC) codec information element ----------

pub const CODEC_SBC: u8 = 0x00;
pub const CODEC_AAC: u8 = 0x02;
pub const AAC_MPEG2_LC: u8 = 0x80;
pub const AAC_MPEG4_LC: u8 = 0x40;

/// Capabilities a sink advertises for AAC (the 6-byte codec information element).
#[derive(Debug, Clone, PartialEq)]
pub struct AacCaps {
    pub object_types: u8,
    /// Sampling frequencies as in the element: byte 1 bits and byte 2 high nibble combined (little end = 44.1k).
    pub freq_byte1: u8,
    pub freq_byte2: u8,
    pub channels: u8, // 0x08 mono, 0x04 stereo
    pub vbr: bool,
    pub bitrate: u32,
}

pub fn parse_aac_caps(info: &[u8]) -> Option<AacCaps> {
    if info.len() < 6 {
        return None;
    }
    Some(AacCaps {
        object_types: info[0],
        freq_byte1: info[1],
        freq_byte2: info[2] & 0xF0,
        channels: info[2] & 0x0C,
        vbr: info[3] & 0x80 != 0,
        bitrate: ((info[3] & 0x7F) as u32) << 16 | (info[4] as u32) << 8 | info[5] as u32,
    })
}

/// (byte 1, byte 2 high nibble) for one sampling frequency.
pub fn aac_freq_bits(rate: u32) -> Option<(u8, u8)> {
    Some(match rate {
        8000 => (0x80, 0),
        11025 => (0x40, 0),
        12000 => (0x20, 0),
        16000 => (0x10, 0),
        22050 => (0x08, 0),
        24000 => (0x04, 0),
        32000 => (0x02, 0),
        44100 => (0x01, 0),
        48000 => (0, 0x80),
        64000 => (0, 0x40),
        88200 => (0, 0x20),
        96000 => (0, 0x10),
        _ => return None,
    })
}

/// Set_Configuration for AAC: one object type, one frequency, stereo, CBR at `bitrate`.
pub fn set_aac_configuration(acp_seid: u8, int_seid: u8, object_type: u8, rate: u32, vbr: bool, bitrate: u32) -> Option<Vec<u8>> {
    let (f1, f2) = aac_freq_bits(rate)?;
    Some(vec![
        acp_seid << 2,
        int_seid << 2,
        0x01,
        0x00, // Media Transport
        0x07,
        0x08,
        0x00,
        CODEC_AAC, // Media Codec: audio, MPEG-2,4 AAC, 6 bytes of information
        object_type,
        f1,
        f2 | 0x04, // stereo
        (if vbr { 0x80 } else { 0 }) | ((bitrate >> 16) & 0x7F) as u8,
        (bitrate >> 8) as u8,
        bitrate as u8,
    ])
}

// ---------- RTP / A2DP media ----------

/// AAC media packet: RTP header with the marker bit set (complete LATM element) followed directly by the LATM payload.
pub fn rtp_aac(seq: u16, timestamp: u32, ssrc: u32, latm: &[u8]) -> Vec<u8> {
    let mut v = Vec::with_capacity(12 + latm.len());
    v.extend_from_slice(&[0x80, 0xE0]); // V=2, M=1, PT=96
    v.extend_from_slice(&seq.to_be_bytes());
    v.extend_from_slice(&timestamp.to_be_bytes());
    v.extend_from_slice(&ssrc.to_be_bytes());
    v.extend_from_slice(latm);
    v
}

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
    fn aac_configuration_bytes_match_the_a2dp_layout() {
        // MPEG-2 AAC LC, 48 kHz (byte 2 high nibble 0x8), stereo (0x4), CBR 160000 bit/s = 0x027100
        let c = set_aac_configuration(2, 1, AAC_MPEG2_LC, 48000, false, 160_000).unwrap();
        assert_eq!(c, [0x08, 0x04, 0x01, 0x00, 0x07, 0x08, 0x00, 0x02, 0x80, 0x00, 0x84, 0x02, 0x71, 0x00]);
        // 44.1 kHz lives in byte 1
        let c = set_aac_configuration(2, 1, AAC_MPEG4_LC, 44100, true, 320_000).unwrap();
        assert_eq!(&c[8..], &[0x40, 0x01, 0x04, 0x84, 0xE2, 0x00]);
        assert!(set_aac_configuration(2, 1, AAC_MPEG2_LC, 12345, false, 1).is_none());
    }

    #[test]
    fn aac_caps_parse_like_the_airpods_advertise() {
        // multiple object-type bits (as AirPods set), 44.1/48 kHz, mono+stereo, VBR, 320 kbit/s
        let c = parse_aac_caps(&[0xC0, 0x01, 0x8C, 0x84, 0xE2, 0x00]).unwrap();
        assert_eq!(c.object_types, 0xC0);
        assert_eq!((c.freq_byte1, c.freq_byte2), (0x01, 0x80));
        assert_eq!(c.channels, 0x0C);
        assert!(c.vbr);
        assert_eq!(c.bitrate, 320_000);
        assert!(parse_aac_caps(&[1, 2, 3]).is_none());
    }

    #[test]
    fn aac_rtp_has_marker_and_no_extra_header() {
        let p = rtp_aac(5, 1024, 9, &[0xAA, 0xBB]);
        assert_eq!(&p[..2], &[0x80, 0xE0]);
        assert_eq!(&p[2..4], &[0, 5]);
        assert_eq!(&p[4..8], &1024u32.to_be_bytes());
        assert_eq!(&p[12..], &[0xAA, 0xBB]);
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

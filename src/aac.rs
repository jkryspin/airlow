//! Optional AAC-LC support for A2DP: Windows' built-in AAC encoder (Media Foundation), LATM packaging
//! (RFC 3016 / A2DP, stream config carried in-band in every frame) and, for tests and the simulator, the
//! matching decoder. Windows only, no third-party codec library.

#![cfg(windows)]

use anyhow::{Result, anyhow, bail};
use std::mem::ManuallyDrop;
use std::ptr::null_mut;
use windows::Win32::Media::MediaFoundation::*;
use windows::Win32::System::Com::*;
use windows::core::GUID;

/// AAC-LC frame length in samples per channel.
pub const FRAME_SAMPLES: usize = 1024;

/// AAC sampling frequency index (ISO 14496-3 table 1.16) for the rates A2DP uses.
pub fn rate_index(rate: u32) -> Option<u8> {
    Some(match rate {
        96000 => 0,
        88200 => 1,
        64000 => 2,
        48000 => 3,
        44100 => 4,
        32000 => 5,
        24000 => 6,
        22050 => 7,
        16000 => 8,
        _ => return None,
    })
}

pub fn index_rate(i: u8) -> Option<u32> {
    [96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000].get(i as usize).copied()
}

// ---------------------------------------------------------------------------------------------
// LATM (audioMuxVersion 0, muxConfigPresent = 1: StreamMuxConfig in every AudioMuxElement)
// ---------------------------------------------------------------------------------------------

struct BitWriter {
    bytes: Vec<u8>,
    nbits: usize,
}

impl BitWriter {
    fn new() -> Self {
        Self { bytes: vec![], nbits: 0 }
    }
    fn put(&mut self, value: u32, bits: usize) {
        for i in (0..bits).rev() {
            if self.nbits % 8 == 0 {
                self.bytes.push(0);
            }
            let bit = ((value >> i) & 1) as u8;
            *self.bytes.last_mut().unwrap() |= bit << (7 - self.nbits % 8);
            self.nbits += 1;
        }
    }
}

struct BitReader<'a> {
    b: &'a [u8],
    pos: usize,
}

impl BitReader<'_> {
    fn get(&mut self, bits: usize) -> Result<u32> {
        let mut v = 0u32;
        for _ in 0..bits {
            let byte = *self.b.get(self.pos / 8).ok_or_else(|| anyhow!("LATM truncated"))?;
            v = (v << 1) | ((byte >> (7 - self.pos % 8)) & 1) as u32;
            self.pos += 1;
        }
        Ok(v)
    }
}

/// Wrap one raw AAC-LC access unit in a LATM AudioMuxElement carrying its own StreamMuxConfig.
pub fn latm_mux(rate_idx: u8, channels: u8, frame: &[u8]) -> Vec<u8> {
    let mut w = BitWriter::new();
    w.put(0, 1); // useSameStreamMux = 0 -> a StreamMuxConfig follows
    w.put(0, 1); // audioMuxVersion = 0
    w.put(1, 1); // allStreamsSameTimeFraming
    w.put(0, 6); // numSubFrames
    w.put(0, 4); // numProgram
    w.put(0, 3); // numLayer
    // AudioSpecificConfig
    w.put(2, 5); // audioObjectType = AAC LC
    w.put(rate_idx as u32, 4);
    w.put(channels as u32, 4); // channelConfiguration (1 = mono, 2 = stereo)
    w.put(0, 1); // frameLengthFlag: 1024
    w.put(0, 1); // dependsOnCoreCoder
    w.put(0, 1); // extensionFlag
    w.put(0, 3); // frameLengthType = 0 (variable length, signalled in PayloadLengthInfo)
    w.put(0xFF, 8); // latmBufferFullness: VBR/unknown
    w.put(0, 1); // otherDataPresent
    w.put(0, 1); // crcCheckPresent
    // PayloadLengthInfo: a run of 255s terminated by the remainder
    let mut len = frame.len();
    while len >= 255 {
        w.put(255, 8);
        len -= 255;
    }
    w.put(len as u32, 8);
    for &b in frame {
        w.put(b as u32, 8);
    }
    w.bytes // trailing bits of the last byte are already zero (byte alignment)
}

/// Parse a LATM AudioMuxElement produced by [`latm_mux`]: (rate index, channels, raw frame).
pub fn latm_demux(p: &[u8]) -> Result<(u8, u8, Vec<u8>)> {
    let mut r = BitReader { b: p, pos: 0 };
    if r.get(1)? != 0 {
        bail!("LATM without in-band StreamMuxConfig");
    }
    if r.get(1)? != 0 {
        bail!("audioMuxVersion != 0");
    }
    r.get(1)?; // allStreamsSameTimeFraming
    if r.get(6)? != 0 || r.get(4)? != 0 || r.get(3)? != 0 {
        bail!("multiple sub-frames/programs/layers are not supported");
    }
    if r.get(5)? != 2 {
        bail!("not AAC LC");
    }
    let (ri, ch) = (r.get(4)? as u8, r.get(4)? as u8);
    r.get(3)?; // frameLengthFlag, dependsOnCoreCoder, extensionFlag
    if r.get(3)? != 0 {
        bail!("frameLengthType != 0");
    }
    r.get(8)?; // latmBufferFullness
    if r.get(1)? != 0 || r.get(1)? != 0 {
        bail!("otherData/crc present");
    }
    let mut len = 0usize;
    loop {
        let b = r.get(8)? as usize;
        len += b;
        if b != 255 {
            break;
        }
    }
    let mut frame = Vec::with_capacity(len);
    for _ in 0..len {
        frame.push(r.get(8)? as u8);
    }
    Ok((ri, ch, frame))
}

// ---------------------------------------------------------------------------------------------
// Media Foundation plumbing
// ---------------------------------------------------------------------------------------------

fn mf_init() -> Result<()> {
    // COM may already be initialised on this thread (S_FALSE) or with another model (RPC_E_CHANGED_MODE): both fine.
    let _ = unsafe { CoInitializeEx(None, COINIT_MULTITHREADED) };
    unsafe { MFStartup(MF_VERSION, MFSTARTUP_FULL)? };
    Ok(())
}

fn audio_type(subtype: &GUID, rate: u32, channels: u32, avg_bytes: u32, block_align: u32) -> Result<IMFMediaType> {
    unsafe {
        let t = MFCreateMediaType()?;
        t.SetGUID(&MF_MT_MAJOR_TYPE, &MFMediaType_Audio)?;
        t.SetGUID(&MF_MT_SUBTYPE, subtype)?;
        t.SetUINT32(&MF_MT_AUDIO_BITS_PER_SAMPLE, 16)?;
        t.SetUINT32(&MF_MT_AUDIO_SAMPLES_PER_SECOND, rate)?;
        t.SetUINT32(&MF_MT_AUDIO_NUM_CHANNELS, channels)?;
        t.SetUINT32(&MF_MT_AUDIO_AVG_BYTES_PER_SECOND, avg_bytes)?;
        t.SetUINT32(&MF_MT_AUDIO_BLOCK_ALIGNMENT, block_align)?;
        Ok(t)
    }
}

fn sample_from(bytes: &[u8], time_100ns: i64, duration_100ns: i64) -> Result<IMFSample> {
    unsafe {
        let buf = MFCreateMemoryBuffer(bytes.len() as u32)?;
        let mut p: *mut u8 = null_mut();
        buf.Lock(&mut p, None, None)?;
        std::ptr::copy_nonoverlapping(bytes.as_ptr(), p, bytes.len());
        buf.Unlock()?;
        buf.SetCurrentLength(bytes.len() as u32)?;
        let s = MFCreateSample()?;
        s.AddBuffer(&buf)?;
        s.SetSampleTime(time_100ns)?;
        s.SetSampleDuration(duration_100ns)?;
        Ok(s)
    }
}

/// Drain every output the transform has ready (`None` = needs more input).
fn drain_outputs(mft: &IMFTransform) -> Result<Vec<Vec<u8>>> {
    let mut out = Vec::new();
    loop {
        unsafe {
            let info = mft.GetOutputStreamInfo(0)?;
            let provides = info.dwFlags & MFT_OUTPUT_STREAM_PROVIDES_SAMPLES.0 as u32 != 0;
            let mut db = MFT_OUTPUT_DATA_BUFFER { dwStreamID: 0, ..Default::default() };
            if !provides {
                let buf = MFCreateMemoryBuffer(info.cbSize.max(4096))?;
                let s = MFCreateSample()?;
                s.AddBuffer(&buf)?;
                db.pSample = ManuallyDrop::new(Some(s));
            }
            let mut status = 0u32;
            let r = mft.ProcessOutput(0, std::slice::from_mut(&mut db), &mut status);
            let sample = ManuallyDrop::take(&mut db.pSample);
            let _events = ManuallyDrop::take(&mut db.pEvents);
            match r {
                Ok(()) => {
                    if let Some(s) = sample {
                        let b = s.ConvertToContiguousBuffer()?;
                        let (mut p, mut len): (*mut u8, u32) = (null_mut(), 0);
                        b.Lock(&mut p, None, Some(&mut len))?;
                        out.push(std::slice::from_raw_parts(p, len as usize).to_vec());
                        b.Unlock()?;
                    }
                }
                Err(e) if e.code() == MF_E_TRANSFORM_NEED_MORE_INPUT => return Ok(out),
                Err(e) if e.code() == MF_E_TRANSFORM_STREAM_CHANGE => {
                    // Output type renegotiation: accept the first available type and continue.
                    let t = mft.GetOutputAvailableType(0, 0)?;
                    mft.SetOutputType(0, &t, 0)?;
                }
                Err(e) => return Err(e.into()),
            }
        }
    }
}

/// Windows AAC-LC encoder: interleaved s16 PCM in, raw AAC access units out.
pub struct AacEncoder {
    mft: IMFTransform,
    channels: usize,
    rate: u32,
    /// Running input position in samples per channel (drives the sample timestamps).
    pos: i64,
    /// Frames emitted so far (for delay measurements).
    pub frames_out: usize,
}

impl AacEncoder {
    /// `bitrate` in bits/s; the Windows encoder supports 96, 128, 160 and 192 kbps for stereo at 44.1/48 kHz.
    pub fn new(rate: u32, channels: u32, bitrate: u32) -> Result<Self> {
        mf_init()?;
        unsafe {
            let mft: IMFTransform = CoCreateInstance(&AACMFTEncoder, None, CLSCTX_INPROC_SERVER)?;
            let out = audio_type(&MFAudioFormat_AAC, rate, channels, bitrate / 8, 1)?;
            out.SetUINT32(&MF_MT_AAC_PAYLOAD_TYPE, 0)?; // raw access units
            out.SetUINT32(&MF_MT_AAC_AUDIO_PROFILE_LEVEL_INDICATION, 0x29)?; // AAC-LC, level 2
            mft.SetOutputType(0, &out, 0)?;
            let input = audio_type(&MFAudioFormat_PCM, rate, channels, rate * channels * 2, channels * 2)?;
            mft.SetInputType(0, &input, 0)?;
            mft.ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0)?;
            mft.ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, 0)?;
            Ok(Self { mft, channels: channels as usize, rate, pos: 0, frames_out: 0 })
        }
    }

    /// A warmed-up encoder: created, initialised and fed 120 ms of silence (output discarded) so that after the
    /// stream starts, the first real audio produces frames immediately. Build it BEFORE AVDTP Start: first-time
    /// Media Foundation setup takes long enough to look like dead air, which makes AirPods send AVRCP PAUSE.
    pub fn new_primed(rate: u32, channels: u32, bitrate: u32) -> Result<Self> {
        let mut e = Self::new(rate, channels, bitrate)?;
        let silence = vec![0i16; (rate as usize / 1000 * 10) * channels as usize]; // 10 ms
        for _ in 0..12 {
            e.encode(&silence)?;
        }
        Ok(e)
    }

    /// Feed interleaved PCM (any length); returns the AAC frames that became ready.

    pub fn encode(&mut self, pcm: &[i16]) -> Result<Vec<Vec<u8>>> {
        debug_assert_eq!(pcm.len() % self.channels, 0);
        let bytes: Vec<u8> = pcm.iter().flat_map(|v| v.to_le_bytes()).collect();
        let n = (pcm.len() / self.channels) as i64;
        let (t, d) = (self.pos * 10_000_000 / self.rate as i64, n * 10_000_000 / self.rate as i64);
        self.pos += n;
        let s = sample_from(&bytes, t, d)?;
        unsafe { self.mft.ProcessInput(0, &s, 0)? };
        let frames = drain_outputs(&self.mft)?;
        self.frames_out += frames.len();
        Ok(frames)
    }
}

/// Windows AAC decoder, used by tests and the simulated sink to verify what an AAC sink would play.
pub struct AacDecoder {
    mft: IMFTransform,
    rate: u32,
    pos: i64,
}

// SAFETY: the Windows AAC decoder MFT is not tied to a thread. The simulator creates it on its own thread and
// only ever uses it there; `Send` just lets the enclosing simulator struct be moved into that thread.
unsafe impl Send for AacDecoder {}

impl AacDecoder {
    pub fn new(rate: u32, channels: u32) -> Result<Self> {
        mf_init()?;
        unsafe {
            let mft: IMFTransform = CoCreateInstance(&CLSID_MSAACDecMFT, None, CLSCTX_INPROC_SERVER)?;
            let t = audio_type(&MFAudioFormat_AAC, rate, channels, 16000, 1)?;
            t.SetUINT32(&MF_MT_AAC_PAYLOAD_TYPE, 0)?;
            t.SetUINT32(&MF_MT_AAC_AUDIO_PROFILE_LEVEL_INDICATION, 0x29)?;
            // HEAACWAVEINFO tail (payload type, profile/level, struct type, 2 reserved) + AudioSpecificConfig
            let ri = rate_index(rate).ok_or_else(|| anyhow!("unsupported rate {rate}"))? as u16;
            let asc: u16 = (2 << 11) | (ri << 7) | ((channels as u16) << 3);
            let mut blob = vec![0u8, 0, 0x29, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            blob.extend_from_slice(&asc.to_be_bytes());
            t.SetBlob(&MF_MT_USER_DATA, &blob)?;
            mft.SetInputType(0, &t, 0)?;
            let mut chosen = false;
            for i in 0.. {
                let Ok(ot) = mft.GetOutputAvailableType(0, i) else { break };
                if ot.GetGUID(&MF_MT_SUBTYPE).map(|g| g == MFAudioFormat_PCM).unwrap_or(false) {
                    mft.SetOutputType(0, &ot, 0)?;
                    chosen = true;
                    break;
                }
            }
            if !chosen {
                bail!("AAC decoder offers no PCM output");
            }
            mft.ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0)?;
            mft.ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, 0)?;
            Ok(Self { mft, rate, pos: 0 })
        }
    }

    /// Decode one raw AAC access unit; returns interleaved s16 PCM (may be empty while priming).
    pub fn decode(&mut self, frame: &[u8]) -> Result<Vec<i16>> {
        let (t, d) = (self.pos * 10_000_000 / self.rate as i64, FRAME_SAMPLES as i64 * 10_000_000 / self.rate as i64);
        self.pos += FRAME_SAMPLES as i64;
        let s = sample_from(frame, t, d)?;
        unsafe { self.mft.ProcessInput(0, &s, 0)? };
        Ok(drain_outputs(&self.mft)?
            .into_iter()
            .flat_map(|b| b.chunks_exact(2).map(|c| i16::from_le_bytes([c[0], c[1]])).collect::<Vec<_>>())
            .collect())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latm_roundtrips_frames_of_any_length() {
        let mut seed = 7u32;
        for len in [1usize, 2, 100, 254, 255, 256, 509, 510, 511, 700] {
            let frame: Vec<u8> = (0..len)
                .map(|_| {
                    seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
                    (seed >> 16) as u8
                })
                .collect();
            for (ri, ch) in [(3u8, 2u8), (4, 2), (3, 1)] {
                let m = latm_mux(ri, ch, &frame);
                let (r2, c2, f2) = latm_demux(&m).expect("demux");
                assert_eq!((r2, c2), (ri, ch));
                assert_eq!(f2, frame, "len {len}");
            }
        }
    }

    #[test]
    fn latm_header_has_the_documented_bit_layout() {
        // 48 kHz stereo: 0 0 1 000000 0000 000 | 00010 0011 0010 0 0 0 | 000 | 11111111 | 0 0 -> 45 bits, then length.
        let m = latm_mux(3, 2, &[0xAB; 3]);
        // bits: 001 00000 | 00000 000 | 00010 001 | 10010000 | 0000 1111 | 1111 00 + len(3)=00000011 ...
        assert_eq!(&m[..4], &[0b0010_0000, 0b0000_0000, 0b0001_0001, 0b1001_0000]);
        // After 32 bits: frameLengthType(3)=000, latmBufferFullness(8)=0xFF, other(1)=0, crc(1)=0, then length 3.
        assert_eq!(m[4], 0b0001_1111);
        assert_eq!(m[5], 0b1110_0000);
        assert_eq!(m[6], 0b0001_1101); // last 5 bits of the length byte (00011) + first 3 payload bits (101)
        // Total size: 45 header bits + 8 length + 24 payload = 77 bits -> 10 bytes.
        assert_eq!(m.len(), 10);
    }

    #[test]
    fn rate_indices_match_the_standard() {
        assert_eq!(rate_index(48000), Some(3));
        assert_eq!(rate_index(44100), Some(4));
        assert_eq!(index_rate(3), Some(48000));
        assert_eq!(rate_index(12345), None);
    }

    fn stereo_tone(n: usize, rate: f64) -> Vec<i16> {
        (0..n)
            .flat_map(|i| {
                let t = i as f64 / rate;
                [((std::f64::consts::TAU * 440.0 * t).sin() * 8000.0) as i16, ((std::f64::consts::TAU * 1500.0 * t).sin() * 8000.0) as i16]
            })
            .collect()
    }

    #[test]
    fn windows_encoder_produces_decodable_aac_at_the_right_rate() {
        let mut enc = AacEncoder::new(48000, 2, 160_000).expect("Windows AAC encoder");
        let pcm = stereo_tone(48000 * 2, 48000.0); // 2 s
        let mut frames = Vec::new();
        for chunk in pcm.chunks(480 * 2) {
            frames.extend(enc.encode(chunk).unwrap()); // 10 ms at a time, like WASAPI
        }
        println!(
            "{} frames out for 2 s of audio; first frame {} B; avg {:.0} B",
            frames.len(),
            frames[0].len(),
            frames.iter().map(|f| f.len()).sum::<usize>() as f64 / frames.len() as f64
        );
        assert!((frames.len() as i64 - 93).abs() <= 3, "expected ~93 frames for 2 s, got {}", frames.len());
        let avg_kbps = frames.iter().map(|f| f.len()).sum::<usize>() as f64 * 8.0 / 2.0 / 1000.0;
        assert!((100.0..220.0).contains(&avg_kbps), "bitrate {avg_kbps:.0} kbps");
        // Decode and check the tones survived (left 440 Hz, right 1500 Hz).
        let mut dec = AacDecoder::new(48000, 2).expect("Windows AAC decoder");
        let mut out = Vec::new();
        for f in &frames {
            out.extend(dec.decode(f).unwrap());
        }
        assert!(out.len() > 48000 * 2, "decoded only {} samples", out.len());
        let seg = &out[2 * 20000..2 * 70000.min(out.len() / 2)];
        let (l, r): (Vec<i16>, Vec<i16>) = (seg.chunks(2).map(|c| c[0]).collect(), seg.chunks(2).map(|c| c[1]).collect());
        let amp = |x: &[i16], f: f64| {
            let (mut re, mut im) = (0.0, 0.0);
            for (i, &v) in x.iter().enumerate() {
                let w = std::f64::consts::TAU * f * i as f64 / 48000.0;
                re += v as f64 * w.cos();
                im -= v as f64 * w.sin();
            }
            2.0 * (re * re + im * im).sqrt() / x.len() as f64
        };
        let (al, ar) = (amp(&l, 440.0), amp(&r, 1500.0));
        println!("decoded tone amplitudes: L {al:.0} R {ar:.0} (input 8000)");
        assert!((al - 8000.0).abs() < 900.0 && (ar - 8000.0).abs() < 900.0, "tones damaged: {al:.0} / {ar:.0}");
    }

    #[test]
    fn encoder_delay_is_measured_and_bounded() {
        // Steady-state cost of AAC versus SBC: feed 10 ms chunks (like WASAPI), put a loud click in chunk 40,
        // and see how much MORE audio had to be fed before the click comes out of encode+decode.
        let mut enc = AacEncoder::new(48000, 2, 160_000).unwrap();
        let mut dec = AacDecoder::new(48000, 2).unwrap();
        let (mut fed, mut first_frame_after) = (0usize, None);
        let mut out: Vec<i16> = Vec::new();
        let click_chunk = 40;
        let mut click_seen_after = None;
        for k in 0..90 {
            let mut chunk = vec![0i16; 480 * 2];
            if k == click_chunk {
                for s in 0..64 {
                    chunk[s * 2] = if s % 2 == 0 { 20000 } else { -20000 };
                    chunk[s * 2 + 1] = chunk[s * 2];
                }
            }
            fed += 480;
            for f in enc.encode(&chunk).unwrap() {
                first_frame_after.get_or_insert(fed);
                out.extend(dec.decode(&f).unwrap());
            }
            if click_seen_after.is_none() && out.chunks(2).any(|c| c[0].abs() > 6000) {
                click_seen_after = Some(fed);
            }
        }
        let click_in = click_chunk * 480; // sample index where the click starts
        let latency_ms = (click_seen_after.expect("click must come out") as f64 - click_in as f64) / 48.0;
        let prime_ms = first_frame_after.unwrap() as f64 / 48.0;
        println!(
            "AAC: first frame after {prime_ms:.0} ms of input; a click is available {latency_ms:.0} ms after it was fed (includes up to one 10 ms chunk of feed granularity)"
        );
        assert!(latency_ms > 20.0 && latency_ms < 140.0, "AAC encode delay {latency_ms:.0} ms outside the plausible range");
    }
    #[test]
    fn muxed_frames_survive_the_whole_chain() {
        // raw frame -> LATM -> demux -> decode: what the AirPods would receive and decode.
        let mut enc = AacEncoder::new(48000, 2, 128_000).unwrap();
        let pcm = stereo_tone(48000, 48000.0);
        let frames: Vec<Vec<u8>> = pcm.chunks(960 * 2).flat_map(|c| enc.encode(c).unwrap()).collect();
        let mut dec = AacDecoder::new(48000, 2).unwrap();
        let mut samples = 0usize;
        for f in &frames {
            let payload = latm_mux(3, 2, f);
            assert!(payload.len() < 1000, "frame must fit one packet");
            let (ri, ch, raw) = latm_demux(&payload).unwrap();
            assert_eq!((ri, ch), (3, 2));
            assert_eq!(&raw, f);
            samples += dec.decode(&raw).unwrap().len() / 2;
        }
        assert!(samples >= 48000 - 4096, "decoded {samples} samples");
    }
}

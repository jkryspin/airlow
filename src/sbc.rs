//! Safe wrapper over vendored libsbc (LGPL-2.1+). Encoder only.

use std::os::raw::{c_int, c_ulong, c_void};

#[repr(C)]
struct SbcT {
    flags: c_ulong,
    frequency: u8,
    blocks: u8,
    subbands: u8,
    mode: u8,
    allocation: u8,
    bitpool: u8,
    endian: u8,
    _priv: *mut c_void,
    _priv_alloc_base: *mut c_void,
}

unsafe extern "C" {
    fn sbc_init(sbc: *mut SbcT, flags: c_ulong) -> c_int;
    fn sbc_finish(sbc: *mut SbcT);
    fn sbc_encode(
        sbc: *mut SbcT,
        input: *const c_void,
        input_len: usize,
        output: *mut c_void,
        output_len: usize,
        written: *mut isize,
    ) -> isize;
    fn sbc_get_frame_length(sbc: *mut SbcT) -> usize;
    fn sbc_get_codesize(sbc: *mut SbcT) -> usize;
    fn sbc_decode(
        sbc: *mut SbcT,
        input: *const c_void,
        input_len: usize,
        output: *mut c_void,
        output_len: usize,
        written: *mut usize,
    ) -> isize;
}

#[derive(Clone, Copy, Debug)]
pub enum Rate {
    Hz44100 = 2,
    Hz48000 = 3,
}

#[derive(Clone, Copy, Debug)]
pub enum Mode {
    Stereo = 2,
    JointStereo = 3,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub rate: Rate,
    pub mode: Mode,
    /// 4, 8, 12 or 16. Fewer blocks = shorter frames = lower algorithmic delay.
    pub blocks: u8,
    /// 4 or 8.
    pub subbands: u8,
    pub bitpool: u8,
    pub snr: bool,
}

pub struct Encoder {
    s: Box<SbcT>,
    pub cfg: Config,
}

impl Encoder {
    pub fn new(cfg: Config) -> Self {
        let mut s = Box::new(SbcT {
            flags: 0,
            frequency: 0,
            blocks: 0,
            subbands: 0,
            mode: 0,
            allocation: 0,
            bitpool: 0,
            endian: 0,
            _priv: std::ptr::null_mut(),
            _priv_alloc_base: std::ptr::null_mut(),
        });
        unsafe { sbc_init(&mut *s, 0) };
        s.frequency = cfg.rate as u8;
        s.blocks = (cfg.blocks / 4 - 1).min(3);
        s.subbands = if cfg.subbands == 4 { 0 } else { 1 };
        s.mode = cfg.mode as u8;
        s.allocation = cfg.snr as u8;
        s.bitpool = cfg.bitpool;
        s.endian = 0;
        Self { s, cfg }
    }

    /// Input bytes (interleaved s16le stereo) consumed per frame.
    pub fn codesize(&mut self) -> usize {
        unsafe { sbc_get_codesize(&mut *self.s) }
    }

    pub fn frame_len(&mut self) -> usize {
        unsafe { sbc_get_frame_length(&mut *self.s) }
    }

    /// PCM samples per channel in one frame.
    pub fn samples_per_frame(&self) -> usize {
        self.cfg.blocks as usize * self.cfg.subbands as usize
    }

    pub fn frame_duration_us(&self) -> f64 {
        let hz = match self.cfg.rate {
            Rate::Hz44100 => 44100.0,
            Rate::Hz48000 => 48000.0,
        };
        self.samples_per_frame() as f64 / hz * 1e6
    }

    /// Encode exactly one frame from `pcm` (len == codesize) into `out`; returns bytes written.
    pub fn encode_frame(&mut self, pcm: &[u8], out: &mut [u8]) -> usize {
        let mut written: isize = 0;
        let r = unsafe { sbc_encode(&mut *self.s, pcm.as_ptr().cast(), pcm.len(), out.as_mut_ptr().cast(), out.len(), &mut written) };
        assert!(r >= 0, "sbc_encode failed: {r}");
        written as usize
    }
}

/// Reference decoder (libsbc) used by the simulator and tests to verify what a sink would play.
pub struct Decoder {
    s: Box<SbcT>,
}

impl Decoder {
    pub fn new() -> Self {
        let mut s = Box::new(SbcT {
            flags: 0,
            frequency: 0,
            blocks: 0,
            subbands: 0,
            mode: 0,
            allocation: 0,
            bitpool: 0,
            endian: 0,
            _priv: std::ptr::null_mut(),
            _priv_alloc_base: std::ptr::null_mut(),
        });
        unsafe { sbc_init(&mut *s, 0) };
        Self { s }
    }

    /// Decode one frame from the start of `input`. Returns (bytes consumed, interleaved s16le bytes written).
    pub fn decode_frame(&mut self, input: &[u8], out: &mut [u8]) -> Option<(usize, usize)> {
        let mut written = 0usize;
        let r = unsafe { sbc_decode(&mut *self.s, input.as_ptr().cast(), input.len(), out.as_mut_ptr().cast(), out.len(), &mut written) };
        if r <= 0 { None } else { Some((r as usize, written)) }
    }
}

// SAFETY: the libsbc context is a private heap allocation owned by exactly one Encoder/Decoder and
// is not tied to the creating thread, so moving it between threads is sound.
unsafe impl Send for Decoder {}
unsafe impl Send for Encoder {}

impl Drop for Decoder {
    fn drop(&mut self) {
        unsafe { sbc_finish(&mut *self.s) };
    }
}

impl Drop for Encoder {
    fn drop(&mut self) {
        unsafe { sbc_finish(&mut *self.s) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg(blocks: u8, bitpool: u8) -> Config {
        Config { rate: Rate::Hz48000, mode: Mode::JointStereo, blocks, subbands: 8, bitpool, snr: false }
    }

    #[test]
    fn frame_length_matches_a2dp_formula() {
        // A2DP joint stereo: 4 + 4*sb*ch/8 + ceil((sb + blocks*bitpool)/8)   (ch=2, sb=8)
        for (blocks, bp) in [(16u8, 53u8), (8, 53), (4, 40)] {
            let mut e = Encoder::new(cfg(blocks, bp));
            let expect = 4 + 8 + (8 + blocks as usize * bp as usize + 7) / 8;
            assert_eq!(e.frame_len(), expect, "blocks={blocks} bitpool={bp}");
            let pcm = vec![0u8; e.codesize()];
            let mut out = vec![0u8; 512];
            let n = e.encode_frame(&pcm, &mut out);
            assert_eq!(n, expect);
            assert_eq!(out[0], 0x9C, "SBC syncword");
        }
    }

    #[test]
    fn roundtrip_recovers_the_tone() {
        for (blocks, bp) in [(16u8, 53u8), (8, 53), (4, 53)] {
            let mut e = Encoder::new(cfg(blocks, bp));
            let mut d = Box::new(SbcT {
                flags: 0,
                frequency: 0,
                blocks: 0,
                subbands: 0,
                mode: 0,
                allocation: 0,
                bitpool: 0,
                endian: 0,
                _priv: std::ptr::null_mut(),
                _priv_alloc_base: std::ptr::null_mut(),
            });
            unsafe { sbc_init(&mut *d, 0) };
            let spf = e.samples_per_frame();
            let mut phase = 0.0f32;
            let (mut energy_in, mut energy_out) = (0f64, 0f64);
            let (mut g_re, mut g_im) = (0f64, 0f64);
            let mut n_out = 0usize;
            for f in 0..200 {
                let mut pcm = vec![0u8; e.codesize()];
                for s in 0..spf {
                    let v = ((phase * std::f32::consts::TAU).sin() * 6000.0) as i16;
                    phase = (phase + 440.0 / 48000.0).fract();
                    pcm[s * 4..s * 4 + 2].copy_from_slice(&v.to_le_bytes());
                    pcm[s * 4 + 2..s * 4 + 4].copy_from_slice(&v.to_le_bytes());
                    if f > 20 {
                        energy_in += (v as f64).powi(2);
                    }
                }
                let mut fr = vec![0u8; 512];
                let n = e.encode_frame(&pcm, &mut fr);
                let mut out = vec![0u8; spf * 4 + 64];
                let mut w = 0usize;
                let r = unsafe { sbc_decode(&mut *d, fr.as_ptr().cast(), n, out.as_mut_ptr().cast(), out.len(), &mut w) };
                assert!(r > 0, "decode failed {r}");
                for s in 0..w / 4 {
                    let v = i16::from_le_bytes([out[s * 4], out[s * 4 + 1]]) as f64;
                    if f > 20 {
                        energy_out += v * v;
                        let t = n_out as f64;
                        g_re += v * (t * std::f64::consts::TAU * 440.0 / 48000.0).cos();
                        g_im += v * (t * std::f64::consts::TAU * 440.0 / 48000.0).sin();
                        n_out += 1;
                    }
                }
            }
            let tone_power = 2.0 * (g_re * g_re + g_im * g_im).sqrt() / n_out as f64;
            println!(
                "blocks={blocks}: in_rms={:.0} out_rms={:.0} tone_amp@440={:.0}",
                (energy_in / n_out as f64).sqrt(),
                (energy_out / n_out as f64).sqrt(),
                tone_power
            );
            assert!(tone_power > 4500.0, "tone not recovered (amp {tone_power})");
        }
    }

    #[test]
    fn encodes_realtime_by_a_wide_margin() {
        let mut e = Encoder::new(cfg(8, 53));
        let cs = e.codesize();
        let pcm: Vec<u8> = (0..cs / 2).flat_map(|i| (((i as f32 * 0.05).sin() * 12000.0) as i16).to_le_bytes()).collect();
        let mut out = vec![0u8; 512];
        let t = std::time::Instant::now();
        let frames = 20_000;
        for _ in 0..frames {
            e.encode_frame(&pcm, &mut out);
        }
        let per = t.elapsed().as_secs_f64() / frames as f64 * 1e6;
        println!("encode: {per:.1} us/frame vs {:.0} us of audio", e.frame_duration_us());
        assert!(per < e.frame_duration_us() / 10.0);
    }

    /// Encode a stereo signal (different tone per channel) and decode it; per-channel SNR of the fit.
    fn stereo_roundtrip_snr(mode: Mode, amp: f64, bitpool: u8) -> (f64, f64) {
        let fs = 48000.0;
        let mut e = Encoder::new(Config { rate: Rate::Hz48000, mode, blocks: 8, subbands: 8, bitpool, snr: false });
        let mut d = Decoder::new();
        let spf = e.samples_per_frame();
        let (mut l_out, mut r_out) = (Vec::new(), Vec::new());
        let mut n = 0usize;
        for _ in 0..400 {
            let mut pcm = vec![0u8; e.codesize()];
            for s in 0..spf {
                let l = ((std::f64::consts::TAU * 440.0 * n as f64 / fs).sin() * amp) as i16;
                let r = ((std::f64::consts::TAU * 1500.0 * n as f64 / fs).sin() * amp) as i16;
                pcm[s * 4..s * 4 + 2].copy_from_slice(&l.to_le_bytes());
                pcm[s * 4 + 2..s * 4 + 4].copy_from_slice(&r.to_le_bytes());
                n += 1;
            }
            let mut fr = vec![0u8; 512];
            let len = e.encode_frame(&pcm, &mut fr);
            let mut out = vec![0u8; 2048];
            let (_, wrote) = d.decode_frame(&fr[..len], &mut out).expect("decode");
            for c in out[..wrote].chunks(4) {
                l_out.push(i16::from_le_bytes([c[0], c[1]]));
                r_out.push(i16::from_le_bytes([c[2], c[3]]));
            }
        }
        let skip = 4000;
        let snr = |x: &[i16], f: f64| {
            let x = &x[skip..];
            let (mut c, mut s) = (0.0, 0.0);
            for (i, &v) in x.iter().enumerate() {
                let w = std::f64::consts::TAU * f * (i + skip) as f64 / fs;
                c += v as f64 * w.cos();
                s += v as f64 * w.sin();
            }
            let (a, b) = (2.0 * c / x.len() as f64, 2.0 * s / x.len() as f64);
            let (mut sig, mut noise) = (0.0, 0.0);
            for (i, &v) in x.iter().enumerate() {
                let w = std::f64::consts::TAU * f * (i + skip) as f64 / fs;
                let fit = a * w.cos() + b * w.sin();
                sig += fit * fit;
                noise += (v as f64 - fit).powi(2);
            }
            10.0 * (sig / noise.max(1e-9)).log10()
        };
        (snr(&l_out, 440.0), snr(&r_out, 1500.0))
    }

    #[test]
    fn true_stereo_and_loud_signals_survive_the_codec() {
        for mode in [Mode::Stereo, Mode::JointStereo] {
            for amp in [6000.0, 16000.0, 24000.0, 31000.0] {
                let (l, r) = stereo_roundtrip_snr(mode, amp, 53);
                println!("{mode:?} amp {amp:>6}: L {l:5.1} dB  R {r:5.1} dB");
                assert!(l > 30.0 && r > 30.0, "{mode:?} amp {amp}: codec damaged the signal (L {l:.1} dB, R {r:.1} dB)");
            }
        }
    }
}

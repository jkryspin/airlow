//! Apple's AirPods control channel (AACP, L2CAP PSM 0x1001), reverse engineered by the LibrePods project.
//! Only what the tray needs: the handshake, notification subscription and noise control (ANC/transparency).

use std::sync::atomic::{AtomicU8, Ordering};

pub const PSM: u16 = 0x1001;

/// Must be the first packet, or the AirPods ignore everything else.
pub const HANDSHAKE: [u8; 16] = [0x00, 0x00, 0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0, 0, 0, 0, 0, 0, 0, 0];
/// Enables features Apple gates behind its own OSes (adaptive transparency, conversational awareness).
pub const FEATURES: [u8; 14] = [0x04, 0x00, 0x04, 0x00, 0x4D, 0x00, 0xFF, 0, 0, 0, 0, 0, 0, 0];
/// Subscribes to notifications (noise control mode, battery, ear detection).
pub const NOTIFY: [u8; 10] = [0x04, 0x00, 0x04, 0x00, 0x0F, 0x00, 0xFF, 0xFF, 0xFF, 0xFF];

const NOISE_HEADER: [u8; 7] = [0x04, 0x00, 0x04, 0x00, 0x09, 0x00, 0x0D];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoiseMode {
    Off = 1,
    Anc = 2,
    Transparency = 3,
    Adaptive = 4,
}

impl NoiseMode {
    pub const ALL: [NoiseMode; 4] = [NoiseMode::Off, NoiseMode::Anc, NoiseMode::Transparency, NoiseMode::Adaptive];

    pub fn from_byte(b: u8) -> Option<NoiseMode> {
        Self::ALL.into_iter().find(|m| *m as u8 == b)
    }

    pub fn label(self) -> &'static str {
        match self {
            NoiseMode::Off => "Off",
            NoiseMode::Anc => "Noise cancellation",
            NoiseMode::Transparency => "Transparency",
            NoiseMode::Adaptive => "Adaptive",
        }
    }
}

/// "Allow Off option" for the listening modes (setting 0x34 = enabled). AirPods ship with Off disabled and ignore
/// a request for it (hardware-verified) until this is set, which also adds Off to the iPhone's noise control menu.
pub const ALLOW_OFF: [u8; 11] = [0x04, 0x00, 0x04, 0x00, 0x09, 0x00, 0x34, 0x01, 0x00, 0x00, 0x00];

pub fn set_noise(m: NoiseMode) -> [u8; 11] {
    let mut p = [0u8; 11];
    p[..7].copy_from_slice(&NOISE_HEADER);
    p[7] = m as u8;
    p
}

/// The mode in a noise-control notification (also the format of the set command).
pub fn parse_noise(d: &[u8]) -> Option<NoiseMode> {
    if d.len() >= 8 && d[..7] == NOISE_HEADER { NoiseMode::from_byte(d[7]) } else { None }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Battery {
    pub percent: u8,
    pub charging: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ear {
    In,
    Out,
    Case,
}

/// What the AirPods have told us about themselves (battery of each part, ear detection).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct Pods {
    pub left: Option<Battery>,
    pub right: Option<Battery>,
    pub case: Option<Battery>,
    pub ears: Option<(Ear, Ear)>,
}

/// Battery report: `04 00 04 00 04 00 [n] ([part] 01 [percent] [status] 01) * n`, part 04 left, 02 right, 08 case,
/// status 01 charging, 02 discharging, 04 not present. Returns None if it is not a battery packet.
pub fn parse_battery(d: &[u8]) -> Option<Pods> {
    if d.len() < 7 || d[..6] != [0x04, 0x00, 0x04, 0x00, 0x04, 0x00] {
        return None;
    }
    let mut pods = Pods::default();
    for rec in d[7..].chunks_exact(5).take(d[6] as usize) {
        let (percent, status) = (rec[2], rec[3]);
        let value = (status != 0x04 && percent <= 100).then_some(Battery { percent, charging: status == 0x01 });
        match rec[0] {
            0x04 => pods.left = value,
            0x02 => pods.right = value,
            0x08 => pods.case = value,
            _ => {}
        }
    }
    Some(pods)
}

/// Ear detection: `04 00 04 00 06 00 [pod] [pod]` with 00 in ear, 01 out of ear, 02 in case.
pub fn parse_ears(d: &[u8]) -> Option<(Ear, Ear)> {
    if d.len() < 8 || d[..6] != [0x04, 0x00, 0x04, 0x00, 0x06, 0x00] {
        return None;
    }
    let ear = |b| match b {
        0 => Some(Ear::In),
        1 => Some(Ear::Out),
        2 => Some(Ear::Case),
        _ => None,
    };
    Some((ear(d[6])?, ear(d[7])?))
}

impl Pods {
    pub fn battery_text(&self) -> String {
        let part = |name: &str, b: Option<Battery>| match b {
            Some(b) => format!("{name} {}%{}", b.percent, if b.charging { "+" } else { "" }),
            None => format!("{name} --"),
        };
        format!("Battery: {} · {} · {}", part("L", self.left), part("R", self.right), part("Case", self.case))
    }

    pub fn ears_text(&self) -> String {
        let Some((a, b)) = self.ears else { return "Ears: --".into() };
        let ins = [a, b].iter().filter(|e| **e == Ear::In).count();
        let cased = [a, b].iter().filter(|e| **e == Ear::Case).count();
        let s = match (ins, cased) {
            (2, _) => "both in ears",
            (1, _) => "one in ear",
            (_, 2) => "in the case",
            _ => "out of ears",
        };
        format!("Ears: {s}")
    }
}

static PODS: std::sync::Mutex<Pods> = std::sync::Mutex::new(Pods { left: None, right: None, case: None, ears: None });

pub fn pods() -> Pods {
    *PODS.lock().unwrap_or_else(|e| e.into_inner())
}

pub(crate) fn update_battery(p: Pods) {
    let mut g = PODS.lock().unwrap_or_else(|e| e.into_inner());
    g.left = p.left;
    g.right = p.right;
    g.case = p.case;
}

pub(crate) fn update_ears(e: (Ear, Ear)) {
    PODS.lock().unwrap_or_else(|e| e.into_inner()).ears = Some(e);
}

/// Forget everything (link gone).
pub(crate) fn reset() {
    set_current(None);
    *PODS.lock().unwrap_or_else(|e| e.into_inner()) = Pods::default();
}

/// Mode last reported by the AirPods (0 = unknown / not connected).
static CURRENT: AtomicU8 = AtomicU8::new(0);
/// Mode the user asked for, not yet sent (0 = none).
static REQUEST: AtomicU8 = AtomicU8::new(0);

pub fn current() -> Option<NoiseMode> {
    NoiseMode::from_byte(CURRENT.load(Ordering::Relaxed))
}

pub(crate) fn set_current(m: Option<NoiseMode>) {
    CURRENT.store(m.map(|m| m as u8).unwrap_or(0), Ordering::Relaxed);
}

/// Ask the running session to switch mode; it is sent on the next service pass.
pub fn request(m: NoiseMode) {
    REQUEST.store(m as u8, Ordering::Relaxed);
}

pub(crate) fn take_request() -> Option<NoiseMode> {
    NoiseMode::from_byte(REQUEST.swap(0, Ordering::Relaxed))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packets_match_the_librepods_captures() {
        assert_eq!(set_noise(NoiseMode::Transparency), [0x04, 0x00, 0x04, 0x00, 0x09, 0x00, 0x0D, 0x03, 0, 0, 0]);
        assert_eq!(set_noise(NoiseMode::Anc)[7], 2);
        assert_eq!(set_noise(NoiseMode::Off)[7], 1);
        assert_eq!(set_noise(NoiseMode::Adaptive)[7], 4);
        assert_eq!(HANDSHAKE.len(), 16);
        assert_eq!(FEATURES.len(), 14);
    }

    #[test]
    fn battery_report_from_real_airpods_pro_2_parses() {
        // Captured from the hardware: left 100% discharging, right 100% discharging, case not present.
        let d = [
            0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x03, 0x04, 0x01, 0x64, 0x02, 0x01, 0x02, 0x01, 0x64, 0x02, 0x01, 0x08, 0x01, 0x00, 0x04,
            0x01,
        ];
        let p = parse_battery(&d).unwrap();
        assert_eq!(p.left, Some(Battery { percent: 100, charging: false }));
        assert_eq!(p.right, Some(Battery { percent: 100, charging: false }));
        assert_eq!(p.case, None);
        assert_eq!(p.battery_text(), "Battery: L 100% · R 100% · Case --");
        let charging = [0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x01, 0x08, 0x01, 0x32, 0x01, 0x01];
        assert_eq!(parse_battery(&charging).unwrap().case, Some(Battery { percent: 50, charging: true }));
        assert_eq!(
            parse_battery(&[0x04, 0x00, 0x04, 0x00, 0x04, 0x00, 0x03, 0x04, 0x01]).map(|p| p.left),
            Some(None),
            "truncated records are skipped"
        );
        assert_eq!(parse_battery(&[0x04, 0x00, 0x04, 0x00, 0x06, 0x00, 0, 0]), None);
    }

    #[test]
    fn ear_detection_parses_and_reads_naturally() {
        let ears = |a, b| Pods { ears: parse_ears(&[0x04, 0x00, 0x04, 0x00, 0x06, 0x00, a, b]), ..Default::default() };
        assert_eq!(ears(0, 0).ears_text(), "Ears: both in ears");
        assert_eq!(ears(0, 1).ears_text(), "Ears: one in ear");
        assert_eq!(ears(1, 1).ears_text(), "Ears: out of ears");
        assert_eq!(ears(2, 2).ears_text(), "Ears: in the case");
        assert_eq!(Pods::default().ears_text(), "Ears: --");
        assert_eq!(parse_ears(&[0x04, 0x00, 0x04, 0x00, 0x06, 0x00, 9, 0]), None);
    }

    #[test]
    fn notifications_round_trip_and_garbage_is_ignored() {
        for m in NoiseMode::ALL {
            assert_eq!(parse_noise(&set_noise(m)), Some(m));
        }
        assert_eq!(parse_noise(&[0x04, 0x00, 0x04, 0x00, 0x09, 0x00, 0x0D, 0x09, 0, 0, 0]), None, "unknown mode");
        assert_eq!(parse_noise(&[0x04, 0x00, 0x04, 0x00, 0x09, 0x00, 0x28, 0x01, 0, 0, 0]), None, "other setting");
        assert_eq!(parse_noise(&[0x04, 0x00]), None);
        assert_eq!(parse_noise(&[]), None);
    }
}

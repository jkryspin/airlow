//! Low-latency WASAPI loopback capture (IAudioClient3 shared mode) and an engine-period probe.
//! Windows only.

#![cfg(windows)]

use anyhow::Result;
use windows::Win32::Media::Audio::*;
use windows::Win32::System::Com::*;

/// `airlow audiocaps`: what shared-mode engine periods do the render endpoints offer?
/// The loopback capture can never deliver audio faster than the engine period of the endpoint it taps, so this
/// bounds how much a low-latency capture (option #5) can save, and which endpoint to capture.
pub fn probe() -> Result<()> {
    unsafe {
        let _ = CoInitializeEx(None, COINIT_MULTITHREADED);
        let enumr: IMMDeviceEnumerator = CoCreateInstance(&MMDeviceEnumerator, None, CLSCTX_ALL)?;
        let default_id =
            enumr.GetDefaultAudioEndpoint(eRender, eConsole).ok().and_then(|d| d.GetId().ok()).map(|i| i.to_string().unwrap_or_default());
        let coll = enumr.EnumAudioEndpoints(eRender, DEVICE_STATE_ACTIVE)?;
        for i in 0..coll.GetCount()? {
            let dev = coll.Item(i)?;
            let id = dev.GetId()?.to_string()?;
            let name = friendly_name(&dev).unwrap_or_else(|| "?".into());
            let is_default = default_id.as_deref() == Some(id.as_str());
            print!("{}{name}: ", if is_default { "[DEFAULT] " } else { "" });
            let client: IAudioClient3 = match dev.Activate(CLSCTX_ALL, None) {
                Ok(c) => c,
                Err(e) => {
                    println!("cannot activate ({e})");
                    continue;
                }
            };
            let fmt = client.GetMixFormat()?;
            let f = std::ptr::read_unaligned(fmt); // WAVEFORMATEX is packed: copy it, never reference its fields
            let (rate, chans) = (f.nSamplesPerSec, f.nChannels);
            let (mut def, mut fund, mut min, mut max) = (0u32, 0u32, 0u32, 0u32);
            match client.GetSharedModeEnginePeriod(fmt, &mut def, &mut fund, &mut min, &mut max) {
                Ok(()) => {
                    let ms = |fr: u32| fr as f64 / rate as f64 * 1000.0;
                    println!(
                        "{rate} Hz {chans} ch | period default {def} fr ({:.2} ms), MIN {min} fr ({:.2} ms), max {max} fr ({:.2} ms){}",
                        ms(def),
                        ms(min),
                        ms(max),
                        if min < def { "   <-- shorter period available" } else { "" }
                    );
                }
                Err(e) => println!("{rate} Hz {chans} ch | engine period unavailable ({e})"),
            }
            CoTaskMemFree(Some(fmt as *const _));
        }
    }
    Ok(())
}

/// The endpoint's friendly name (e.g. "Speakers (Realtek(R) Audio)").
unsafe fn friendly_name(dev: &IMMDevice) -> Option<String> {
    use windows::Win32::Foundation::PROPERTYKEY;
    let store = unsafe { dev.OpenPropertyStore(STGM_READ).ok()? };
    // PKEY_Device_FriendlyName = {a45c254e-df1c-4efd-8020-67d146a850e0}, 14
    let key = PROPERTYKEY { fmtid: windows::core::GUID::from_u128(0xa45c254e_df1c_4efd_8020_67d146a850e0), pid: 14 };
    let v = unsafe { store.GetValue(&key).ok()? };
    let s = unsafe { v.Anonymous.Anonymous.Anonymous.pwszVal.to_string().ok() };
    s
}

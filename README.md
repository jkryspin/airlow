# airlow

Low-latency Bluetooth A2DP audio for Windows, built for competitive gaming. First target: **AirPods Pro 2**.

airlow is a **user-mode Bluetooth stack** (HCI, L2CAP, SDP, AVDTP, AVRCP, A2DP/SBC) that talks straight to a USB
Bluetooth controller over WinUSB. Windows' own stack is bypassed for that one radio, which makes the latency levers
tunable: ACL flush timeout, SBC frame layout and packet size, no sniff mode, and sending each packet the moment it is
ready.

## Status
Working on a MediaTek MT79xx (USB `13D3:3602`) with AirPods Pro 2: pairing (SSP), encrypted reconnect, AVDTP/SBC
negotiation, and live Windows audio (WASAPI loopback) that sounds clean. In a click test against speakers the AirPods
were only tens of milliseconds behind (judged by ear, not measured; the AirPods do not report their buffer depth).

**Experimental.** One controller and one pair of earbuds have been tested. The radio is unavailable to Windows while it
is bound to airlow (see [docs/setup.md](docs/setup.md); it is reversible).

## Usage
```
cargo build --release
airlow pair live        # AirPods in pairing mode; streams Windows system audio (loopback) for 120 s
airlow pair tone        # pairs and plays a test tone
airlow live             # reconnect with the saved key and stream (the AirPods must be connectable)
airlow captest          # no Bluetooth: checks WASAPI loopback capture on this PC
```
Options for `live`/`tone`: `[blocks=16] [frames_per_packet] [flush_ms=40] [bitpool=53] [seconds]`.
`AIRLOW_USB=vvvv:pppp` selects another WinUSB-bound controller. `AIRLOW_TRACE=1` prints raw HCI/ACL traffic.
The pairing key is stored in `%APPDATA%\airlow\keys.txt`.

## Everyday use: the tray app (no terminal)
`airlow-tray.exe` runs in the background with a tray icon and no console window.
1. One-time: bind your USB Bluetooth controller to WinUSB ([docs/setup.md](docs/setup.md)).
2. Run `airlow-tray.exe`. In the tray menu choose **Pair AirPods...** and put the AirPods in pairing mode (case open, hold the
   back button until the light flashes white). Tick **Start with Windows** to launch it at login.
3. In Windows *Sound settings* pick an output device that exists only to be captured, by default
   **Speakers (Steam Streaming Speakers)** (installed with Steam; no kernel driver of ours is involved, so anti-cheat and
   Memory Integrity are unaffected). Any virtual output works: set `capture_device` in the settings file.
4. From then on, opening the case or putting the AirPods in connects them automatically (airlow accepts their reconnect and
   pages them if they do not) and whatever Windows plays on that output is streamed to them.

The tray menu also shows battery (left, right, case) and ear detection, and has a **Noise control** submenu (Off, Noise cancellation, Transparency, Adaptive). These use the AirPods' private control channel (AACP, L2CAP PSM 0x1001; protocol from the [LibrePods](https://github.com/kavishdevar/librepods) project), opened a few seconds after audio starts because opening it earlier makes the AirPods abandon the audio setup. Choosing Off also enables the AirPods' "Allow Off option" setting, which they require before they accept Off.

Always end a session with a proper HCI Disconnect (the tray does, including on Quit): resetting the controller instead leaves the AirPods thinking the old link is alive, and they then refuse the next session's channels in an endless reconnect loop.

Icon colour: grey = needs action, amber = waiting/connecting, green = streaming, red = error (hover or open the menu for text).
Settings (`%APPDATA%\airlow\config.txt`, created on first use): `capture_device`, `volume` (the AirPods' initial volume, 0-127;
Windows' own volume slider does **not** reach them because loopback is captured before it), `codec` (`sbc` or `aac`).
The log is `%APPDATA%\airlow\airlow.log`. Tray latency is the same as `airlow live`: the Bluetooth-side tuning applies, and
capture still happens at the endpoint's 10 ms engine period.

## Things we learned the hard way (all verified on hardware)
* **Attach the Wi-Fi/Bluetooth antenna.** On a combined Wi-Fi/Bluetooth board module the antenna carries Bluetooth too;
  without it ~78% of packets were lost.
* **AirPods Pro 2 crackle badly on 8-block SBC and are perfectly clean on 16-block frames.** 16 blocks is the default.
* **Send media immediately after AVDTP Start** (and set AVRCP volume *before* it). Dead air makes the AirPods send an
  AVRCP PAUSE.
* **Answer everything the sink asks:** SDP (publish an A2DP Source record), AVRCP capabilities/notifications.
* **AirPods reject AVDTP Reconfigure**; change settings by Suspend/Close/reopen.
* The controller completes only ~300-360 ACL packets/s regardless of size, so packets are sized adaptively. A sink fed
  slower than real time starves and outputs nothing.

## Optional AAC mode
SBC is the default. AirPods are reported to use a smaller playback buffer for AAC than for SBC, so there is an
optional AAC mode (Windows' built-in AAC-LC encoder through Media Foundation; no third-party codec). It costs about
**70 ms of fixed encoder delay** on our side (measured), so it only wins if the AirPods' AAC buffer is more than that
much smaller. **Measured by ear on AirPods Pro 2 (SBC/AAC/SBC/AAC click test against speakers): SBC was faster, so SBC is the default and AAC is kept as an option.** To try it yourself:
```
set AIRLOW_CODEC=aac && airlow pair live          # AAC (AIRLOW_AAC_KBPS=96|128|160|192, default 160)
set AIRLOW_SWEEP=codec && airlow pair live        # one session: SBC, AAC, SBC, AAC (15 s each) with audible gap markers
```
The Windows encoder's delay comes from AAC-LC's 1024-sample blocks plus a block of lookahead (the transform windows overlap) and some internal buffering. The AirPods advertise only AAC-LC over A2DP (object types 0xC0), so lower-delay variants such as AAC-ELD are not available; a different LC encoder might save 20-25 ms at most.

## Roadmap: further latency options (researched, not built)
Where the delay is: roughly 15 ms is ours (Windows' 10 ms capture period, packetising, USB, radio) and most of the rest is
the AirPods' own playback buffer, which does not respond to how we send audio (tested) and has no documented control: a
search of the LibrePods/ntpods AirPods protocol code finds nothing latency-related, MagicPods' "Game Mode" is documented
only for counterfeit Airoha-chip AirPods, and Apple's own macOS Game Mode works on the *source* side (dropping queued audio).

* **#5, low-latency WASAPI capture (IAudioClient3 shared mode): ruled out on this machine.** Loopback capture sees audio at the
  engine period of the endpoint it taps. `airlow audiocaps` lists every output's shared-mode period range; on the test PC all
  five (Realtek speakers, USB speakers, monitor audio, digital out, Steam Streaming Speakers) are fixed at 480 frames = 10 ms
  (minimum = maximum = default), so a shorter capture period is simply not offered. It would help on a machine whose audio
  driver advertises smaller periods.
* **#4, a virtual audio device (kernel driver) with a short engine period: the only way past that 10 ms.** A virtual render
  endpoint can advertise e.g. 128-frame (2.7 ms) periods, which removes most of the capture quantisation and the loopback
  hop: an estimated saving of about 5-10 ms. Cost and risk: it needs the Windows Driver Kit, a signed driver (test-signing
  mode and a reboot for development; Secure Boot must allow it), and an ACX/WaveRT audio driver (the ntpods project already
  does this with Microsoft's ACX `AudioCodec` sample, which is the natural starting point). Worth it only if every
  millisecond counts: the gain is small next to the AirPods' own buffer.
* **HFP/SCO (the phone-call profile)** has far smaller buffers (an Apple developer measured about 9 ms reported output latency vs
  163 ms for A2DP, ~30 ms better in practice), but it is mono 16 kHz, so no stereo positioning, and needs RFCOMM/HFP call
  setup plus isochronous USB audio, which this controller may route outside USB.
* **Probe the AirPods' private AACP channel** (L2CAP PSM 0x1001, handshake and "set feature flags" packet known from LibrePods)
  for an undocumented latency capability. No one has found one; it needs a latency meter to evaluate.
## Testing
A hardware-free simulation suite (91 tests) covers the whole stack against a simulated controller and a strict,
AirPods-like sink; see [docs/TESTING.md](docs/TESTING.md).

## Limits
* SBC by default; AAC-LC is optional and adds ~70 ms of encoder delay (see above). No AAC-ELD or other low-delay codec yet.
* The AirPods' own playout buffer is outside our control and bounds the total latency.
* Audio comes from WASAPI loopback; a virtual audio endpoint would remove Windows' mixer delay.
* Only tested with one controller. Controllers that need vendor firmware download are not handled.
* Windows only.

## License
LGPL-2.1-or-later (the vendored SBC codec, libsbc, is LGPL-2.1+; see `vendor/sbc`). Full text in `LICENSE`.

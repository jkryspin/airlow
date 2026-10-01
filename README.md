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
much smaller. Whether it does is exactly what the A/B is for:
```
set AIRLOW_CODEC=aac && airlow pair live          # AAC (AIRLOW_AAC_KBPS=96|128|160|192, default 160)
set AIRLOW_SWEEP=codec && airlow pair live        # one session: SBC, AAC, SBC, AAC (15 s each) with audible gap markers
```
If AAC helps, the follow-up is a lower-delay encoder (AAC-ELD is ~15 ms).

## Testing
A hardware-free simulation suite (53 tests) covers the whole stack against a simulated controller and a strict,
AirPods-like sink; see [docs/TESTING.md](docs/TESTING.md).

## Limits
* SBC only (AirPods also offer AAC, which no free low-delay encoder covers here).
* The AirPods' own playout buffer is outside our control and bounds the total latency.
* Audio comes from WASAPI loopback; a virtual audio endpoint would remove Windows' mixer delay.
* Only tested with one controller. Controllers that need vendor firmware download are not handled.
* Windows only.

## License
LGPL-2.1-or-later (the vendored SBC codec, libsbc, is LGPL-2.1+; see `vendor/sbc`). Full text in `LICENSE`.

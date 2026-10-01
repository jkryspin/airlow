# Setup (Windows)

airlow replaces Windows' Bluetooth stack for ONE controller with a user-mode stack, so that radio
is unavailable to Windows while bound. Fully reversible.

## Bind the controller to WinUSB
1. Install Zadig: `winget install akeo.ie.Zadig`
2. Run Zadig as admin -> Options -> List All Devices.
3. Select the Bluetooth *interface 0* entry (e.g. "MediaTek Bluetooth Adapter (Interface 0)", VID 13D3 PID 3602).
4. Driver: WinUSB -> Replace Driver.
5. `cargo run` should print controller version / BD_ADDR.

## Revert
Device Manager -> the device -> Update driver -> Browse -> "Let me pick" -> MediaTek Bluetooth Adapter,
or `pnputil /remove-device` then `pnputil /scan-devices`.

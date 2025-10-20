# ESP32 Wi-Fi RGB LED

## Overview
This PlatformIO project targets the ESP32-S3-DevKitM-1 and exposes a TCP control service for a single WS2812-compatible RGB LED wired to GPIO 38 (override with `-D RGB_DATA_PIN=<pin>` in `platformio.ini`). The firmware connects to the configured network, publishes itself as `esp32s3-rgb.local` via mDNS, and accepts line-oriented commands such as `SET 255 0 0` or `OFF` over `TCP_PORT` (12345 by default). A Python helper script is included for setting colors, querying state, or experimenting interactively from a workstation.

## Requirements
- ESP32-S3-DevKitM-1 (or equivalent board defined in `platformio.ini`).
- PlatformIO Core (`pio`) 6.0+ with the Espressif32 platform 6.7.0 or newer.
- Python 3.8+ on the host for `host.py`.
- Local Wi-Fi network credentials (2.4 GHz recommended for stability).

## Configure Wi-Fi credentials
1. Copy `src/secrets.example.h` to `src/secrets.h` and replace `WIFI_SSID` / `WIFI_PASS` with your test network.
2. Avoid committing real credentials—`src/secrets.h` is git-ignored for safety, keep placeholders in version control and document overrides separately.
3. If you rotate settings, rebuild before flashing to ensure the new values are linked in.

## Build and flash
All commands below run from the project root.

```bash
pio run                     # compile firmware and surface warnings
pio run -t upload           # flash the board over USB
pio device monitor -b 115200  # view serial logs
```

The board boots at 115200 baud, logs connection progress, and restarts if Wi-Fi association takes longer than 15 seconds. `platformio.ini` applies `CORE_DEBUG_LEVEL=3` so you can adjust verbosity by editing `build_flags` if needed.

## Host-side LED controller
`host.py` sends TCP commands directly to the board:

```bash
python3 host.py --host esp32s3-rgb.local set 255 0 0   # solid red
python3 host.py --host esp32s3-rgb.local off           # LED off
python3 host.py --host esp32s3-rgb.local get           # read current color
python3 host.py --host esp32s3-rgb.local repl          # interactive prompt
```

Key notes:
- `--port` overrides the default when `TCP_PORT` changes.
- Colors may be provided in decimal (`set 128 64 32`). Hex literals such as `0xFF` are also accepted per channel.
- The device responds with `OK <r> <g> <b>`, `COLOR <r> <g> <b>`, `OK OFF`, or `ERR ...` depending on the command.
- If your board gates NeoPixel power (e.g., ESP32-S3-DevKitC-1), set `-D RGB_POWER_PIN=<gpio>` in `platformio.ini` so firmware drives it high during `setup()`.

## Testing
Unity tests live under `test/` and run on-device or in the PlatformIO simulator:

```bash
pio test -e esp32-s3-devkitm-1
```

Add cases that cover Wi-Fi setup, command parsing, and LED state handling before changing protocol behavior. Document any hardware fixtures or host scripts in this README or `test/` notes so CI can mirror your setup.

## Configuration reference
- `platformio.ini` is the single source for board selection, upload speed, debug level, and `TCP_PORT`.
- Adjust serial and upload speeds there instead of modifying source files.
- The firmware enables `WiFi.setSleep(false)` to minimize latency—revert if you need lower power consumption.

## Troubleshooting
- **Cannot resolve `esp32s3-rgb.local`**: ensure your workstation supports mDNS/Bonjour or connect via the printed IP address instead.
- **Commands rejected**: confirm the TCP port matches between firmware (`TCP_PORT`) and `host.py`. Watch the serial console for `ERR` messages indicating malformed lines.
- **Frequent restarts**: weak Wi-Fi RSSI forces the watchdog to reboot after 15 seconds; try moving closer to the access point or updating credentials.

## Useful references
- PlatformIO ESP32 unit testing docs: <https://docs.platformio.org/en/latest/advanced/unit-testing/index.html>
- Arduino Wi-Fi library: <https://github.com/espressif/arduino-esp32>

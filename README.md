# ESP32 Wi-Fi Echo

## Overview
This PlatformIO project targets the ESP32-S3-DevKitM-1 and hosts a TCP echo service over Wi-Fi. The Arduino firmware connects to the configured network, advertises itself as `esp32s3-echo.local` via mDNS, and mirrors all TCP payloads it receives on `TCP_PORT` (12345 by default). The repository also ships a lightweight Python helper that fronts `nc`/`ncat` for interactive echo checks and throughput benchmarking.

## Requirements
- ESP32-S3-DevKitM-1 (or equivalent board defined in `platformio.ini`).
- PlatformIO Core (`pio`) 6.0+ with the Espressif32 platform 6.7.0 or newer.
- Python 3.8+ on the host for `host.py`, plus an `nc`/`ncat` binary in `PATH`.
- Local Wi-Fi network credentials (2.4 GHz recommended for stability).

## Configure Wi-Fi credentials
1. Open `src/secrets.h` and replace `WIFI_SSID` / `WIFI_PASS` with your test network.
2. Avoid committing real credentials—keep placeholders in version control and document overrides separately.
3. If you rotate settings, rebuild before flashing to ensure the new values are linked in.

## Build and flash
All commands below run from the project root.

```bash
pio run                     # compile firmware and surface warnings
pio run -t upload           # flash the board over USB
pio device monitor -b 115200  # view serial logs
```

The board boots at 115200 baud, logs connection progress, and restarts if Wi-Fi association takes longer than 15 seconds. `platformio.ini` applies `CORE_DEBUG_LEVEL=3` so you can adjust verbosity by editing `build_flags` if needed.

## Host-side echo client
`host.py` wraps `nc` for convenience:

```bash
python3 host.py --host esp32s3-echo.local       # interactive echo session
python3 host.py --host esp32s3-echo.local --bench --size-mb 8  # throughput test
```

Key notes:
- `--port` overrides the default when `TCP_PORT` changes.
- `--nc` selects an alternate netcat binary (e.g., `ncat` on Windows Subsystem for Linux).
- Benchmarks stream random payloads in `chunk`-sized writes (default 1460 bytes) and report goodput in MiB/s.

## Testing
Unity tests live under `test/` and run on-device or in the PlatformIO simulator:

```bash
pio test -e esp32-s3-devkitm-1
```

Add cases that cover Wi-Fi setup, echo integrity, and timeout handling before changing protocol behavior. Document any hardware fixtures or host scripts in this README or `test/` notes so CI can mirror your setup.

## Configuration reference
- `platformio.ini` is the single source for board selection, upload speed, debug level, and `TCP_PORT`.
- Adjust serial and upload speeds there instead of modifying source files.
- The firmware enables `WiFi.setSleep(false)` to minimize latency—revert if you need lower power consumption.

## Troubleshooting
- **Cannot resolve `esp32s3-echo.local`**: ensure your workstation supports mDNS/Bonjour or connect via the printed IP address instead.
- **No echo traffic**: confirm the TCP port matches between firmware (`TCP_PORT`) and `host.py`, and check that `nc` is installed.
- **Frequent restarts**: weak Wi-Fi RSSI forces the watchdog to reboot after 15 seconds; try moving closer to the access point or updating credentials.

## Useful references
- PlatformIO ESP32 unit testing docs: <https://docs.platformio.org/en/latest/advanced/unit-testing/index.html>
- Arduino Wi-Fi library: <https://github.com/espressif/arduino-esp32>

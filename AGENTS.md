# Repository Guidelines

## Project Structure & Module Organization
Firmware sits in `src/`: `main.cpp` drives the Wi-Fi echo loop and `secrets.h` holds per-device credentials. Stage shared headers in `include/`, reusable drivers in `lib/`, and hardware-facing tests in `test/`. All build flags and the `esp32-s3-devkitm-1` environment live in `platformio.ini`. Host-side tooling, including the interactive echo client and throughput benchmark, resides in `host.py`; update it alongside protocol changes.

## Build, Test, and Development Commands
- `pio run` — compile the default environment and surface compiler warnings.
- `pio run -t upload` — flash the board over USB using the configured baud rate.
- `pio device monitor -b 115200` — open a serial console for log inspection.
- `pio test -e esp32-s3-devkitm-1` — run Unity tests under `test/` on hardware or the simulator.
- `python3 host.py --host esp32s3-echo.local [--bench]` — verify TCP echo behavior or measure throughput.

## Coding Style & Naming Conventions
Match the existing Arduino C++ style: two-space indentation, braces on the same line, and `snake_case` helpers such as `connect_wifi`. Use `UPPER_CASE` macros only for PlatformIO tunables (`TCP_PORT`) and prefer `constexpr` constants elsewhere. Keep includes ordered as standard, framework, then project headers, and retain concise `//` comments for behavior notes instead of restating code.

## Testing Guidelines
Author Unity tests per feature (e.g., `test_tcp_echo.cpp`) and keep fixtures deterministic so they pass on CI and on-device. Exercise connection setup, echo integrity, and timeout paths before merging. Run `pio test` after any change to networking, timing, or credentials handling, and document test expectations in the local README when special hardware or scripts are required.

## Commit & Pull Request Guidelines
Write commit subjects in the imperative mood (`echo: tighten timeout`) with optional bodies covering rationale or fallout. Group related firmware, config, and host-tool edits, and never bundle credentials or IDE artifacts. Pull requests should describe the motivation, list validation steps (`pio run`, `pio test`, host bench output), and link tracking issues; attach serial logs or screenshots whenever user-visible behavior shifts.

## Security & Configuration Tips
Do not commit real Wi-Fi secrets—check in placeholders in `src/secrets.h` and explain overrides in deployment notes. Treat `platformio.ini` as the source of truth for ports, debug levels, and mDNS names; update flags in tandem with code so teammates can reproduce builds.

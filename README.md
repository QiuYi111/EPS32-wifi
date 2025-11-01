# ESP32-S3 Unitree LiDAR Bridge

This PlatformIO project turns an ESP32-S3-DevKitM (default target: `esp32-s3-devkitc-1`) into a UART-to-Wi-Fi bridge for Unitree LiDAR L1 data. The firmware:
- opens a high-speed UART to the LiDAR (default 2 Mbps on `Serial1`)
- parses MAVLink frames to expose IMU and point-cloud streams through `UnitreeMavlinkParser`/`LidarPipeline`
- drives LiDAR control commands (work mode, LED ring, flash save, reboot) via the new `UnitreeMavlinkController`

All Unitree protocol headers and binaries live under `include/unitree_lidar_sdk/`; they come straight from Unitree’s official SDK so we stay bug-compatible with upstream firmware.

## Hardware Checklist
- ESP32-S3-DevKitC-1 (or any Arduino-compatible ESP32-S3 board)
- Unitree LiDAR L1 with UART cable
- 5 V power budget ≥ 1.5 A (LiDAR spin-up current)
- UART wiring (default pins can be overridden through `UNITREE_RX_PIN` / `UNITREE_TX_PIN` build flags):
  - ESP32 RX ← LiDAR TX
  - ESP32 TX → LiDAR RX
  - Common GND
  - Leave LiDAR’s USB-C connected for power/firmware when needed, but use only one data path at a time

## Repository Layout
- `src/main.cpp` — minimal firmware: sets up the UART, parses MAVLink, prints IMU + point-cloud hints, and forces the LiDAR into NORMAL mode at boot.
- `lib/unitree_mavlink/` — ESP32-native helpers:
  - `UnitreeMavlink.h/.cpp` — byte-by-byte MAVLink parser + point cloud assembler
  - `UnitreeMavlinkControl.h/.cpp` — high-level control wrapper around Unitree’s MAVLink commands
- `include/unitree_lidar_sdk/` — vendor SDK (headers, prebuilt libs, docs, examples)
- `examples/unitree_mavlink_demo/` — standalone Arduino sketch showing how to log LiDAR data
- `host.py` — legacy TCP echo helper; update when exposing LiDAR data over Wi-Fi

## Introducing `UnitreeMavlinkController`
`UnitreeMavlinkController` lives next to the parser and removes the need to handcraft MAVLink commands on the ESP32:

```cpp
#include "UnitreeMavlinkControl.h"

HardwareSerial &lidar_serial = Serial1;
UnitreeMavlinkController controller(lidar_serial);

void setup() {
  lidar_serial.begin(2000000, SERIAL_8N1, UNITREE_RX_PIN, UNITREE_TX_PIN);

  controller.set_work_mode(UnitreeMavlinkController::WorkMode::kNormal);
  controller.set_led_pattern(UnitreeMavlinkController::LedPattern::kSixStageBreathing);

  UnitreeMavlinkController::LedTable table{};
  table.fill(0xFF);
  controller.set_led_table(table);  // command-mode full ring on
}
```

Available helpers:
- `set_work_mode(WorkMode::kNormal / kStandby / kRaw)`  
- `set_led_pattern(...)` (built-in animations)  
- `set_led_table(...)` (45-byte custom bitmap, command mode)  
- `save_configuration()` (Unitree flash save)  
- `reboot_device()` (soft reboot)

Each call serializes the proper MAVLink packet and pushes it onto the bound `Stream`. No extra buffers or RTOS tasks required.

## Building and Flashing
```bash
pio run                     # build firmware
pio run -t upload           # flash via USB
pio device monitor -b 115200  # view logs
```

Boot messages include `[Unitree Demo] booting` and IMU prints once MAVLink traffic flows. If the monitor stays silent, press the EN button, double-check UART wiring, and keep `Serial.begin(115200)` attached to USB CDC (`ARDUINO_USB_CDC_ON_BOOT=1` is already set).

## Quick Serial Sanity Checklist
1. Connect LiDAR → ESP32 UART, power everything.
2. Open `pio device monitor`.
3. Reset the ESP32. Expected timeline:
   - `[Unitree Demo] booting`
   - `[Unitree Demo] UART ready (baud=2000000, RX=18, TX=17)`
   - `Unitree` mode command acknowledged (no serial ack; check LiDAR behavior)
   - IMU packets stream at ~200 Hz (`[IMU] packet=...`)
4. If the LiDAR keeps spinning but no data shows up:
   - confirm baud rate (early firmware shipped at 921 600)
   - ensure LiDAR isn’t still opened over USB by a desktop program
   - verify common ground

## Wi-Fi & Future Work
`host.py` currently targets the legacy echo firmware. Rework it to publish LiDAR data (e.g., UDP, TCP, or WebSocket) once the ESP32 side exports the necessary buffers. If you add network features, remember:
- keep `src/secrets.h` ignored; clone from `src/secrets.example.h`
- document new TCP/UDP ports in this README and `platformio.ini`
- add Unity tests under `test/` for any protocol changes (`pio test -e esp32-s3-devkitc-1`)

## Upstream SDK Notes
- The headers under `include/unitree_lidar_sdk/include/mavlink/` define every MAVLink frame Unitree devices speak (`CONFIG_LIDAR_WORKING_MODE`, `CONFIG_LED_RING_TABLE_PACKET`, etc.).
- The vendor examples (`include/unitree_lidar_sdk/examples/`) show desktop usage. Use them when sanity-checking ESP32 behavior against a PC.
- Version tracking lives in `include/unitree_lidar_sdk/include/unitree_lidar_sdk_config.h` (`1.0.16`). Update this folder wholesale if you take a newer SDK drop.

## Troubleshooting (Hard Truths)
- LiDAR spins but no packets: UART wires wrong or missing ground.
- LiDAR never leaves standby: make sure `set_work_mode()` runs after serial init and that the pattern commands aren’t stuck in command-table mode.
- ESP32 brownouts when LiDAR starts: supply can’t handle inrush—use a separate 5 V rail or beefier regulator.
- Still lost? Capture raw UART bytes with a logic analyzer and compare against Unitree’s PC SDK. The protocol isn’t magic—if the frames aren’t there, you’re not sending them.


# Unitree MAVLink Helpers

ESP32-oriented helpers for decoding Unitree LiDAR/IMU MAVLink streams.

- Minimal usage demo: `examples/unitree_mavlink_demo/unitree_mavlink_demo.cpp`
- Control helper for ESP32 firmware: see `UnitreeMavlinkController` in `UnitreeMavlinkControl.h`

## Usage

```cpp
#include "UnitreeMavlink.h"
#include "UnitreeMavlinkControl.h"

using namespace unitree::mav;

HardwareSerial &serial = Serial1;
UnitreeMavlinkParser parser;
LidarPipeline pipeline;
UnitreeMavlinkController controller(serial);

void setup() {
  serial.begin(921600, SERIAL_8N1, RX_PIN, TX_PIN);

  parser.set_imu_callback([](const ImuSample &imu) {
    // consume imu data
  });

  parser.set_lidar_aux_callback([&](const LidarAuxPacket &aux) {
    pipeline.handle_auxiliary(aux);
  });

  parser.set_lidar_distance_callback([&](const LidarDistancePacket &dist) {
    pipeline.handle_distance(dist);
  });

  pipeline.set_cloud_callback([](uint16_t packet_id, const std::vector<PointXYZI> &cloud) {
    // point cloud ready
  });

  controller.set_work_mode(UnitreeMavlinkController::WorkMode::kNormal);
}

void loop() {
  parser.poll(serial);
}
```

Update `Calibration` and callbacks to match your application.

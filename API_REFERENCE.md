# ESP32 Unitree LiDAR API Reference

## Overview

This document provides comprehensive API documentation for the ESP32 Unitree LiDAR bridge libraries. The API is organized into three main components: MAVLink parsing, LiDAR control, and data processing pipelines.

## Core Namespaces

```cpp
namespace unitree::mav {
    // MAVLink parsing and control
}
```

## Data Structures

### ImuSample
IMU sensor data structure containing orientation and motion information.

```cpp
struct ImuSample {
    uint16_t packet_id = 0;                    // Sequential packet identifier
    std::array<float, 4> quaternion{};        // Orientation quaternion (W, X, Y, Z)
    std::array<float, 3> angular_velocity{};   // Angular velocity in rad/s
    std::array<float, 3> linear_acceleration{}; // Linear acceleration in m/s²
};
```

**Usage Example:**
```cpp
parser.set_imu_callback([](const ImuSample &imu) {
    Serial.printf("IMU Quat: %.3f, %.3f, %.3f, %.3f\n",
                  imu.quaternion[0], imu.quaternion[1],
                  imu.quaternion[2], imu.quaternion[3]);
});
```

### LidarAuxPacket
Auxiliary LiDAR data containing metadata and operational parameters.

```cpp
struct LidarAuxPacket {
    uint16_t packet_id = 0;
    uint16_t payload_size = 0;
    uint8_t lidar_work_status = 0;
    uint32_t lidar_sync_delay_time = 0;
    uint32_t time_stamp_s_step = 0;
    uint32_t time_stamp_us_step = 0;
    uint32_t sys_rotation_period = 0;
    uint32_t com_rotation_period = 0;
    float com_horizontal_angle_start = 0.f;
    float com_horizontal_angle_step = 0.f;
    float sys_vertical_angle_start = 0.f;
    float sys_vertical_angle_span = 0.f;
    float apd_temperature = 0.f;
    float dirty_index = 0.f;
    float imu_temperature = 0.f;
    float up_optical_q = 0.f;
    float down_optical_q = 0.f;
    float apd_voltage = 0.f;
    float imu_angle_x_offset = 0.f;
    float imu_angle_y_offset = 0.f;
    float imu_angle_z_offset = 0.f;
    float b_axis_dist = 0.f;
    float theta_angle = 0.f;
    float ksi_angle = 0.f;
    std::array<uint8_t, 120> reflect_data{};
};
```

### LidarDistancePacket
Distance measurement data from LiDAR sensors.

```cpp
struct LidarDistancePacket {
    uint16_t packet_id = 0;
    uint16_t payload_size = 0;
    std::array<uint8_t, 240> distance_data{};
};
```

### PointXYZI
3D point cloud data structure with intensity information.

```cpp
struct PointXYZI {
    float x, y, z;           // Cartesian coordinates in meters
    float intensity;         // Reflectivity value (0.0 - 1.0)
    uint16_t ring;          // LiDAR ring identifier
    uint32_t timestamp;     // Relative timestamp
};
```

## UnitreeMavlinkParser Class

### Overview
Byte-by-byte MAVLink frame parser for Unitree LiDAR data streams.

### Constructor
```cpp
UnitreeMavlinkParser();
```

### Methods

#### set_imu_callback
Sets callback function for IMU data processing.

```cpp
void set_imu_callback(std::function<void(const ImuSample&)> callback);
```

**Parameters:**
- `callback`: Function to process IMU samples

**Example:**
```cpp
parser.set_imu_callback([](const ImuSample& imu) {
    process_imu_data(imu);
});
```

#### set_lidar_aux_callback
Sets callback function for auxiliary LiDAR data.

```cpp
void set_lidar_aux_callback(std::function<void(const LidarAuxPacket&)> callback);
```

#### set_lidar_distance_callback
Sets callback function for distance measurement data.

```cpp
void set_lidar_distance_callback(std::function<void(const LidarDistancePacket&)> callback);
```

#### poll
Processes incoming serial data and triggers appropriate callbacks.

```cpp
void poll(Stream& serial);
```

**Parameters:**
- `serial`: Serial stream to read from

**Usage:**
```cpp
void loop() {
    parser.poll(Serial1);
}
```

## LidarPipeline Class

### Overview
Point cloud assembly and processing pipeline for LiDAR data.

### Constructor
```cpp
LidarPipeline();
```

### Methods

#### handle_auxiliary
Processes auxiliary LiDAR data packet.

```cpp
void handle_auxiliary(const LidarAuxPacket& aux);
```

#### handle_distance
Processes distance measurement packet.

```cpp
void handle_distance(const LidarDistancePacket& dist);
```

#### set_cloud_callback
Sets callback for completed point clouds.

```cpp
void set_cloud_callback(std::function<void(uint16_t, const std::vector<PointXYZI>&)> callback);
```

**Parameters:**
- `callback`: Function receiving packet ID and point cloud vector

**Example:**
```cpp
pipeline.set_cloud_callback([](uint16_t packet_id, const std::vector<PointXYZI>& cloud) {
    Serial.printf("Point cloud %d: %d points\n", packet_id, cloud.size());
    for (const auto& point : cloud) {
        process_point(point);
    }
});
```

## UnitreeMavlinkController Class

### Overview
High-level control interface for Unitree LiDAR devices.

### Constructor
```cpp
explicit UnitreeMavlinkController(Stream& stream,
                                  uint8_t system_id = 1,
                                  uint8_t component_id = 200);
```

**Parameters:**
- `stream`: Serial stream for communication
- `system_id`: MAVLink system ID (default: 1)
- `component_id`: MAVLink component ID (default: 200)

### Enumerations

#### WorkMode
LiDAR operational modes.

```cpp
enum class WorkMode : uint8_t {
    kNormal = NORMAL_MODE,      // Standard operation
    kStandby = STANDBY_MODE,    // Low power mode
    kRaw = RAW_MODE            // Raw data output
};
```

#### LedPattern
Built-in LED animation patterns.

```cpp
enum class LedPattern : uint8_t {
    kCommandTable = LED_RING_COMMAND_MODE,
    kForwardSlow = LED_RING_FUN_FORWARD_SLOW_MODE,
    kForwardFast = LED_RING_FUN_FORWARD_FAST_MODE,
    kReverseSlow = LED_RING_FUN_REVERSE_SLOW_MODE,
    kReverseFast = LED_RING_FUN_REVERSE_FAST_MODE,
    kTripleFlip = LED_RING_FUN_TRIPLE_FLIP_MODE,
    kTripleBreathing = LED_RING_FUN_TRIPLE_BREATHING_MODE,
    kSixStageBreathing = LED_RING_FUN_SIXSTAGE_BREATHING_MODE
};
```

### Methods

#### bind_stream
Rebinds the controller to a different serial stream.

```cpp
void bind_stream(Stream& stream);
```

#### set_work_mode
Sets LiDAR operational mode.

```cpp
bool set_work_mode(WorkMode mode);
```

**Returns:** `true` if command sent successfully

**Example:**
```cpp
controller.set_work_mode(UnitreeMavlinkController::WorkMode::kNormal);
```

#### set_led_pattern
Configures LED ring animation pattern.

```cpp
bool set_led_pattern(LedPattern pattern,
                     uint32_t rotation_period_us = kDefaultLedRotationPeriodUs,
                     uint32_t zero_point_offset_us = 0);
```

**Parameters:**
- `pattern`: LED animation pattern
- `rotation_period_us`: Rotation period in microseconds
- `zero_point_offset_us`: Zero point offset in microseconds

#### set_led_table
Sets custom LED bitmap for command mode.

```cpp
bool set_led_table(const LedTable& table,
                   uint32_t rotation_period_us = kDefaultLedRotationPeriodUs,
                   uint32_t zero_point_offset_us = 0,
                   LedPattern display_mode = LedPattern::kCommandTable);
```

**Parameters:**
- `table`: 45-byte LED bitmap array
- `rotation_period_us`: Rotation period
- `zero_point_offset_us`: Zero point offset
- `display_mode`: Display mode for the table

**Example:**
```cpp
UnitreeMavlinkController::LedTable table{};
table.fill(0xFF);  // All LEDs on
controller.set_led_table(table);
```

#### save_configuration
Saves current configuration to LiDAR flash memory.

```cpp
bool save_configuration();
```

#### reboot_device
Performs soft reboot of the LiDAR device.

```cpp
bool reboot_device();
```

#### send_device_command
Sends generic device command.

```cpp
bool send_device_command(uint8_t command_type);
```

#### set_ids
Updates MAVLink system and component IDs.

```cpp
void set_ids(uint8_t system_id, uint8_t component_id);
```

## Constants

### Data Limits
```cpp
constexpr size_t kReflectCount = 120;    // Reflection data points
constexpr size_t kDistanceBytes = 240;   // Distance data bytes
```

### Default Values
```cpp
constexpr uint32_t kDefaultLedRotationPeriodUs = 1000000;  // 1 second
```

## Complete Usage Example

```cpp
#include "UnitreeMavlink.h"
#include "UnitreeMavlinkControl.h"

using namespace unitree::mav;

// Hardware setup
HardwareSerial &lidar_serial = Serial1;
UnitreeMavlinkParser parser;
LidarPipeline pipeline;
UnitreeMavlinkController controller(lidar_serial);

// IMU data callback
void handle_imu(const ImuSample &imu) {
    Serial.printf("IMU: q=%.3f,%.3f,%.3f,%.3f\n",
                  imu.quaternion[0], imu.quaternion[1],
                  imu.quaternion[2], imu.quaternion[3]);
}

// Point cloud callback
void handle_cloud(uint16_t packet_id, const std::vector<PointXYZI> &cloud) {
    Serial.printf("Cloud %d: %d points\n", packet_id, cloud.size());
    // Process point cloud data
}

void setup() {
    // Initialize serial communication
    lidar_serial.begin(2000000, SERIAL_8N1, 18, 17);  // 2 Mbps, RX=18, TX=17

    // Configure parser callbacks
    parser.set_imu_callback(handle_imu);
    parser.set_lidar_aux_callback([&](const LidarAuxPacket &aux) {
        pipeline.handle_auxiliary(aux);
    });
    parser.set_lidar_distance_callback([&](const LidarDistancePacket &dist) {
        pipeline.handle_distance(dist);
    });

    // Configure pipeline callback
    pipeline.set_cloud_callback(handle_cloud);

    // Initialize LiDAR
    controller.set_work_mode(UnitreeMavlinkController::WorkMode::kNormal);
    controller.set_led_pattern(UnitreeMavlinkController::LedPattern::kSixStageBreathing);
}

void loop() {
    // Process incoming data
    parser.poll(lidar_serial);
}
```

## Error Handling

### Return Values
Most controller methods return `bool` indicating success/failure:
- `true`: Command successfully sent to LiDAR
- `false`: Communication error or invalid parameters

### Debug Output
Enable debug output by setting appropriate debug levels:
```cpp
#define CORE_DEBUG_LEVEL 3  // Enable debug messages
```

## Thread Safety

The library is designed for single-threaded operation. For multi-threaded environments:
- Use appropriate synchronization mechanisms
- Consider separate parser instances per thread
- Implement thread-safe callback mechanisms

## Memory Management

### Static Allocation
The library uses static allocation where possible to avoid heap fragmentation.

### Dynamic Allocation
- Point clouds use `std::vector` for flexible sizing
- Callbacks use `std::function` for flexibility
- Consider memory constraints on embedded systems

## Performance Considerations

### Optimization Tips
1. **Callback Efficiency**: Keep callback functions lightweight
2. **Memory Usage**: Pre-allocate vectors when possible
3. **Polling Frequency**: Adjust `poll()` call frequency based on data rate
4. **Serial Buffer**: Ensure adequate serial buffer sizes

### Benchmarks
- IMU processing: <1ms per packet
- Point cloud assembly: Real-time capability
- Memory footprint: ~10KB code, ~2KB RAM

## Version Information

### Library Version
Current version: 1.0.0

### Compatibility
- Unitree LiDAR SDK: 1.0.16
- MAVLink Protocol: 2.0
- Arduino ESP32 Core: ^6.7.0

## Support

For issues and questions:
1. Check existing documentation
2. Review example code
3. Enable debug output
4. Capture serial traces
5. Report with complete configuration details
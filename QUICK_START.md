# ESP32 Unitree LiDAR - Quick Start Guide

## 🚀 Get Started in 5 Minutes

This guide will get your ESP32-S3 talking to a Unitree LiDAR L1 and streaming data over Wi-Fi.

## 📋 Prerequisites

### Hardware Checklist
- [ ] ESP32-S3-DevKitC-1 board
- [ ] Unitree LiDAR L1 with UART cable
- [ ] USB-C cable for programming
- [ ] Jumper wires for connections
- [ ] 5V power supply (≥1.5A recommended)

### Software Requirements
- [ ] PlatformIO installed
- [ ] Git for version control
- [ ] Python 3.8+ (for host utilities)
- [ ] Serial monitor (PlatformIO or Arduino IDE)

## 🔌 Step 1: Hardware Setup

### Wiring Connections
Connect your ESP32 to the LiDAR using these pins:

```
ESP32-S3    →    LiDAR L1
--------         --------
GPIO18 (RX) ←---- TX
GPIO17 (TX) ----→ RX
GND       ------ GND
5V        ------ VCC (optional, see power notes)
```

**⚠️ Power Notes:**
- LiDAR can draw 1.5A during spin-up
- Use separate 5V supply if ESP32 brownouts occur
- Keep USB-C connected to LiDAR for firmware updates

## 💻 Step 2: Software Setup

### 1. Clone the Repository
```bash
git clone <your-repo-url>
cd ESP32-wifi
```

### 2. Configure Wi-Fi Credentials
```bash
cp src/secrets.example.h src/secrets.h
nano src/secrets.h  # Edit with your Wi-Fi details
```

**secrets.h template:**
```cpp
#pragma once
constexpr const char* WIFI_SSID = "YourWiFiNetwork";
constexpr const char* WIFI_PASS = "YourPassword";
```

### 3. Install Dependencies
```bash
pio lib install
```

## 🔧 Step 3: Build and Flash

### Build Firmware
```bash
pio run
```

### Flash to ESP32
```bash
pio run -t upload
```

### Monitor Output
```bash
pio device monitor -b 115200
```

## ✅ Step 4: Verification

### Expected Boot Sequence
Watch for these messages in the serial monitor:

```
[Unitree Demo] booting
[Unitree Demo] UART ready (baud=2000000, RX=18, TX=17)
[IMU] packet=12345, q=0.999,0.001,0.002,0.003
[IMU] packet=12346, q=0.999,0.001,0.002,0.003
...
```

### LiDAR Behavior Check
- Motor should start spinning (normal mode)
- LED ring should show breathing pattern
- IMU data should stream at ~200Hz

## 🎯 Step 5: Basic Control Test

### Using the Controller API
Add this code to your `setup()` function:

```cpp
#include "UnitreeMavlinkControl.h"

using namespace unitree::mav;

HardwareSerial &lidar_serial = Serial1;
UnitreeMavlinkController controller(lidar_serial);

void setup() {
    lidar_serial.begin(2000000, SERIAL_8N1, 18, 17);

    // Set work mode
    controller.set_work_mode(UnitreeMavlinkController::WorkMode::kNormal);

    // Configure LED pattern
    controller.set_led_pattern(UnitreeMavlinkController::LedPattern::kSixStageBreathing);

    // Custom LED table (all LEDs on)
    UnitreeMavlinkController::LedTable table{};
    table.fill(0xFF);
    controller.set_led_table(table);
}
```

## 🧪 Step 6: Data Processing

### Basic Data Reception
```cpp
#include "UnitreeMavlink.h"

UnitreeMavlinkParser parser;

void setup() {
    // Setup serial (same as above)

    // Configure data callbacks
    parser.set_imu_callback([](const ImuSample& imu) {
        Serial.printf("IMU: %.2f,%.2f,%.2f,%.2f\n",
                      imu.quaternion[0], imu.quaternion[1],
                      imu.quaternion[2], imu.quaternion[3]);
    });

    parser.set_lidar_distance_callback([](const LidarDistancePacket& dist) {
        Serial.printf("Distance packet: %d bytes\n", dist.payload_size);
    });
}

void loop() {
    parser.poll(Serial1);
}
```

## 🔍 Troubleshooting

### No Data Reception
```bash
# Check wiring
- Verify RX/TX are crossed (ESP32-RX ← LiDAR-TX)
- Confirm common ground connection
- Check for loose connections

# Verify baud rate
- Default: 2,000,000 bps
- Older firmware: 921,600 bps
- Check platformio.ini for build flags
```

### LiDAR Not Responding
```bash
# Power cycle everything
- Disconnect all power
- Wait 10 seconds
- Reconnect in order: LiDAR → ESP32 → USB

# Check work mode
- Ensure set_work_mode() is called after serial init
- Verify mode isn't stuck in standby
```

### ESP32 Brownouts
```bash
# Power supply issues
- Use separate 5V supply for LiDAR
- Add 1000µF capacitor across LiDAR power
- Check supply voltage under load
```

## 📡 Network Integration (Advanced)

### Current TCP Helper
```bash
# Test connectivity
python3 host.py --host esp32s3-lidar.local --port 12345 ping

# Interactive control
python3 host.py --host esp32s3-lidar.local repl
```

### Expected Output
```
> PING
PONG
> SET 255 0 0  # Set LED to red
OK
> GET
255,0,0
```

## 📊 Performance Monitoring

### Key Metrics to Watch
- **IMU Rate**: Should be ~200Hz
- **Packet Loss**: Monitor for missing sequence numbers
- **Temperature**: Check LiDAR temp in aux data
- **Power Consumption**: Monitor supply current

### Debug Output
Enable verbose debugging:
```cpp
#define CORE_DEBUG_LEVEL 5  // Maximum verbosity
```

## 🎨 Customization Ideas

### LED Patterns
```cpp
// Create custom animations
UnitreeMavlinkController::LedTable table{};
for (int i = 0; i < table.size(); i++) {
    table[i] = (i % 2) ? 0xFF : 0x00;  // Alternating pattern
}
controller.set_led_table(table);
```

### Data Filtering
```cpp
parser.set_imu_callback([](const ImuSample& imu) {
    // Simple motion detection
    float total_accel = sqrt(
        imu.linear_acceleration[0] * imu.linear_acceleration[0] +
        imu.linear_acceleration[1] * imu.linear_acceleration[1] +
        imu.linear_acceleration[2] * imu.linear_acceleration[2]
    );

    if (total_accel > 10.0) {
        Serial.println("Motion detected!");
    }
});
```

### Point Cloud Processing
```cpp
pipeline.set_cloud_callback([](uint16_t packet_id, const std::vector<PointXYZI>& cloud) {
    // Find closest point
    float min_distance = 999.0;
    for (const auto& point : cloud) {
        float distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    Serial.printf("Closest object: %.2f meters\n", min_distance);
});
```

## 📚 Next Steps

### Documentation
- Read the [Complete Documentation](DOCS.md)
- Study the [API Reference](API_REFERENCE.md)
- Review [Repository Guidelines](AGENTS.md)

### Advanced Features
- Point cloud visualization
- SLAM integration
- Network streaming protocols
- Real-time processing pipelines

### Community Resources
- Unitree SDK documentation
- MAVLink protocol specification
- ESP32 Arduino community
- PlatformIO documentation

## 🆘 Getting Help

### Before Asking
1. Check serial output for error messages
2. Verify wiring matches diagram
3. Test with example code first
4. Enable debug output

### Information to Provide
- ESP32 board model
- LiDAR firmware version
- Serial output during failure
- Wiring configuration
- Power supply specifications

### Support Channels
- GitHub issues for bugs
- Documentation for feature requests
- Community forums for general help

---

**🎉 Congratulations!** Your ESP32 is now a fully functional LiDAR bridge. Explore the examples, experiment with the API, and build something amazing!

*Need more details? Check out the [complete documentation](DOCS.md) or dive into the [API reference](API_REFERENCE.md).*
# ESP32-S3 Unitree LiDAR Bridge - Complete Documentation

## 🎯 Project Overview

This project transforms an ESP32-S3-DevKitM into a UART-to-Wi-Fi bridge for Unitree LiDAR L1 data. The firmware provides real-time MAVLink parsing, IMU and point-cloud stream processing, and comprehensive LiDAR control capabilities.

### 🔑 Key Features
- **High-speed UART communication** at 2 Mbps with LiDAR
- **MAVLink protocol parsing** for IMU and point-cloud data
- **LiDAR control commands** (work mode, LED patterns, configuration)
- **ESP32-native helpers** for streamlined development
- **Wi-Fi connectivity** for remote data access
- **PlatformIO build system** with comprehensive testing

## 🏗️ Architecture & Components

### Core Components

```
ESP32-S3-DevKitM
    ↓ UART (2 Mbps)
Unitree LiDAR L1
    ↓ MAVLink Protocol
Data Processing Pipeline
    ↓ Wi-Fi
Remote Applications
```

### Key Libraries & Dependencies

| Component | Purpose | Version |
|-----------|---------|---------|
| Adafruit NeoPixel | LED control | ^1.12.3 |
| MAVLink | Protocol implementation | ^2.0.23 |
| Unitree LiDAR SDK | Vendor protocol support | 1.0.16 |
| ESP32 Arduino Core | Hardware abstraction | ^6.7.0 |

## 📁 Project Structure

```
ESP32-wifi/
├── src/
│   ├── main.cpp              # Primary firmware entry point
│   ├── secrets.h             # Wi-Fi credentials (git-ignored)
│   └── secrets.example.h     # Template for credentials
├── lib/
│   ├── unitree_mavlink/      # MAVLink parsing and control
│   │   ├── UnitreeMavlink.h/.cpp
│   │   ├── UnitreeMavlinkControl.h/.cpp
│   │   └── README.md
│   └── motor/                # Motor control utilities
├── include/
│   ├── unitree_lidar_sdk/    # Vendor SDK (v1.0.16)
│   │   ├── include/          # Protocol headers
│   │   ├── examples/         # Reference implementations
│   │   └── lib/              # Pre-built libraries
│   └── README
├── examples/
│   └── unitree_mavlink_demo/ # Standalone Arduino demo
├── test/                     # Unity test suite
├── host.py                   # Legacy TCP helper
├── platformio.ini           # Build configuration
├── README.md                # Primary documentation
└── AGENTS.md               # Development guidelines
```

## 🔧 Hardware Requirements

### Essential Components
- **ESP32-S3-DevKitC-1** (or Arduino-compatible ESP32-S3 board)
- **Unitree LiDAR L1** with UART cable
- **Power supply** ≥ 1.5A @ 5V (LiDAR spin-up current)

### Wiring Configuration
```
ESP32-S3            LiDAR L1
--------            --------
RX  (GPIO18)  ←---- TX
TX  (GPIO17)  ----→ RX
GND           ------ GND
5V            ------ VCC (optional, see power notes)
```

### UART Configuration
- **Baud Rate**: 2,000,000 bps (configurable: 921,600 bps for older firmware)
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Pins**: RX=18, TX=17 (override with `UNITREE_RX_PIN`/`UNITREE_TX_PIN` build flags)

## ⚙️ Software Architecture

### MAVLink Processing Pipeline

```cpp
// Data flow architecture
HardwareSerial → UnitreeMavlinkParser → LidarPipeline → Application
                    ↓
            ImuCallback | LidarAuxCallback | LidarDistanceCallback
```

### Core Classes

#### UnitreeMavlinkParser
- **Purpose**: Byte-by-byte MAVLink frame parsing
- **Callbacks**: IMU data, auxiliary packets, distance measurements
- **Features**: Automatic frame validation and reassembly

#### LidarPipeline
- **Purpose**: Point cloud assembly and processing
- **Features**: Packet correlation, coordinate transformation
- **Output**: Structured point cloud data (PointXYZI format)

#### UnitreeMavlinkController
- **Purpose**: High-level LiDAR control interface
- **Capabilities**: Work mode control, LED patterns, configuration management

### Work Mode Control
```cpp
enum WorkMode {
    kNormal,    // Standard operation mode
    kStandby,   // Low-power standby
    kRaw        // Raw data output mode
};
```

## 🚀 Quick Start Guide

### 1. Environment Setup
```bash
# Install PlatformIO
pip install platformio

# Clone repository
git clone <repository-url>
cd ESP32-wifi

# Install dependencies
pio lib install
```

### 2. Configuration
```bash
# Copy credentials template
cp src/secrets.example.h src/secrets.h

# Edit Wi-Fi credentials
nano src/secrets.h
```

### 3. Build & Flash
```bash
# Build firmware
pio run

# Flash to device
pio run -t upload

# Monitor output
pio device monitor -b 115200
```

### 4. Verification Checklist
1. **Boot Message**: `[Unitree Demo] booting`
2. **UART Ready**: `[Unitree Demo] UART ready (baud=2000000, RX=18, TX=17)`
3. **IMU Streaming**: `[IMU] packet=...` at ~200Hz
4. **LiDAR Behavior**: Motor spinning in appropriate mode

## 📊 Data Protocols

### MAVLink Message Types

| Message ID | Purpose | Frequency |
|------------|---------|-----------|
| IMU Attitude | Orientation data | 200 Hz |
| LiDAR Distance | Range measurements | Variable |
| LiDAR Auxiliary | Metadata and status | Variable |
| Time Sync | Synchronization | Periodic |

### Data Formats

#### IMU Sample
```cpp
struct ImuSample {
    float quaternion[4];    // W, X, Y, Z
    float angular_velocity[3];  // rad/s
    float linear_acceleration[3]; // m/s²
    uint64_t timestamp_us;
};
```

#### Point Cloud Point
```cpp
struct PointXYZI {
    float x, y, z;      // Cartesian coordinates (m)
    float intensity;    // Reflectivity value
    uint16_t ring;      // LiDAR ring number
    uint32_t timestamp; // Relative timestamp
};
```

## 🎮 Control Interface

### LED Control
```cpp
// Built-in patterns
controller.set_led_pattern(UnitreeMavlinkController::LedPattern::kSixStageBreathing);

// Custom LED table (45-byte bitmap)
UnitreeMavlinkController::LedTable table{};
table.fill(0xFF);  // All LEDs on
controller.set_led_table(table);
```

### Configuration Management
```cpp
// Save current settings to flash
controller.save_configuration();

// Reboot device
controller.reboot_device();
```

## 🧪 Testing & Validation

### Unit Tests
```bash
# Run Unity test suite
pio test -e esp32-s3-devkitc-1

# Run specific test
pio test -e esp32-s3-devkitc-1 -f test_name
```

### Integration Tests
```bash
# Test with host utility
python3 host.py --host esp32s3-lidar.local --port 12345 ping

# Interactive REPL mode
python3 host.py --host esp32s3-lidar.local repl
```

### Performance Benchmarks
- **UART Throughput**: 2 Mbps sustained
- **IMU Processing**: <1ms per packet
- **Point Cloud Assembly**: Real-time (depends on point density)

## 🔍 Troubleshooting

### Common Issues

#### No Data Reception
```
Symptom: LiDAR spins but no packets received
Diagnosis: UART wiring or configuration issue
Solutions:
- Verify RX/TX connections (cross-connected)
- Check baud rate (2M vs 921K)
- Ensure common ground
- Test with logic analyzer
```

#### LiDAR Stays in Standby
```
Symptom: Motor doesn't spin or spins slowly
Diagnosis: Work mode not set correctly
Solutions:
- Call set_work_mode(kNormal) after serial init
- Verify mode command timing
- Check for command-table mode conflicts
```

#### ESP32 Brownouts
```
Symptom: ESP32 resets when LiDAR starts
Diagnosis: Power supply insufficient
Solutions:
- Use separate 5V rail for LiDAR
- Increase power supply capacity
- Add bulk capacitance
```

### Debug Tools
```bash
# Enable verbose debugging
pio run -D CORE_DEBUG_LEVEL=5

# Monitor UART traffic
pio device monitor --raw

# Logic analyzer verification
# Check for 0xFE 0xFE MAVLink frame markers
```

## 📡 Network Integration

### Current Implementation
- **Legacy TCP Helper**: `host.py` provides basic connectivity
- **Port**: 12345 (configurable in `platformio.ini`)
- **Protocol**: Simple command-response

### Future Enhancements
- **UDP Streaming**: Real-time data broadcast
- **WebSocket Interface**: Browser-based visualization
- **MQTT Integration**: IoT connectivity
- **REST API**: HTTP-based control interface

## 🔒 Security Considerations

### Current Measures
- Wi-Fi credentials isolated in `secrets.h`
- No hardcoded authentication
- Local network operation

### Recommendations
- Implement WPA3 when available
- Add device authentication for remote access
- Enable secure firmware updates
- Monitor for unauthorized access attempts

## 📈 Performance Optimization

### Memory Management
- **Static allocation** for real-time operations
- **Circular buffers** for data streaming
- **Zero-copy** where possible

### Processing Optimization
- **Interrupt-driven** UART handling
- **DMA transfers** for bulk data
- **Task prioritization** for real-time requirements

### Power Management
- **Dynamic frequency scaling** based on load
- **Sleep modes** during idle periods
- **Efficient LED patterns** to minimize current draw

## 🔄 Development Workflow

### Code Style Guidelines
- **Indentation**: 2 spaces
- **Naming**: snake_case for functions, UPPER_CASE for macros
- **Comments**: Concise behavior notes
- **Includes**: Standard library → Framework → Project headers

### Testing Strategy
- **Unit tests**: Core algorithm validation
- **Integration tests**: End-to-end functionality
- **Hardware-in-loop**: Real device validation
- **Regression tests**: CI pipeline integration

### Version Control
- **Semantic versioning**: MAJOR.MINOR.PATCH
- **Feature branches**: Isolate development
- **Tag releases**: Mark stable versions
- **Documentation**: Keep synchronized with code

## 📚 Additional Resources

### Reference Documentation
- [Unitree LiDAR SDK Documentation](include/unitree_lidar_sdk/README.md)
- [MAVLink Protocol Specification](https://mavlink.io/en/)
- [ESP32-S3 Technical Reference](https://www.espressif.com/en/products/socs/esp32-s3)
- [PlatformIO Documentation](https://docs.platformio.org/)

### Community Resources
- [Arduino ESP32 Community](https://github.com/espressif/arduino-esp32)
- [MAVLink Developer Community](https://mavlink.io/en/about/support.html)
- [Unitree Robotics Support](https://www.unitree.com/)

### Development Tools
- **PlatformIO IDE**: Integrated development environment
- **ESP-IDF**: Espressif IoT Development Framework
- **Logic Analyzers**: UART protocol debugging
- **Oscilloscopes**: Signal quality analysis

---

## 📞 Support & Contributing

### Issue Reporting
When reporting issues, please include:
1. **Hardware configuration** (ESP32 variant, LiDAR model)
2. **Software versions** (firmware, SDK versions)
3. **Wiring diagram** or connection details
4. **Serial output** during failure
5. **Steps to reproduce** the issue

### Contribution Guidelines
1. **Fork** the repository
2. **Create feature branch** from main
3. **Write tests** for new functionality
4. **Update documentation** as needed
5. **Submit pull request** with detailed description

### Development Environment
- **PlatformIO Core**: Latest version
- **ESP32 Arduino Core**: ^6.7.0
- **Unitree LiDAR SDK**: 1.0.16
- **Python**: 3.8+ (for host utilities)

---

*This documentation is automatically generated and maintained. For updates, please refer to the project's README.md and inline code documentation.*
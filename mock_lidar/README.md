# Mock Unitree Lidar L1

模拟 Unitree Lidar L1 的独立 ESP32 项目，用于 Wokwi 仿真环境。

## 功能

通过 UART 发送 MAVLink 格式数据：

| 数据类型 | 频率 | 描述 |
|---------|------|------|
| IMU 姿态 | 200Hz | 四元数、角速度、线加速度 |
| 点云 | 10Hz | 120个点的距离和反射率 |

## 使用方法

### 本地编译
```bash
cd mock_lidar
pio run
```

### Wokwi 仿真

1. 在 Wokwi 中创建双 ESP32 项目
2. 使用 `diagram.json` 配置连接
3. Mock Lidar 的 TX (GPIO17) 连接到主 ESP32 的 RX (GPIO18)

## 数据生成模式

- `MODE_STATIC`: 固定数据，用于基本功能验证
- `MODE_DYNAMIC`: 模拟真实场景（旋转、距离变化）

## UART 配置

- **波特率**: 2,000,000 bps
- **TX**: GPIO17
- **RX**: GPIO18

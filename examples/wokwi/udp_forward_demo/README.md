# Wokwi Virtual Serial + UDP Forwarding Demo

学习 Wokwi 虚拟串口和 UDP 网关转发功能的完整 Demo。

## ✅ 功能验证结果

| 功能 | 状态 | 说明 |
|------|------|------|
| RFC2217 虚拟串口 | ✅ | Python 通过 `rfc2217://localhost:4000` 发送数据到 ESP32 |
| ESP32 WiFi 连接 | ✅ | 连接到 Wokwi-GUEST (IP: 10.13.37.2) |
| UDP 转发到本机 | ✅ | ESP32 → host.wokwi.internal → wokwigw → Python |

## 🚧 核心卡点与解决方案

### 1. wokwi-cli 不支持私有网关
- **问题**: `wokwi-cli` 不读取 `[net] gateway` 配置
- **解决**: 必须使用 **VS Code Wokwi 扩展** + 独立运行 **wokwigw**

### 2. RFC2217 只能在 VS Code 终端运行 ⚠️
- **问题**: 外部终端 (iTerm) 运行脚本无法发送数据到 ESP32
- **解决**: **必须在 VS Code 集成终端中运行 Python 脚本**

### 3. UDP 转发目标地址
- **问题**: 使用网关 IP `10.13.37.1` 无法转发
- **解决**: 使用 `host.wokwi.internal` 特殊域名

### 4. 数据发送丢失
- **问题**: 批量发送数据丢失
- **解决**: 逐字符发送 + 10ms 延迟

## 📁 文件说明

| 文件 | 说明 |
|------|------|
| `udp_forward_test.cpp` | ESP32 固件 - 接收串口消息并通过 UDP 转发 |
| `serial_test.py` | 串口发送脚本 (RFC2217) |
| `udp_receiver.py` | UDP 接收器 |
| `wokwi.toml` | Wokwi 配置 (启用 RFC2217 + 私有网关) |
| `diagram.json` | ESP32-S3 硬件配置 |

## 🚀 运行步骤

### 1. 编译固件
```bash
cp examples/wokwi/udp_forward_demo/udp_forward_test.cpp src/main.cpp
pio run
```

### 2. 启动 wokwigw 网关 (保持运行)
```bash
~/.local/bin/wokwigw
```

### 3. VS Code 启动 Wokwi 仿真
- 打开 `examples/wokwi/udp_forward_demo/diagram.json`
- 按 `F1` → "Wokwi: Start Simulator"
- 等待 WiFi 连接成功

### 4. 启动 UDP 接收器 (任意终端)
```bash
uv run examples/wokwi/udp_forward_demo/udp_receiver.py
```

### 5. 在 VS Code 集成终端运行串口发送 ⚠️
```bash
uv run examples/wokwi/udp_forward_demo/serial_test.py
```

## 📊 数据流

```
Python serial_test.py (VS Code 终端)
    ↓ RFC2217 (localhost:4000)
ESP32 仿真
    ↓ Serial.read() → UDP.send()
    ↓ host.wokwi.internal:8888
wokwigw 网关 (localhost:9011)
    ↓ 转发
Python udp_receiver.py (localhost:8888)
```

## 📋 wokwi.toml 配置说明

```toml
[wokwi]
version = 1
firmware = "../../../.pio/build/esp32-s3-devkitc-1/firmware.bin"
elf = "../../../.pio/build/esp32-s3-devkitc-1/firmware.elf"

# RFC2217 虚拟串口
rfc2217ServerPort = 4000

# 私有网关
[net]
gateway = "ws://localhost:9011"

# 端口转发
[[net.forward]]
from = "localhost:8888"
to = "target:8888"
```

## 💡 关键学习点

1. **VS Code 集成终端是必须的** - RFC2217 脚本只能在 VS Code 终端正常工作
2. **wokwigw 必须独立运行** - VS Code 扩展不内置网关
3. **host.wokwi.internal** - Wokwi 专用域名，用于 ESP32 连接本机
4. **逐字符发送** - 避免数据丢失，每字符间隔 10ms

# Wokwi Virtual Serial + WebSocket Forwarding Demo

学习 Wokwi 虚拟串口和 **WebSocket** 转发功能的 Demo。

## 📊 数据流

```
Python serial_sender.py (VS Code 终端)
    ↓ RFC2217 (localhost:4000)
ESP32 仿真
    ↓ Serial.read() → WebSocket.send()
    ↓ host.wokwi.internal:8888
wokwigw 网关 (localhost:9011)
    ↓ 转发
Python ws_server.py (localhost:8888)
```

## 📁 文件说明

| 文件 | 说明 |
|------|------|
| `websocket_test.cpp` | ESP32 固件 - 接收串口消息并通过 WebSocket 转发 |
| `serial_sender.py` | 串口发送脚本 (RFC2217) |
| `ws_server.py` | WebSocket 服务器 |
| `wokwi.toml` | Wokwi 配置 |
| `diagram.json` | ESP32-S3 硬件配置 |

## 🚀 运行步骤

### 1. 添加 WebSocketsClient 库到 platformio.ini

```ini
lib_deps = 
    links2004/WebSockets@^2.4.0
```

### 2. 编译固件
```bash
cp examples/wokwi/websocket_demo/websocket_test.cpp src/main.cpp
pio run
```

### 3. 启动 wokwigw 网关 (保持运行)
```bash
~/.local/bin/wokwigw
```

### 4. 启动 WebSocket 服务器 (任意终端)
```bash
uv run examples/wokwi/websocket_demo/ws_server.py
```

### 5. VS Code 启动 Wokwi 仿真
- 打开 `examples/wokwi/websocket_demo/diagram.json`
- 按 `F1` → "Wokwi: Start Simulator"
- 等待 WiFi 和 WebSocket 连接成功

### 6. 在 VS Code 集成终端运行串口发送 ⚠️
```bash
uv run examples/wokwi/websocket_demo/serial_sender.py
```

## ⚠️ 注意事项

1. **必须在 VS Code 集成终端运行 serial_sender.py**
2. **wokwigw 必须先启动**
3. **保持仿真窗口可见**

## 💡 与 UDP 方案的对比

| 特性 | UDP | WebSocket |
|------|-----|-----------|
| 可靠性 | 不可靠 | 可靠 (TCP) |
| 连接状态 | 无连接 | 有连接 |
| 双向通信 | 需要额外代码 | 原生支持 |
| 延迟 | 更低 | 略高 |
| 复杂度 | 简单 | 稍复杂 |

# Wokwi Virtual Serial Demo

这是一个学习 Wokwi 虚拟串口功能的测试 Demo。

## ✅ 测试结果

成功实现 Python 脚本向 ESP32 发送 "Hello World!" 并在 Wokwi console 中显示！

```
[ESP32] ┌─────────────────────────────────────────┐
[ESP32] │ [#1] 收到消息: Hello World! #1
[ESP32] │ Received: Hello World! #1
[ESP32] └─────────────────────────────────────────┘
[ESP32] [ACK] Message #1 received successfully
```

## 原理说明

```
┌──────────────────┐   subprocess    ┌──────────────────┐
│   Python 脚本    │ ─────stdin────▶│   wokwi-cli      │
│ serial_sender.py │                │  --interactive   │
└──────────────────┘                └────────┬─────────┘
        │                                    │
        │ print()                            │ 虚拟串口
        ▼                                    ▼
   ┌──────────┐                      ┌──────────────┐
   │ Terminal │                      │ ESP32 (Wokwi)│
   │  输出    │◀────[ESP32] output───│  仿真        │
   └──────────┘                      └──────────────┘
```

## 文件说明

| 文件 | 说明 |
|------|------|
| `virtual_serial_test.cpp` | ESP32 固件源码，接收并显示消息 |
| `serial_sender.py` | Python 发送脚本，通过 subprocess 控制 wokwi-cli |
| `wokwi.toml` | Wokwi 配置文件 |
| `diagram.json` | Wokwi 硬件配置（ESP32-S3） |

## 使用方法

### 一键运行

```bash
# 1. 编译固件
pio run

# 2. 运行 Python 脚本（自动启动 Wokwi）
uv run examples/wokwi/virtual_serial/serial_sender.py
```

### 手动测试（交互模式）

如果你想手动输入消息进行测试：

```bash
# 1. 启动 Wokwi（交互模式）
wokwi-cli --interactive examples/wokwi/virtual_serial

# 2. 在终端中输入消息，按 Enter 发送
Hello World!
```

## 预期输出

### 运行 Python 脚本

```
=======================================================
   Wokwi Virtual Serial Sender Demo (Subprocess)
   通过 wokwi-cli --interactive 发送消息
=======================================================

[CONFIG] Project Dir: examples/wokwi/virtual_serial
[CONFIG] Send Interval: 1.0s
[CONFIG] Max Messages: 10

[INFO] 正在启动 Wokwi 仿真...
[SUCCESS] Wokwi 进程已启动

[ESP32] ╔══════════════════════════════════════════╗
[ESP32] ║   Wokwi Virtual Serial Demo              ║
[ESP32] ║   虚拟串口学习测试                       ║
[ESP32] ╚══════════════════════════════════════════╝

[SENT] 已发送 #1: Hello World! #1
[ESP32] ┌─────────────────────────────────────────┐
[ESP32] │ [#1] 收到消息: Hello World! #1
[ESP32] │ Received: Hello World! #1
[ESP32] └─────────────────────────────────────────┘
[ESP32] [ACK] Message #1 received successfully

[SENT] 已发送 #2: Hello World! #2
...
```

## 技术要点

### wokwi-cli --interactive 模式

`wokwi-cli` 的 `--interactive` 选项可以将 stdin 重定向到 ESP32 的串口：

```bash
wokwi-cli --interactive <project-dir>
```

这允许你：
- 手动在终端输入发送到 ESP32
- 或使用 subprocess 从 Python 发送数据

### diagram.json 配置

```json
{
    "attrs": {
        "serialInterface": "USB_SERIAL_JTAG"
    }
}
```

> **重要**: `serialInterface` 必须设置为 `USB_SERIAL_JTAG`，这样 Serial 才能正常工作。

## 扩展练习

1. **修改发送内容**: 在 `serial_sender.py` 中修改消息内容
2. **双向通信**: 在 ESP32 中发送数据，Python 读取并处理
3. **命令解析**: 实现简单的命令协议（如 LED 控制）
4. **持续监控**: 添加 `--timeout 0` 让仿真持续运行

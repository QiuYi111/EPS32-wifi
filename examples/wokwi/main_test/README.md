# Wokwi Main Test - ESP32 全链路仿真

测试 `examples/main.cpp` 的完整数据流：Mock LiDAR → ESP32 → WebUI。

## ⚠️ 重要限制

> 根据 [udp_forward_demo](../udp_forward_demo/README.md) 的验证结果：
> 1. **Wokwi 必须通过 VS Code 扩展启动** - `wokwi-cli` 不支持私有网关
> 2. **Mock LiDAR 脚本必须在 VS Code 集成终端运行** - RFC2217 只能在 VS Code 终端工作

## 架构

```
┌─────────────────┐     RFC2217      ┌─────────────────┐
│  mock_lidar.py  │ ──────────────→  │  ESP32 (Wokwi)  │
│  (VS Code终端)  │     Serial1      │  main_wokwi.cpp │
└─────────────────┘                  └────────┬────────┘
                                              │
                                         WebSocket
                                         (wokwigw)
                                              │
                                              ▼
                                     ┌─────────────────┐
                                     │  ws_server.py   │
                                     │  格式转换+转发  │
                                     └────────┬────────┘
                                              │
                                         WebSocket :81
                                              │
                                              ▼
                                     ┌─────────────────┐
                                     │  WebUI (浏览器) │
                                     │  点云可视化     │
                                     └─────────────────┘
```

## 快速开始

### Step 1: 编译固件

```bash
pio run
```

### Step 2: 启动辅助服务 (任意终端)

```bash
./examples/wokwi/main_test/run.sh
```

这会在 tmux 中启动：
- wokwigw (网关) - 端口 9011
- ws_server.py (WebSocket 中继) - 端口 8888/81
- WebUI HTTP 服务器 - 端口 8080

### Step 3: 在 VS Code 中启动 Wokwi

1. 打开 `examples/wokwi/main_test/diagram.json`
2. 按 `F1` → 输入 "Wokwi: Start Simulator"
3. 等待 ESP32 初始化完成，看到 `[WS] Connected` 日志

### Step 4: 在 VS Code 集成终端运行 Mock LiDAR ⚠️

```bash
uv run examples/wokwi/main_test/mock_lidar.py
```

> ⚠️ **必须使用 VS Code 集成终端**，外部终端 (iTerm) 无法正常发送数据！

### Step 5: 验证

1. 打开浏览器 http://localhost:8080
2. 在设置中将 WebSocket URL 改为 `ws://localhost:81`
3. 确认状态显示 "Connected"
4. 观察点云可视化

## 文件说明

| 文件 | 说明 |
|------|------|
| `wokwi.toml` | Wokwi 配置 (RFC2217 + 网关) |
| `diagram.json` | 硬件连接图 (单 ESP32) |
| `main_wokwi.cpp` | WebSocket 版 main.cpp |
| `mock_lidar.py` | Python MAVLink 生成器 |
| `ws_server.py` | WebSocket 双向中继 |
| `run.sh` | 辅助服务启动脚本 |

## 数据格式

### ESP32 → ws_server.py

```c
struct DataPacket {
    uint32_t magic;     // 0xDEADBEEF
    uint32_t timestamp;
    uint8_t type;       // DATA_IMU=1, DATA_POINTCLOUD=2, ...
    uint16_t length;
    uint8_t data[];
};
```

### ws_server.py → WebUI

点云数据:
```c
struct WebUIPointCloud {
    uint32_t magic;      // 0x4C494441 "LIDA"
    uint16_t packet_id;
    uint8_t point_count;
    uint8_t reserved;
    struct {
        float x, y, z;
        uint8_t intensity;
    } points[];
};
```

状态/电机数据: JSON 格式

## 故障排除

### Mock LiDAR 无法发送数据

**问题**: 外部终端运行 mock_lidar.py 没有数据

**解决**: 必须在 **VS Code 集成终端** 运行

### ESP32 无法连接 WebSocket

1. 确认 wokwigw 正在运行 (端口 9011)
2. 确认 ws_server.py 在端口 8888 监听
3. 检查 Wokwi 是否通过 VS Code 扩展启动

### WebUI 无数据

1. 确认 ws_server.py 在端口 81 监听
2. 检查浏览器控制台是否有 WebSocket 错误
3. 确认 ESP32 输出中有 `[IMU]` 和 `[PointCloud]` 日志

## tmux 命令参考

| 命令 | 功能 |
|------|------|
| `tmux attach -t wokwi-main-test` | 重新连接 |
| `tmux kill-session -t wokwi-main-test` | 停止所有服务 |
| `Ctrl+B, 方向键` | 切换 pane |
| `Ctrl+B, z` | 放大/缩小当前 pane |
| `Ctrl+B, d` | 后台运行 (detach) |

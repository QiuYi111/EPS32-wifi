#!/bin/bash
# ============================================================
# Wokwi Main Test - 启动助手脚本
# 
# ⚠️ 重要限制:
#   1. Wokwi 必须通过 VS Code 扩展启动 (不能用 wokwi-cli)
#   2. Mock LiDAR 脚本必须在 VS Code 集成终端运行
#
# 此脚本启动辅助服务 (wokwigw, ws_server, WebUI)
# Wokwi 仿真和 Mock LiDAR 需要手动在 VS Code 中操作
#
# 使用方法:
#   chmod +x examples/wokwi/main_test/run.sh
#   ./examples/wokwi/main_test/run.sh
# ============================================================

SESSION="wokwi-main-test"
PROJECT_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
TEST_DIR="$PROJECT_ROOT/examples/wokwi/main_test"

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║   Wokwi Main Test - Helper Services Launcher                 ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║   ⚠️  注意: Wokwi 必须通过 VS Code 扩展启动!                 ║"
echo "║   ⚠️  注意: mock_lidar.py 必须在 VS Code 终端运行!           ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo
echo "Project: $PROJECT_ROOT"
echo

# 检查依赖
check_command() {
    if ! command -v "$1" &> /dev/null; then
        echo "[ERROR] 未找到命令: $1"
        echo "[HINT] 请先安装: $2"
        exit 1
    fi
}

check_command "tmux" "brew install tmux"
check_command "wokwigw" "npm install -g wokwi-cli"
check_command "uv" "curl -LsSf https://astral.sh/uv/install.sh | sh"

# 检查固件是否存在
FIRMWARE="$PROJECT_ROOT/.pio/build/esp32-s3-devkitc-1/firmware.bin"
if [ ! -f "$FIRMWARE" ]; then
    echo "[WARN] 固件不存在: $FIRMWARE"
    echo "[INFO] 正在编译..."
    cd "$PROJECT_ROOT"
    pio run
    if [ $? -ne 0 ]; then
        echo "[ERROR] 编译失败"
        exit 1
    fi
fi

# 如果 session 已存在，先关闭
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

echo "[INFO] 创建 tmux session: $SESSION"

# 创建新 session
tmux new-session -d -s "$SESSION" -c "$PROJECT_ROOT"
tmux rename-window -t "$SESSION:0" "services"

# ==================== 设置布局 ====================
# 分割成 3 个 panes
tmux split-window -v -t "$SESSION:0" -c "$PROJECT_ROOT"
tmux split-window -v -t "$SESSION:0.0" -c "$PROJECT_ROOT"

# ==================== 启动辅助服务 ====================

# Pane 0: wokwigw (网关)
tmux send-keys -t "$SESSION:0.0" "echo '=== Wokwi Gateway ===' && wokwigw -p 9011" Enter

# Pane 1: ws_server.py (WebSocket 中继)
tmux send-keys -t "$SESSION:0.1" "sleep 1 && echo '=== WebSocket Relay ===' && uv run '$TEST_DIR/ws_server.py'" Enter

# Pane 2: WebUI HTTP 服务器
tmux send-keys -t "$SESSION:0.2" "sleep 1 && echo '=== WebUI HTTP Server ===' && python3 -m http.server 8080 -d '$PROJECT_ROOT/webui/frontend'" Enter

# ==================== 显示操作指南 ====================

echo
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║   辅助服务已在 tmux 中启动                                   ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║   接下来请在 VS Code 中执行:                                 ║"
echo "║                                                              ║"
echo "║   1. 打开 examples/wokwi/main_test/diagram.json              ║"
echo "║   2. F1 → \"Wokwi: Start Simulator\"                          ║"
echo "║   3. 等待 ESP32 连接 WiFi 和 WebSocket                       ║"
echo "║   4. 在 VS Code 终端运行:                                    ║"
echo "║      uv run examples/wokwi/main_test/mock_lidar.py           ║"
echo "║                                                              ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║   WebUI: http://localhost:8080                               ║"
echo "║   WebSocket 设置: ws://localhost:81                          ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║   tmux 命令:                                                 ║"
echo "║   - tmux attach -t $SESSION      重新连接                    ║"
echo "║   - tmux kill-session -t $SESSION 停止所有服务               ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo

# 询问是否附加到 session
read -p "是否附加到 tmux session? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Nn]$ ]]; then
    echo "[INFO] 辅助服务在后台运行中"
    echo "[INFO] 使用 'tmux attach -t $SESSION' 查看"
else
    tmux attach-session -t "$SESSION"
fi

#!/usr/bin/env python3
# /// script
# requires-python = ">=3.8"
# dependencies = ["websockets"]
# ///
"""
Wokwi WebSocket Server

接收来自 ESP32 的 WebSocket 消息。

使用方法:
  uv run examples/wokwi/websocket_demo/ws_server.py
"""

import asyncio
import websockets
import signal
import sys

WS_PORT = 8888
message_count = 0

async def handler(websocket, path=None):
    global message_count
    client_addr = websocket.remote_address
    print(f"[WS] ✓ 客户端连接: {client_addr}")
    
    try:
        async for message in websocket:
            message_count += 1
            print()
            print("┌────────────────────────────────────────┐")
            print(f"│ [#{message_count}] 收到消息:")
            print(f"│ 来源: {client_addr}")
            print(f"│ 内容: {message}")
            print("└────────────────────────────────────────┘")
            
            # 发送确认
            await websocket.send(f"[Server] 收到消息 #{message_count}")
            
    except websockets.exceptions.ConnectionClosed:
        print(f"[WS] 客户端断开: {client_addr}")

async def main():
    print()
    print("╔════════════════════════════════════════╗")
    print("║   Wokwi WebSocket Server               ║")
    print("╚════════════════════════════════════════╝")
    print()
    print(f"[INFO] 监听端口: {WS_PORT}")
    print("[INFO] 按 Ctrl+C 停止")
    print()
    print("[OK] WebSocket 服务器已启动，等待连接...")
    print()
    
    async with websockets.serve(handler, "0.0.0.0", WS_PORT):
        await asyncio.Future()  # 永远运行

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] 已停止")

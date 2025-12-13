#!/usr/bin/env python3
# /// script
# requires-python = ">=3.8"
# dependencies = []
# ///
"""
UDP 接收器 (独立版本)

用于独立测试 UDP 接收功能，监听指定端口接收来自 ESP32 的 UDP 消息。

使用方法:
  uv run examples/wokwi/udp_forward_demo/udp_receiver.py

注意: 这是一个独立脚本，用于手动测试时使用。
      完整 demo 请使用 gateway_demo.py
"""

import socket
import sys

UDP_PORT = 8888

def main():
    """主函数"""
    print()
    print("╔════════════════════════════════════════╗")
    print("║   Wokwi UDP Receiver (独立版本)        ║")
    print("╚════════════════════════════════════════╝")
    print()
    print(f"[INFO] 监听端口: {UDP_PORT}")
    print("[INFO] 按 Ctrl+C 停止")
    print()
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', UDP_PORT))
        
        print(f"[OK] UDP 接收器已启动，等待消息...")
        print()
        
        message_count = 0
        
        while True:
            data, addr = sock.recvfrom(1024)
            message_count += 1
            message = data.decode('utf-8', errors='replace')
            
            print("┌────────────────────────────────────────┐")
            print(f"│ [#{message_count}] 收到消息:")
            print(f"│ 来源: {addr[0]}:{addr[1]}")
            print(f"│ 内容: {message}")
            print("└────────────────────────────────────────┘")
            print()
            
    except KeyboardInterrupt:
        print()
        print("[INFO] 用户中断")
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)
    finally:
        sock.close()
        print("[INFO] 已停止")

if __name__ == "__main__":
    main()

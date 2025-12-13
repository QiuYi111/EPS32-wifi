#!/usr/bin/env python3
# /// script
# requires-python = ">=3.8"
# dependencies = ["pyserial"]
# ///
"""
RFC2217 串口发送测试 - 简化版

注意: 运行此脚本时，请确保 VS Code Wokwi 仿真窗口保持可见且活动！
"""

import serial
import time
import sys
import threading

SERIAL_URL = "rfc2217://localhost:4000"

def main():
    print()
    print("═" * 50)
    print("  Wokwi RFC2217 Serial 发送测试")
    print("═" * 50)
    print()
    print("⚠️  重要：请确保 VS Code Wokwi 仿真窗口保持可见！")
    print()
    
    try:
        print("[1] 连接 RFC2217...")
        ser = serial.serial_for_url(SERIAL_URL, baudrate=115200, timeout=1)
        print(f"    ✓ 连接成功: {ser.port}")
    except Exception as e:
        print(f"    ✗ 连接失败: {e}")
        sys.exit(1)
    
    # 后台读取线程
    running = True
    def reader():
        while running:
            try:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting).decode('utf-8', errors='replace')
                    for line in data.splitlines():
                        if line.strip():
                            print(f"[RX] {line}")
            except:
                pass
            time.sleep(0.05)
    
    t = threading.Thread(target=reader, daemon=True)
    t.start()
    
    print()
    print("[2] 清空缓冲区...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.5)
    
    print()
    print("[3] 发送测试消息...")
    print("-" * 50)
    
    messages = ["Hello_world_1", "Hello_world_2", "Hello_world_3"]
    
    for i, msg in enumerate(messages, 1):
        full_msg = f"{msg}\r\n"  # 使用 \r\n 作为行结束符
        
        print(f"[TX #{i}] 发送: {msg!r}")
        
        # 逐字节发送，给仿真时间处理
        for char in full_msg:
            ser.write(char.encode())
            time.sleep(0.01)  # 10ms 每字节
        
        ser.flush()
        
        # 等待响应
        print(f"        等待响应...")
        time.sleep(3)
    
    print("-" * 50)
    print()
    print("[4] 等待最后的输出 (5秒)...")
    time.sleep(5)
    
    running = False
    ser.close()
    
    print()
    print("完成！")

if __name__ == "__main__":
    main()

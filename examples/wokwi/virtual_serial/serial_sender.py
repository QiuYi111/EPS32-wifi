#!/usr/bin/env python3
# /// script
# requires-python = ">=3.8"
# dependencies = []
# ///
"""
Wokwi Virtual Serial Sender Demo (Subprocess 版本)
通过 subprocess 控制 wokwi-cli --interactive 发送消息

这个脚本启动 wokwi-cli 并通过 stdin 向 ESP32 发送 "Hello World!" 消息。

使用方法:
  1. 确保已编译固件: pio run
  2. 运行此脚本: uv run examples/wokwi/virtual_serial/serial_sender.py
  3. 观察 ESP32 接收到的消息

注意: 此脚本使用 wokwi-cli --interactive 模式，不需要 RFC2217。
"""

import subprocess
import time
import sys
import os

# 配置
WOKWI_PROJECT_DIR = "examples/wokwi/virtual_serial"
SEND_INTERVAL = 1.0  # 发送间隔（秒）
MAX_MESSAGES = 10     # 最大发送消息数

def print_banner():
    """打印欢迎横幅"""
    print("=" * 55)
    print("   Wokwi Virtual Serial Sender Demo (Subprocess)")
    print("   通过 wokwi-cli --interactive 发送消息")
    print("=" * 55)
    print()
    print(f"[CONFIG] Project Dir: {WOKWI_PROJECT_DIR}")
    print(f"[CONFIG] Send Interval: {SEND_INTERVAL}s")
    print(f"[CONFIG] Max Messages: {MAX_MESSAGES}")
    print()

def run_wokwi_with_messages():
    """
    启动 wokwi-cli 并发送消息
    """
    print("[INFO] 正在启动 Wokwi 仿真...")
    print("[INFO] Starting Wokwi simulation...")
    print()
    
    # 获取项目根目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    project_dir = os.path.join(project_root, WOKWI_PROJECT_DIR)
    
    # 启动 wokwi-cli
    try:
        process = subprocess.Popen(
            ["wokwi-cli", "--interactive", "--timeout", "30000", project_dir],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,  # 行缓冲
            cwd=project_root
        )
    except FileNotFoundError:
        print("[ERROR] wokwi-cli 未找到，请确保已安装并添加到 PATH")
        return
    
    print("[SUCCESS] Wokwi 进程已启动")
    print()
    
    # 等待 ESP32 启动
    print("[INFO] 等待 ESP32 初始化...")
    time.sleep(3)
    
    # 读取并打印初始输出
    try:
        # 设置非阻塞读取或超时
        import select
        import threading
        
        output_lines = []
        
        def read_output():
            """后台线程读取输出"""
            try:
                for line in iter(process.stdout.readline, ''):
                    if line:
                        print(f"[ESP32] {line}", end='')
                        output_lines.append(line)
            except:
                pass
        
        # 启动输出读取线程
        reader_thread = threading.Thread(target=read_output, daemon=True)
        reader_thread.start()
        
        # 等待初始化完成
        time.sleep(2)
        
        print()
        print("-" * 55)
        print("开始发送消息 | Starting to send messages")
        print("按 Ctrl+C 停止 | Press Ctrl+C to stop")
        print("-" * 55)
        print()
        
        # 发送消息
        for i in range(1, MAX_MESSAGES + 1):
            message = f"Hello World! #{i}"
            
            try:
                # 发送消息到 wokwi-cli 的 stdin
                process.stdin.write(f"{message}\n")
                process.stdin.flush()
                
                print(f"[SENT] 已发送 #{i}: {message}")
                
            except BrokenPipeError:
                print("[ERROR] Wokwi 进程已终止")
                break
            
            # 等待下一次发送
            time.sleep(SEND_INTERVAL)
        
        print()
        print("-" * 55)
        print(f"[INFO] 发送完成，共发送 {min(i, MAX_MESSAGES)} 条消息")
        print("-" * 55)
        
        # 等待一会儿看输出
        time.sleep(2)
        
    except KeyboardInterrupt:
        print()
        print("[INFO] 用户中断")
    
    finally:
        # 终止进程
        print("[INFO] 正在停止 Wokwi...")
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
        print("[INFO] Wokwi 已停止")

def main():
    """主函数"""
    print_banner()
    run_wokwi_with_messages()

if __name__ == "__main__":
    main()

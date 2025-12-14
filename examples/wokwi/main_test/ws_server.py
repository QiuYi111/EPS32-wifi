#!/usr/bin/env python3
# /// script
# requires-python = ">=3.8"
# dependencies = ["websockets"]
# ///
"""
WebSocket 中转服务器

双端口服务:
- 端口 8888: ESP32 连接 (通过 wokwigw 网关)
- 端口 81: WebUI 浏览器连接

数据流:
  ESP32 (port 8888) ←→ ws_server.py ←→ WebUI (port 81)

数据格式转换:
  ESP32 DataPacket: magic=0xDEADBEEF, timestamp, type, length, data
  WebUI PointCloud: magic=0x4C494441 "LIDA", packet_id, point_count, points

使用方法:
  uv run examples/wokwi/main_test/ws_server.py
"""

import asyncio
import websockets
import struct
import json
import sys

# ==================== 配置 ====================
ESP32_PORT = 8888  # ESP32 连接端口
WEBUI_PORT = 81    # WebUI 连接端口

# 数据类型常量 (与 ESP32 main_wokwi.cpp 一致)
DATA_IMU = 1
DATA_POINTCLOUD = 2
DATA_MOTOR = 3
DATA_STATUS = 4
DATA_CONTROL = 5

# Magic 常量
ESP32_MAGIC = 0xDEADBEEF
WEBUI_MAGIC = 0x4C494441  # "LIDA"

# ==================== 全局状态 ====================
esp32_clients = set()
webui_clients = set()
message_count = 0


# ==================== 数据格式转换 ====================
def parse_esp32_packet(data):
    """解析 ESP32 DataPacket"""
    if len(data) < 11:  # header size
        return None
    
    magic, timestamp, data_type, length = struct.unpack('<IIBH', data[:11])
    
    if magic != ESP32_MAGIC:
        return None
    
    payload = data[11:11 + length] if len(data) >= 11 + length else b''
    
    return {
        'magic': magic,
        'timestamp': timestamp,
        'type': data_type,
        'length': length,
        'payload': payload
    }


def convert_pointcloud_to_webui(packet):
    """
    转换点云数据为 WebUI 格式
    
    WebUI 期望格式:
      - magic: uint32 = 0x4C494441 "LIDA"
      - packet_id: uint16
      - point_count: uint8
      - reserved: uint8
      - points[]: 每个点 13 bytes (x:f32, y:f32, z:f32, intensity:u8)
    """
    payload = packet['payload']
    
    # 假设 ESP32 发送的是 PointXYZI 数组
    # PointXYZI = { float x, y, z, intensity; uint16 ring; uint32 timestamp; } = 22 bytes
    point_size = 22
    point_count = len(payload) // point_size if point_size > 0 else 0
    point_count = min(point_count, 255)  # WebUI 限制
    
    # 构建 WebUI 格式
    webui_header = struct.pack('<IHBB', WEBUI_MAGIC, packet['timestamp'] & 0xFFFF, point_count, 0)
    
    webui_points = bytearray()
    for i in range(point_count):
        offset = i * point_size
        if offset + 16 <= len(payload):
            x, y, z, intensity = struct.unpack('<4f', payload[offset:offset + 16])
            # WebUI 格式: x, y, z (float), intensity (uint8)
            webui_points += struct.pack('<3fB', x, y, z, int(min(255, max(0, intensity * 255))))
    
    return webui_header + bytes(webui_points)


def convert_status_to_webui(packet):
    """转换系统状态为 WebUI JSON 格式"""
    payload = packet['payload']
    
    if len(payload) >= 18:  # SystemStatus size
        cpu_temp, wifi_rssi, free_heap, lidar_status, imu_count, cloud_count = struct.unpack(
            '<fBHBII', payload[:18]
        )
        return json.dumps({
            'type': 'status',
            'cpu_temp': cpu_temp,
            'wifi_rssi': wifi_rssi,
            'free_heap': free_heap,
            'lidar_status': lidar_status,
            'imu_count': imu_count,
            'cloud_count': cloud_count
        }).encode()
    return None


def convert_motor_to_webui(packet):
    """转换电机状态为 WebUI JSON 格式"""
    payload = packet['payload']
    
    if len(payload) >= 10:  # MotorStatus size
        speedA, speedB, stateA, stateB, currentA, currentB = struct.unpack(
            '<hhBBHH', payload[:10]
        )
        return json.dumps({
            'type': 'motor',
            'speedA': speedA,
            'speedB': speedB,
            'stateA': stateA,
            'stateB': stateB,
            'currentA': currentA,
            'currentB': currentB
        }).encode()
    return None


def convert_webui_to_esp32(data):
    """
    转换 WebUI 控制命令为 ESP32 格式
    
    WebUI 发送 JSON: { motor: { speedA, speedB, stop, brake } }
    ESP32 期望: DataPacket { magic, timestamp, type=DATA_CONTROL, length, JSON_payload }
    """
    try:
        # 尝试解析为 JSON
        if isinstance(data, bytes):
            data = data.decode('utf-8')
        
        cmd = json.loads(data)
        json_bytes = json.dumps(cmd).encode('utf-8')
        
        # 构建 ESP32 DataPacket
        header = struct.pack('<IIBH', ESP32_MAGIC, 0, DATA_CONTROL, len(json_bytes))
        return header + json_bytes
        
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None


# ==================== ESP32 处理器 ====================
async def handle_esp32(websocket, path=None):
    """处理 ESP32 连接"""
    global message_count
    
    client_addr = websocket.remote_address
    esp32_clients.add(websocket)
    print(f"[ESP32] ✓ 连接: {client_addr}")
    
    try:
        async for message in websocket:
            message_count += 1
            
            # 文本消息 - 直接打印
            if isinstance(message, str):
                print(f"[ESP32] Text: {message}")
                continue
            
            # 二进制消息 - 解析并转发
            packet = parse_esp32_packet(message)
            if not packet:
                continue
            
            # 根据类型转换并转发到 WebUI
            webui_data = None
            
            if packet['type'] == DATA_POINTCLOUD:
                webui_data = convert_pointcloud_to_webui(packet)
            elif packet['type'] == DATA_STATUS:
                webui_data = convert_status_to_webui(packet)
            elif packet['type'] == DATA_MOTOR:
                webui_data = convert_motor_to_webui(packet)
            elif packet['type'] == DATA_IMU:
                # IMU 数据频率太高，可选择性转发
                pass
            
            if webui_data:
                # 广播给所有 WebUI 客户端
                for client in webui_clients.copy():
                    try:
                        await client.send(webui_data)
                    except websockets.exceptions.ConnectionClosed:
                        webui_clients.discard(client)
                        
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        esp32_clients.discard(websocket)
        print(f"[ESP32] 断开: {client_addr}")


# ==================== WebUI 处理器 ====================
async def handle_webui(websocket, path=None):
    """处理 WebUI 连接"""
    client_addr = websocket.remote_address
    webui_clients.add(websocket)
    print(f"[WebUI] ✓ 连接: {client_addr}")
    
    try:
        async for message in websocket:
            # WebUI 发送控制命令
            esp32_data = convert_webui_to_esp32(message)
            
            if esp32_data:
                # 转发给所有 ESP32 客户端
                for client in esp32_clients.copy():
                    try:
                        await client.send(esp32_data)
                    except websockets.exceptions.ConnectionClosed:
                        esp32_clients.discard(client)
                        
                print(f"[WebUI] Command forwarded: {message[:100]}...")
                
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        webui_clients.discard(websocket)
        print(f"[WebUI] 断开: {client_addr}")


# ==================== 主程序 ====================
async def main():
    print()
    print("╔══════════════════════════════════════════════════════╗")
    print("║   WebSocket Relay Server                             ║")
    print("║   ESP32 ↔ ws_server.py ↔ WebUI                       ║")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║   ESP32 Port: {ESP32_PORT:<6} (via wokwigw gateway)          ║")
    print(f"║   WebUI Port: {WEBUI_PORT:<6} (browser connection)           ║")
    print("╚══════════════════════════════════════════════════════╝")
    print()
    print("[INFO] 按 Ctrl+C 停止")
    print()
    
    # 启动两个 WebSocket 服务器
    esp32_server = await websockets.serve(handle_esp32, "0.0.0.0", ESP32_PORT)
    webui_server = await websockets.serve(handle_webui, "0.0.0.0", WEBUI_PORT)
    
    print(f"[OK] ESP32 服务器启动: ws://0.0.0.0:{ESP32_PORT}")
    print(f"[OK] WebUI 服务器启动: ws://0.0.0.0:{WEBUI_PORT}")
    print()
    print("[WAIT] 等待连接...")
    print()
    
    # 保持运行
    await asyncio.gather(
        esp32_server.wait_closed(),
        webui_server.wait_closed()
    )


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] 服务器已停止")

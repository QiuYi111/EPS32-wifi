#!/usr/bin/env python3
"""
ESP32 LiDAR Bridge + Motor WebSocket 客户端
配合新的 main.cpp 混合架构使用

功能:
1. 连接 ESP32 WebSocket
2. 接收透传的 LiDAR MAVLink 数据并解析
3. 发送电机控制命令
4. 显示系统状态
"""

import asyncio
import struct
import time
import math
from collections import deque

try:
    import websockets
except ImportError:
    print("请安装 websockets: pip install websockets")
    exit(1)

# ================= 配置 =================
ESP_IP = "192.168.4.1"
WS_PORT = 81
WS_URL = f"ws://{ESP_IP}:{WS_PORT}"

# 数据包魔数和类型
PACKET_MAGIC = 0xDEADBEEF

class PacketType:
    LIDAR_RAW = 1
    MOTOR_STATUS = 2
    SYSTEM_STATUS = 3
    MOTOR_CONTROL = 4

# MAVLink 配置
MAVLINK_MAGIC = 0xFD
MSG_ID_DISTANCE = 16
MSG_ID_AUXILIARY = 17
MSG_ID_IMU = 19


class MAVLinkParser:
    """MAVLink v2 解析器 (从透传数据中解析)"""
    
    def __init__(self):
        self.buffer = bytearray()
        self.stats = {
            'imu': 0,
            'distance': 0,
            'auxiliary': 0,
            'unknown': set()
        }
        self.aux_cache = {}
        self.dist_cache = {}
        self.cloud_count = 0
    
    def feed(self, data):
        """喂入原始数据"""
        self.buffer.extend(data)
        self._parse()
    
    def _parse(self):
        """解析缓冲区中的 MAVLink 包"""
        while len(self.buffer) >= 12:
            # 找帧头
            try:
                idx = self.buffer.index(MAVLINK_MAGIC)
                if idx > 0:
                    del self.buffer[:idx]
            except ValueError:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return
            
            if len(self.buffer) < 12:
                return
            
            # 解析帧头
            payload_len = self.buffer[1]
            incompat_flags = self.buffer[2]
            has_signature = (incompat_flags & 0x01) != 0
            packet_len = 10 + payload_len + 2 + (13 if has_signature else 0)
            
            if len(self.buffer) < packet_len:
                return
            
            packet = self.buffer[:packet_len]
            del self.buffer[:packet_len]
            
            # 消息 ID
            msg_id = packet[7] | (packet[8] << 8) | (packet[9] << 16)
            payload = packet[10:10 + payload_len]
            
            self._dispatch(msg_id, payload)
    
    def _dispatch(self, msg_id, payload):
        """分发消息"""
        if msg_id == MSG_ID_IMU:
            self.stats['imu'] += 1
        elif msg_id == MSG_ID_DISTANCE:
            self.stats['distance'] += 1
            self._handle_distance(payload)
        elif msg_id == MSG_ID_AUXILIARY:
            self.stats['auxiliary'] += 1
            self._handle_auxiliary(payload)
        else:
            self.stats['unknown'].add(msg_id)
    
    def _handle_distance(self, payload):
        if len(payload) < 246:
            return
        packet_id = struct.unpack('<H', payload[0:2])[0]
        self.dist_cache[packet_id] = payload[6:246]
        self._try_combine(packet_id)
    
    def _handle_auxiliary(self, payload):
        if len(payload) < 209:
            return
        packet_id = struct.unpack('<H', payload[84:86])[0]
        self.aux_cache[packet_id] = {
            'com_horizontal_angle_start': struct.unpack('<f', payload[20:24])[0],
            'com_horizontal_angle_step': struct.unpack('<f', payload[24:28])[0],
            'sys_vertical_angle_start': struct.unpack('<f', payload[28:32])[0],
            'sys_vertical_angle_span': struct.unpack('<f', payload[32:36])[0],
            'b_axis_dist': struct.unpack('<f', payload[72:76])[0],
            'theta_angle': struct.unpack('<f', payload[76:80])[0],
            'ksi_angle': struct.unpack('<f', payload[80:84])[0],
            'reflect_data': payload[89:209]
        }
        self._try_combine(packet_id)
    
    def _try_combine(self, packet_id):
        if packet_id in self.aux_cache and packet_id in self.dist_cache:
            self.cloud_count += 1
            del self.aux_cache[packet_id]
            del self.dist_cache[packet_id]
            # 清理旧缓存
            if len(self.aux_cache) > 50:
                oldest = sorted(self.aux_cache.keys())[:10]
                for k in oldest:
                    del self.aux_cache[k]
            if len(self.dist_cache) > 50:
                oldest = sorted(self.dist_cache.keys())[:10]
                for k in oldest:
                    del self.dist_cache[k]


class ESP32Client:
    """ESP32 WebSocket 客户端"""
    
    def __init__(self):
        self.mavlink = MAVLinkParser()
        self.last_system_status = None
        self.last_motor_status = None
        self.bytes_received = 0
        self.start_time = time.time()
        self.last_stats_time = time.time()
    
    def parse_packet(self, data):
        """解析 ESP32 数据包"""
        if len(data) < 11:  # PacketHeader size
            return
        
        magic, timestamp, pkt_type, length = struct.unpack('<IIBH', data[:11])
        
        if magic != PACKET_MAGIC:
            print(f"[!] Invalid magic: 0x{magic:08X}")
            return
        
        payload = data[11:11+length]
        
        if pkt_type == PacketType.LIDAR_RAW:
            self.mavlink.feed(payload)
            
        elif pkt_type == PacketType.SYSTEM_STATUS:
            if len(payload) >= 18:
                self.last_system_status = struct.unpack('<fBHIIB', payload[:18])
                
        elif pkt_type == PacketType.MOTOR_STATUS:
            if len(payload) >= 6:
                self.last_motor_status = struct.unpack('<hhBB', payload[:6])
    
    def create_motor_command(self, speed_a, speed_b, brake_a=False, brake_b=False):
        """创建电机控制命令"""
        flags = (1 if brake_a else 0) | (2 if brake_b else 0)
        
        header = struct.pack('<IIBH', 
            PACKET_MAGIC,
            int(time.time() * 1000) & 0xFFFFFFFF,
            PacketType.MOTOR_CONTROL,
            5  # MotorControlCmd size
        )
        payload = struct.pack('<hhB', 
            max(-1024, min(1024, speed_a)),
            max(-1024, min(1024, speed_b)),
            flags
        )
        return header + payload
    
    def print_stats(self):
        """打印统计信息"""
        now = time.time()
        if now - self.last_stats_time < 1.0:
            return
        
        elapsed = now - self.start_time
        bw_kb = self.bytes_received / 1024
        
        print(f"\n{'='*60}")
        print(f"[Stats] Elapsed: {elapsed:.1f}s, Total: {bw_kb:.1f} KB")
        
        # MAVLink 统计
        stats = self.mavlink.stats
        print(f"  MAVLink: IMU={stats['imu']}, Distance={stats['distance']}, "
              f"Auxiliary={stats['auxiliary']}, Clouds={self.mavlink.cloud_count}")
        
        if stats['unknown']:
            print(f"  Unknown MSG IDs: {stats['unknown']}")
        
        # 系统状态
        if self.last_system_status:
            cpu, rssi, heap, lidar_bytes, uptime, clients = self.last_system_status
            print(f"  ESP32: CPU={cpu:.1f}°C, Heap={heap}KB, "
                  f"LiDAR={lidar_bytes/1024:.1f}KB, Uptime={uptime}s, Clients={clients}")
        
        # 电机状态
        if self.last_motor_status:
            sa, sb, sta, stb = self.last_motor_status
            state_names = ['STOP', 'FWD', 'REV', 'BRAKE']
            print(f"  Motor: A={sa}({state_names[sta]}), B={sb}({state_names[stb]})")
        
        # 重置计数
        self.mavlink.stats = {'imu': 0, 'distance': 0, 'auxiliary': 0, 'unknown': set()}
        self.mavlink.cloud_count = 0
        self.bytes_received = 0
        self.last_stats_time = now


async def main():
    print(f"Connecting to {WS_URL}...")
    
    client = ESP32Client()
    
    try:
        async with websockets.connect(WS_URL, max_size=1024*1024) as ws:
            print("Connected!\n")
            print("Commands:")
            print("  Press Ctrl+C to exit")
            print("  (Motor control can be added via async input)\n")
            
            while True:
                try:
                    data = await asyncio.wait_for(ws.recv(), timeout=0.1)
                    if isinstance(data, bytes):
                        client.bytes_received += len(data)
                        client.parse_packet(data)
                except asyncio.TimeoutError:
                    pass
                
                client.print_stats()
                
    except ConnectionRefusedError:
        print(f"Connection refused. Is ESP32 running at {ESP_IP}?")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    asyncio.run(main())

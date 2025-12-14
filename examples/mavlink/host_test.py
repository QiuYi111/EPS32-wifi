#!/usr/bin/env python3
"""
Unitree LiDAR L1 数据解析工具
使用 MAVLink v2 协议，参考 unitree_lidar_sdk

消息 ID（来自 SDK 头文件）：
- MAVLINK_MSG_ID_RET_LIDAR_DISTANCE_DATA_PACKET = 16 (点云距离数据)
- MAVLINK_MSG_ID_RET_LIDAR_AUXILIARY_DATA_PACKET = 17 (辅助数据+反射率)
- MAVLINK_MSG_ID_RET_IMU_ATTITUDE_DATA_PACKET = 19 (IMU 数据)
"""

import socket
import struct
import time
import math
from collections import deque

# ================= 配置 =================
ESP_IP = "192.168.4.1"  # ESP32 默认 AP 地址
ESP_PORT = 8888
BUFFER_SIZE = 65536

# MAVLink v2 Magic Byte
MAGIC_V2 = 0xFD

# Unitree LiDAR 消息 ID（正确的 ID，来自 SDK）
MSG_ID_DISTANCE = 16   # RET_LIDAR_DISTANCE_DATA_PACKET
MSG_ID_AUXILIARY = 17  # RET_LIDAR_AUXILIARY_DATA_PACKET
MSG_ID_IMU = 19        # RET_IMU_ATTITUDE_DATA_PACKET

# Payload 长度
PAYLOAD_LEN_DISTANCE = 246   # 6 bytes header + 240 bytes point_data
PAYLOAD_LEN_AUXILIARY = 209  # 复杂结构，见 SDK
PAYLOAD_LEN_IMU = 42         # 4*4 + 3*4 + 3*4 + 2 = 42 bytes


class UnitreeParser:
    """Unitree LiDAR MAVLink v2 解析器"""
    
    def __init__(self):
        self.buffer = bytearray()
        self.imu_count = 0
        self.distance_count = 0
        self.auxiliary_count = 0
        self.start_time = time.time()
        self.total_bytes = 0
        self.unknown_msg_ids = set()
        
        # 缓存 auxiliary 和 distance 数据，用于匹配
        self.aux_cache = {}  # packet_id -> auxiliary data
        self.dist_cache = {}  # packet_id -> distance data
        
        self.cloud_count = 0  # 成功组合的点云数量

    def process(self, chunk):
        """处理接收到的数据块"""
        self.buffer.extend(chunk)
        self.total_bytes += len(chunk)
        
        while len(self.buffer) >= 12:  # MAVLink v2 最小帧头10 + CRC 2
            # 1. 寻找帧头 (0xFD)
            try:
                idx = self.buffer.index(MAGIC_V2)
                if idx > 0:
                    # 丢弃帧头前的垃圾数据
                    del self.buffer[:idx]
            except ValueError:
                # 没找到帧头，保留最后一个字节（可能是帧头的一部分）
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return

            if len(self.buffer) < 12:
                return
                
            # 2. 解析 MAVLink v2 帧头
            # [STX(1)] [LEN(1)] [INC_FLAGS(1)] [CMP_FLAGS(1)] [SEQ(1)] [SYS_ID(1)] [COMP_ID(1)] [MSG_ID(3)] [PAYLOAD] [CRC(2)]
            payload_len = self.buffer[1]
            incompat_flags = self.buffer[2]
            
            # 计算包总长度
            # 基本长度 = header(10) + payload(len) + checksum(2)
            # 如果有签名 = + signature(13)
            has_signature = (incompat_flags & 0x01) != 0
            packet_len = 10 + payload_len + 2 + (13 if has_signature else 0)
            
            if len(self.buffer) < packet_len:
                return  # 数据不够，等下一波

            # 3. 提取完整包
            packet = self.buffer[:packet_len]
            del self.buffer[:packet_len]
            
            # 4. 解析消息 ID (3字节，小端)
            msg_id = packet[7] | (packet[8] << 8) | (packet[9] << 16)
            
            # 5. 提取 payload
            payload = packet[10:10 + payload_len]
            
            # 6. 解码消息
            self.decode_msg(msg_id, payload)

    def decode_msg(self, msg_id, payload):
        """解析具体消息类型"""
        
        if msg_id == MSG_ID_IMU:  # 19: IMU Attitude Data
            self.imu_count += 1
            self.decode_imu(payload)
            
        elif msg_id == MSG_ID_DISTANCE:  # 16: Distance Data
            self.distance_count += 1
            self.decode_distance(payload)
            
        elif msg_id == MSG_ID_AUXILIARY:  # 17: Auxiliary Data
            self.auxiliary_count += 1
            self.decode_auxiliary(payload)
        else:
            self.unknown_msg_ids.add(msg_id)

    def decode_imu(self, payload):
        """
        解析 IMU 数据
        结构（来自 SDK）：
        - quaternion[4]: float[4], offset 0
        - angular_velocity[3]: float[3], offset 16
        - linear_acceleration[3]: float[3], offset 28  
        - packet_id: uint16, offset 40
        """
        if len(payload) < 42:
            return
            
        try:
            # 解包四元数 (x, y, z, w)
            quat = struct.unpack('<4f', payload[0:16])
            # 角速度
            angular_vel = struct.unpack('<3f', payload[16:28])
            # 线性加速度
            linear_acc = struct.unpack('<3f', payload[28:40])
            # packet_id
            packet_id = struct.unpack('<H', payload[40:42])[0]
            
            # 可以打印或处理 IMU 数据
            # print(f"IMU [{packet_id}] Quat: {quat}")
        except struct.error:
            pass

    def decode_distance(self, payload):
        """
        解析距离数据包
        结构（来自 SDK）：
        - packet_id: uint16, offset 0
        - packet_cnt: uint16, offset 2
        - payload_size: uint16, offset 4
        - point_data[240]: uint8[240], offset 6  (120个点，每点2字节)
        """
        if len(payload) < 246:
            return
            
        try:
            packet_id = struct.unpack('<H', payload[0:2])[0]
            packet_cnt = struct.unpack('<H', payload[2:4])[0]
            payload_size = struct.unpack('<H', payload[4:6])[0]
            point_data = payload[6:246]  # 240 bytes = 120 points * 2 bytes
            
            # 存入缓存
            self.dist_cache[packet_id] = {
                'packet_cnt': packet_cnt,
                'payload_size': payload_size,
                'point_data': point_data
            }
            
            # 尝试与 auxiliary 匹配
            self.try_combine_cloud(packet_id)
            
        except struct.error:
            pass

    def decode_auxiliary(self, payload):
        """
        解析辅助数据包
        结构（来自 SDK mavlink_msg_ret_lidar_auxiliary_data_packet.h）：
        - lidar_sync_delay_time: uint32, offset 0
        - time_stamp_s_step: uint32, offset 4
        - time_stamp_us_step: uint32, offset 8
        - sys_rotation_period: uint32, offset 12
        - com_rotation_period: uint32, offset 16
        - com_horizontal_angle_start: float, offset 20
        - com_horizontal_angle_step: float, offset 24
        - sys_vertical_angle_start: float, offset 28
        - sys_vertical_angle_span: float, offset 32
        - apd_temperature: float, offset 36
        - dirty_index: float, offset 40
        - imu_temperature: float, offset 44
        - up_optical_q: float, offset 48
        - down_optical_q: float, offset 52
        - apd_voltage: float, offset 56
        - imu_angle_x_offset: float, offset 60
        - imu_angle_y_offset: float, offset 64
        - imu_angle_z_offset: float, offset 68
        - b_axis_dist: float, offset 72
        - theta_angle: float, offset 76
        - ksi_angle: float, offset 80
        - packet_id: uint16, offset 84
        - payload_size: uint16, offset 86
        - lidar_work_status: uint8, offset 88
        - reflect_data[120]: uint8[120], offset 89
        """
        if len(payload) < 209:
            return
            
        try:
            # 解析关键字段
            com_horizontal_angle_start = struct.unpack('<f', payload[20:24])[0]
            com_horizontal_angle_step = struct.unpack('<f', payload[24:28])[0]
            sys_vertical_angle_start = struct.unpack('<f', payload[28:32])[0]
            sys_vertical_angle_span = struct.unpack('<f', payload[32:36])[0]
            
            b_axis_dist = struct.unpack('<f', payload[72:76])[0]
            theta_angle = struct.unpack('<f', payload[76:80])[0]
            ksi_angle = struct.unpack('<f', payload[80:84])[0]
            
            packet_id = struct.unpack('<H', payload[84:86])[0]
            lidar_work_status = payload[88]
            reflect_data = payload[89:209]  # 120 bytes
            
            # 存入缓存
            self.aux_cache[packet_id] = {
                'com_horizontal_angle_start': com_horizontal_angle_start,
                'com_horizontal_angle_step': com_horizontal_angle_step,
                'sys_vertical_angle_start': sys_vertical_angle_start,
                'sys_vertical_angle_span': sys_vertical_angle_span,
                'b_axis_dist': b_axis_dist,
                'theta_angle': theta_angle,
                'ksi_angle': ksi_angle,
                'lidar_work_status': lidar_work_status,
                'reflect_data': reflect_data
            }
            
            # 尝试与 distance 匹配
            self.try_combine_cloud(packet_id)
            
        except struct.error:
            pass

    def try_combine_cloud(self, packet_id):
        """尝试组合点云数据（当 distance 和 auxiliary 都收到时）"""
        if packet_id not in self.aux_cache or packet_id not in self.dist_cache:
            return
            
        aux = self.aux_cache.pop(packet_id)
        dist = self.dist_cache.pop(packet_id)
        
        # 转换点云（参考 SDK 的 parseRangeAuxiliaryDataToCloud）
        points = self.parse_to_cloud(aux, dist)
        self.cloud_count += 1
        
        # 这里可以处理点云数据
        # print(f"Cloud [{packet_id}]: {len(points)} points")
        
        # 清理旧的缓存（防止内存泄漏）
        self.cleanup_cache()

    def parse_to_cloud(self, aux, dist):
        """
        将 distance + auxiliary 数据解析为 XYZ 点云
        参考 SDK 的 parseRangeAuxiliaryDataToCloud 函数
        """
        points = []
        
        range_scale = 0.001  # mm to m
        z_bias = 0.0445
        points_num = 120
        bias_laser_beam = aux['b_axis_dist'] / 1000.0
        
        sin_theta = math.sin(aux['theta_angle'])
        cos_theta = math.cos(aux['theta_angle'])
        sin_ksi = math.sin(aux['ksi_angle'])
        cos_ksi = math.cos(aux['ksi_angle'])
        
        pitch_cur = aux['sys_vertical_angle_start'] * math.pi / 180.0
        pitch_step = aux['sys_vertical_angle_span'] * math.pi / 180.0
        yaw_cur = aux['com_horizontal_angle_start'] * math.pi / 180.0
        yaw_step = aux['com_horizontal_angle_step'] / points_num * math.pi / 180.0
        
        point_data = dist['point_data']
        reflect_data = aux['reflect_data']
        
        for j in range(points_num):
            i = j * 2
            # 读取距离（小端 uint16）
            range_val = point_data[i] | (point_data[i + 1] << 8)
            
            if range_val == 0:
                # 无效点
                pitch_cur += pitch_step
                yaw_cur += yaw_step
                continue
            
            range_float = range_scale * range_val
            
            # 坐标变换
            sin_alpha = math.sin(pitch_cur)
            cos_alpha = math.cos(pitch_cur)
            sin_beta = math.sin(yaw_cur)
            cos_beta = math.cos(yaw_cur)
            
            A = (-cos_theta * sin_ksi + sin_theta * sin_alpha * cos_ksi) * range_float + bias_laser_beam
            B = cos_alpha * cos_ksi * range_float
            
            x = cos_beta * A - sin_beta * B
            y = sin_beta * A + cos_beta * B
            z = (sin_theta * sin_ksi + cos_theta * sin_alpha * cos_ksi) * range_float + z_bias
            
            intensity = reflect_data[j] if j < len(reflect_data) else 0
            
            points.append((x, y, z, intensity))
            
            pitch_cur += pitch_step
            yaw_cur += yaw_step
        
        return points

    def cleanup_cache(self):
        """清理过期的缓存"""
        # 保留最近 100 个包
        max_cache = 100
        if len(self.aux_cache) > max_cache:
            oldest_keys = sorted(self.aux_cache.keys())[:len(self.aux_cache) - max_cache]
            for k in oldest_keys:
                del self.aux_cache[k]
        if len(self.dist_cache) > max_cache:
            oldest_keys = sorted(self.dist_cache.keys())[:len(self.dist_cache) - max_cache]
            for k in oldest_keys:
                del self.dist_cache[k]

    def print_stats(self):
        """打印统计信息"""
        now = time.time()
        if now - self.start_time >= 1.0:
            bw_kb = self.total_bytes / 1024.0
            elapsed = now - self.start_time
            
            print(f"\n[PC Stats] Elapsed: {elapsed:.1f}s")
            print(f"  Bandwidth: {bw_kb:.1f} KB/s")
            print(f"  IMU: {self.imu_count} Hz")
            print(f"  Distance Pkts: {self.distance_count}")
            print(f"  Auxiliary Pkts: {self.auxiliary_count}")
            print(f"  Combined Clouds: {self.cloud_count}")
            
            if self.unknown_msg_ids:
                print(f"  Unknown MSG IDs: {self.unknown_msg_ids}")
            
            if bw_kb > 10 and self.cloud_count == 0:
                print(">>> 警告: 有大量数据但没解出点云，可能雷达在待机模式")
                
            # 重置计数器
            self.imu_count = 0
            self.distance_count = 0
            self.auxiliary_count = 0
            self.cloud_count = 0
            self.total_bytes = 0
            self.unknown_msg_ids.clear()
            self.start_time = now


# ================= 主程序 =================

def main():
    print(f"Connecting to ESP32 at {ESP_IP}:{ESP_PORT}...")
    print("消息 ID 定义：")
    print(f"  - Distance (点云距离): {MSG_ID_DISTANCE}")
    print(f"  - Auxiliary (辅助数据): {MSG_ID_AUXILIARY}")
    print(f"  - IMU: {MSG_ID_IMU}")
    print()
    
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((ESP_IP, ESP_PORT))
        print("Connected! Waiting for raw stream...\n")
        
        parser = UnitreeParser()

        while True:
            try:
                data = s.recv(BUFFER_SIZE)
                if not data:
                    print("Connection closed by server")
                    break
                
                parser.process(data)
                parser.print_stats()
                
            except socket.error as e:
                print(f"Socket error: {e}")
                break
    except Exception as e:
        print(f"Connection failed: {e}")
    finally:
        s.close()


if __name__ == "__main__":
    main()
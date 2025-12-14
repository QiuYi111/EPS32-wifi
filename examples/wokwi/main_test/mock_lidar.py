#!/usr/bin/env python3
# /// script
# requires-python = ">=3.8"
# dependencies = ["pyserial"]
# ///
"""
Mock Unitree LiDAR L1 - Python 版本

通过 RFC2217 向 Wokwi ESP32 的 Serial1 发送 MAVLink 数据。
复用 mock_lidar/src/main.cpp 的逻辑。

数据格式:
- IMU 姿态数据: 200Hz (MSG ID 19)
- 点云辅助数据: 10Hz (MSG ID 22)  
- 点云距离数据: 10Hz (MSG ID 21)

MAVLink v1 帧格式:
  [0xFE] [len] [seq] [sys_id] [comp_id] [msg_id] [payload...] [crc_low] [crc_high]

使用方法:
  uv run examples/wokwi/main_test/mock_lidar.py
"""

import struct
import time
import math
import threading
import sys

# ==================== MAVLink 常量 ====================
MAVLINK_STX = 0xFE  # MAVLink v1 起始字节
SYSTEM_ID = 1
COMPONENT_ID = 200  # IMU component

# 消息 ID (与 Unitree SDK 头文件一致)
MSG_ID_LIDAR_DISTANCE = 16  # RET_LIDAR_DISTANCE_DATA_PACKET
MSG_ID_LIDAR_AUX = 17       # RET_LIDAR_AUXILIARY_DATA_PACKET
MSG_ID_IMU = 19             # RET_IMU_ATTITUDE_DATA_PACKET

# CRC Extra (from mavlink headers)
CRC_EXTRA_LIDAR_DISTANCE = 74   # MSG_ID_16_CRC
CRC_EXTRA_LIDAR_AUX = 99        # MSG_ID_17_CRC
CRC_EXTRA_IMU = 110             # MSG_ID_19_CRC

# ==================== CRC 计算 ====================
def x25_crc_accumulate(data, crc=0xFFFF):
    """MAVLink X.25 CRC 计算"""
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
        crc &= 0xFFFF
    return crc

def mavlink_crc(payload, msg_id, crc_extra):
    """计算完整的 MAVLink CRC"""
    crc = x25_crc_accumulate(payload)
    crc = x25_crc_accumulate([crc_extra], crc)
    return crc

# ==================== MAVLink 消息打包 ====================
class MavlinkPacker:
    def __init__(self):
        self.seq = 0
    
    def pack_message(self, msg_id, payload, crc_extra):
        """打包 MAVLink v1 消息"""
        header = struct.pack('<BBBBB',
            len(payload),  # payload length
            self.seq,      # sequence
            SYSTEM_ID,     # system id
            COMPONENT_ID,  # component id
            msg_id         # message id
        )
        
        # 计算 CRC
        crc_data = header + payload
        crc = x25_crc_accumulate(list(crc_data))
        crc = x25_crc_accumulate([crc_extra], crc)
        
        # 完整消息
        message = bytes([MAVLINK_STX]) + header + payload + struct.pack('<H', crc)
        
        self.seq = (self.seq + 1) & 0xFF
        return message
    
    def pack_imu(self, packet_id, quaternion, angular_velocity, linear_acceleration):
        """
        打包 IMU 姿态数据包 (MSG ID 19)
        
        Payload 结构 (42 bytes):
          - quaternion[4]: float[4] @ offset 0
          - angular_velocity[3]: float[3] @ offset 16
          - linear_acceleration[3]: float[3] @ offset 28
          - packet_id: uint16 @ offset 40
        """
        payload = struct.pack('<4f3f3fH',
            quaternion[0], quaternion[1], quaternion[2], quaternion[3],
            angular_velocity[0], angular_velocity[1], angular_velocity[2],
            linear_acceleration[0], linear_acceleration[1], linear_acceleration[2],
            packet_id
        )
        return self.pack_message(MSG_ID_IMU, payload, CRC_EXTRA_IMU)
    
    def pack_lidar_aux(self, packet_id, aux_data):
        """
        打包 LiDAR 辅助数据包 (MSG ID 22)
        
        简化版: 只发送必要字段
        """
        # 简化的辅助数据包 - 根据实际协议调整
        payload = bytearray(180)  # 预分配
        
        # 基本参数
        struct.pack_into('<H', payload, 0, packet_id)  # packet_id
        struct.pack_into('<B', payload, 2, 120)  # payload_size (120 points)
        struct.pack_into('<B', payload, 3, 1)  # lidar_work_status
        
        # 角度参数
        struct.pack_into('<f', payload, 20, aux_data.get('horizontal_angle_start', 0.0))
        struct.pack_into('<f', payload, 24, aux_data.get('horizontal_angle_step', 3.0))
        struct.pack_into('<f', payload, 28, aux_data.get('vertical_angle_start', -15.0))
        struct.pack_into('<f', payload, 32, aux_data.get('vertical_angle_span', 0.25))
        
        # 反射率数据 (120 bytes @ offset 60)
        for i in range(120):
            payload[60 + i] = aux_data.get('reflect_data', [128] * 120)[i]
        
        return self.pack_message(MSG_ID_LIDAR_AUX, bytes(payload), CRC_EXTRA_LIDAR_AUX)
    
    def pack_lidar_distance(self, packet_id, distances):
        """
        打包 LiDAR 距离数据包 (MSG ID 21)
        
        每个点 2 bytes (uint16, 单位 mm)
        """
        payload = bytearray(244)  # 预分配
        
        struct.pack_into('<H', payload, 0, packet_id)  # packet_id
        struct.pack_into('<B', payload, 2, 1)  # packet_cnt
        struct.pack_into('<B', payload, 3, 240)  # payload_size (120 * 2)
        
        # 距离数据 (120 points * 2 bytes @ offset 4)
        for i, dist in enumerate(distances[:120]):
            struct.pack_into('<H', payload, 4 + i * 2, int(dist))
        
        return self.pack_message(MSG_ID_LIDAR_DISTANCE, bytes(payload), CRC_EXTRA_LIDAR_DISTANCE)


# ==================== Mock LiDAR 生成器 ====================
class MockLidar:
    def __init__(self):
        self.packer = MavlinkPacker()
        self.packet_id = 0
        self.sim_time = 0.0
        self.rotation_angle = 0.0
        self.running = False
    
    def generate_imu_data(self):
        """生成动态 IMU 数据"""
        # 模拟缓慢摇摆
        roll = 0.1 * math.sin(self.sim_time * 0.5)
        pitch = 0.05 * math.sin(self.sim_time * 0.3)
        yaw = self.rotation_angle
        
        # 欧拉角转四元数
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        
        quaternion = [
            cr * cp * cy + sr * sp * sy,  # W
            sr * cp * cy - cr * sp * sy,  # X
            cr * sp * cy + sr * cp * sy,  # Y
            cr * cp * sy - sr * sp * cy   # Z
        ]
        
        angular_velocity = [
            0.1 * math.cos(self.sim_time * 0.5),
            0.05 * math.cos(self.sim_time * 0.3),
            0.5  # 持续旋转
        ]
        
        linear_acceleration = [
            0.1 * math.sin(self.sim_time * 2.0),
            0.1 * math.cos(self.sim_time * 2.0),
            9.81 + 0.05 * math.sin(self.sim_time * 5.0)
        ]
        
        return self.packer.pack_imu(self.packet_id, quaternion, angular_velocity, linear_acceleration)
    
    def generate_cloud_data(self):
        """生成点云数据 (aux + distance)"""
        # 辅助数据
        aux_data = {
            'horizontal_angle_start': self.rotation_angle * 180.0 / math.pi,
            'horizontal_angle_step': 3.0,
            'vertical_angle_start': -15.0,
            'vertical_angle_span': 0.25,
            'reflect_data': [80 + (i % 40) * 3 for i in range(120)]
        }
        
        # 距离数据 - 模拟矩形房间
        distances = []
        for i in range(120):
            angle = (i / 120.0) * 2.0 * math.pi
            x = math.cos(angle + self.rotation_angle)
            y = math.sin(angle + self.rotation_angle)
            
            if abs(x) > abs(y):
                dist_wall = 2000.0 / abs(x) if abs(x) > 0.01 else 5000
            else:
                dist_wall = 1500.0 / abs(y) if abs(y) > 0.01 else 5000
            
            # 添加噪声
            import random
            dist_wall += random.randint(-20, 20)
            distances.append(max(100, min(10000, dist_wall)))
        
        aux_msg = self.packer.pack_lidar_aux(self.packet_id, aux_data)
        dist_msg = self.packer.pack_lidar_distance(self.packet_id, distances)
        
        return aux_msg + dist_msg
    
    def update(self, dt=0.005):
        """更新模拟状态"""
        self.sim_time += dt
        self.rotation_angle += 0.0005
        if self.rotation_angle > 2.0 * math.pi:
            self.rotation_angle -= 2.0 * math.pi


# ==================== 主程序 ====================
def print_banner():
    print()
    print("╔══════════════════════════════════════════════════════╗")
    print("║   Mock Unitree LiDAR L1 - Python Version             ║")
    print("║   通过 RFC2217 发送 MAVLink 数据到 Wokwi ESP32       ║")
    print("╠══════════════════════════════════════════════════════╣")
    print("║   IMU: 200Hz | PointCloud: 10Hz                      ║")
    print("╚══════════════════════════════════════════════════════╝")
    print()


def run_with_serial(port_url="rfc2217://localhost:4000"):
    """通过串口发送数据"""
    import serial
    
    print(f"[INFO] 连接到 {port_url}")
    
    try:
        ser = serial.serial_for_url(port_url, baudrate=2000000, timeout=1)
        print("[OK] 串口连接成功")
    except Exception as e:
        print(f"[ERROR] 串口连接失败: {e}")
        print("[HINT] 确保 Wokwi 仿真正在运行且 RFC2217 端口已启用")
        return
    
    lidar = MockLidar()
    
    imu_interval = 0.005  # 200Hz
    cloud_interval = 0.1   # 10Hz
    
    last_imu_time = time.time()
    last_cloud_time = time.time()
    last_debug_time = time.time()
    
    print("[INFO] 开始发送数据...")
    print("[INFO] 按 Ctrl+C 停止")
    print()
    
    try:
        while True:
            now = time.time()
            
            # 200Hz IMU
            if now - last_imu_time >= imu_interval:
                imu_msg = lidar.generate_imu_data()
                ser.write(imu_msg)
                last_imu_time = now
                lidar.update(imu_interval)
            
            # 10Hz 点云 - 暂时禁用，先测试 IMU
            # if now - last_cloud_time >= cloud_interval:
            #     cloud_msg = lidar.generate_cloud_data()
            #     ser.write(cloud_msg)
            #     last_cloud_time = now
            #     lidar.packet_id += 1
            
            # 调试输出
            if now - last_debug_time >= 1.0:
                print(f"[Mock] IMU only test, Angle: {lidar.rotation_angle * 180 / math.pi:.1f}°")
                last_debug_time = now
            
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\n[INFO] 用户中断")
    finally:
        ser.close()
        print("[INFO] 串口已关闭")


def main():
    print_banner()
    
    # 检查命令行参数
    port_url = "rfc2217://localhost:4000"
    if len(sys.argv) > 1:
        port_url = sys.argv[1]
    
    print(f"[CONFIG] Port: {port_url}")
    print()
    
    run_with_serial(port_url)


if __name__ == "__main__":
    main()

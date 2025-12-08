#!/usr/bin/env python3
"""
ESP32 Robot Data Bridge - Data Receiver
接收并检查ESP32发送的数据包
"""

import socket
import struct
import time
import json
from dataclasses import dataclass
from typing import Dict, Any
import threading

# ==================== 数据结构定义 ====================
@dataclass
class ImuData:
    """IMU数据结构"""
    packet_id: int
    quaternion: list  # [w, x, y, z]
    angular_velocity: list  # [x, y, z] rad/s
    linear_acceleration: list  # [x, y, z] m/s²

@dataclass
class PointXYZI:
    """点云数据点"""
    x: float
    y: float
    z: float
    intensity: float

@dataclass
class MotorStatus:
    """电机状态"""
    speedA: int  # -1024~1024
    speedB: int
    stateA: int  # 0=停止 1=正转 2=反转 3=刹车
    stateB: int
    currentA: int  # mA
    currentB: int

@dataclass
class SystemStatus:
    """系统状态"""
    cpu_temp: float
    wifi_rssi: int
    free_heap: int
    lidar_status: int
    imu_count: int
    cloud_count: int

class ESP32DataReceiver:
    """ESP32数据接收器"""

    def __init__(self, host='0.0.0.0', port=8888):
        self.host = host
        self.port = port
        self.socket = None

        # 统计信息
        self.stats = {
            'total_packets': 0,
            'imu_packets': 0,
            'pointcloud_packets': 0,
            'motor_packets': 0,
            'status_packets': 0,
            'control_packets': 0,
            'invalid_packets': 0,
            'last_heartbeat': 0
        }

        # 最新数据
        self.latest_data = {
            'imu': None,
            'pointcloud': [],
            'motor': None,
            'status': None
        }

        self.running = False

    def parse_data_packet(self, data: bytes) -> Dict[str, Any]:
        """解析数据包"""
        if len(data) < 12:  # 最小数据包大小
            return None

        try:
            # 解析头部
            magic, timestamp, data_type, length = struct.unpack('<IBHB', data[:12])

            # 验证魔数
            if magic != 0xDEADBEEF:
                self.stats['invalid_packets'] += 1
                print(f"[Error] Invalid magic number: 0x{magic:08X}")
                return None

            # 提取数据负载
            payload = data[12:12+length]

            return {
                'magic': magic,
                'timestamp': timestamp,
                'type': data_type,
                'length': length,
                'payload': payload
            }

        except struct.error as e:
            self.stats['invalid_packets'] += 1
            print(f"[Error] Struct unpack error: {e}")
            return None

    def parse_imu_data(self, payload: bytes) -> ImuData:
        """解析IMU数据"""
        # 根据UnitreeMavlink.h中的ImuSample结构
        packet_id = struct.unpack('<H', payload[:2])[0]
        quaternion = struct.unpack('<4f', payload[4:20])
        angular_velocity = struct.unpack('<3f', payload[20:32])
        linear_acceleration = struct.unpack('<3f', payload[32:44])

        return ImuData(
            packet_id=packet_id,
            quaternion=list(quaternion),
            angular_velocity=list(angular_velocity),
            linear_acceleration=list(linear_acceleration)
        )

    def parse_pointcloud_data(self, payload: bytes) -> list[PointXYZI]:
        """解析点云数据"""
        points = []
        point_size = 16  # x(4) + y(4) + z(4) + intensity(4) = 16 bytes

        for i in range(0, len(payload), point_size):
            if i + point_size > len(payload):
                break

            x, y, z, intensity = struct.unpack('<4f', payload[i:i+point_size])
            points.append(PointXYZI(x=x, y=y, z=z, intensity=intensity))

        return points

    def parse_motor_status(self, payload: bytes) -> MotorStatus:
        """解析电机状态"""
        if len(payload) < 12:  # 最小大小
            return None

        speedA, speedB = struct.unpack('<hh', payload[:4])
        stateA, stateB = struct.unpack('<BB', payload[4:6])
        currentA, currentB = struct.unpack('<HH', payload[6:10])

        return MotorStatus(
            speedA=speedA,
            speedB=speedB,
            stateA=stateA,
            stateB=stateB,
            currentA=currentA,
            currentB=currentB
        )

    def parse_system_status(self, payload: bytes) -> SystemStatus:
        """解析系统状态"""
        if len(payload) < 20:  # 最小大小
            return None

        cpu_temp = struct.unpack('<f', payload[:4])[0]
        wifi_rssi = struct.unpack('<b', payload[4:5])[0]
        free_heap = struct.unpack('<H', payload[6:8])[0]
        lidar_status = struct.unpack('<B', payload[8:9])[0]
        imu_count = struct.unpack('<I', payload[12:16])[0]
        cloud_count = struct.unpack('<I', payload[16:20])[0]

        return SystemStatus(
            cpu_temp=cpu_temp,
            wifi_rssi=wifi_rssi,
            free_heap=free_heap,
            lidar_status=lidar_status,
            imu_count=imu_count,
            cloud_count=cloud_count
        )

    def handle_packet(self, packet: Dict[str, Any]):
        """处理接收到的数据包"""
        self.stats['total_packets'] += 1

        data_type = packet['type']
        payload = packet['payload']

        if data_type == 1:  # DATA_IMU
            self.stats['imu_packets'] += 1
            imu_data = self.parse_imu_data(payload)
            self.latest_data['imu'] = imu_data

            # 每秒打印一次IMU数据
            if self.stats['imu_packets'] % 200 == 0:  # 假设200Hz
                self.print_imu_data(imu_data)

        elif data_type == 2:  # DATA_POINTCLOUD
            self.stats['pointcloud_packets'] += 1
            points = self.parse_pointcloud_data(payload)
            self.latest_data['pointcloud'] = points

            # 每10个点云包打印一次
            if self.stats['pointcloud_packets'] % 10 == 0:
                print(f"[PointCloud] Packet {self.stats['pointcloud_packets']}: {len(points)} points")

        elif data_type == 3:  # DATA_MOTOR
            self.stats['motor_packets'] += 1
            motor_status = self.parse_motor_status(payload)
            if motor_status:
                self.latest_data['motor'] = motor_status

        elif data_type == 4:  # DATA_STATUS
            self.stats['status_packets'] += 1
            system_status = self.parse_system_status(payload)
            if system_status:
                self.latest_data['status'] = system_status
                self.print_system_status(system_status)

        elif data_type == 5:  # DATA_CONTROL (控制命令，通常不会收到)
            self.stats['control_packets'] += 1
            print(f"[Control] Received control command: {payload}")

    def print_imu_data(self, imu: ImuData):
        """打印IMU数据"""
        print(f"[IMU] Packet: {imu.packet_id}")
        print(f"  Quaternion: ({imu.quaternion[0]:.3f}, {imu.quaternion[1]:.3f}, {imu.quaternion[2]:.3f}, {imu.quaternion[3]:.3f})")
        print(f"  Angular Velocity: ({imu.angular_velocity[0]:.3f}, {imu.angular_velocity[1]:.3f}, {imu.angular_velocity[2]:.3f})")
        print(f"  Linear Accel: ({imu.linear_acceleration[0]:.3f}, {imu.linear_acceleration[1]:.3f}, {imu.linear_acceleration[2]:.3f})")

    def print_system_status(self, status: SystemStatus):
        """打印系统状态"""
        print(f"[Status] CPU: {status.cpu_temp:.1f}°C | WiFi: {status.wifi_rssi}dBm | Heap: {status.free_heap}B | LiDAR: {status.lidar_status}")
        print(f"  IMU count: {status.imu_count} | Cloud count: {status.cloud_count}")

    def print_heartbeat(self, message: str):
        """打印心跳"""
        self.stats['last_heartbeat'] = time.time()
        print(f"[Heartbeat] {message}")

    def print_statistics(self):
        """打印统计信息"""
        print(f"\n=== Statistics ===")
        print(f"Total packets: {self.stats['total_packets']}")
        print(f"  IMU: {self.stats['imu_packets']}")
        print(f"  PointCloud: {self.stats['pointcloud_packets']}")
        print(f"  Motor: {self.stats['motor_packets']}")
        print(f"  Status: {self.stats['status_packets']}")
        print(f"  Control: {self.stats['control_packets']}")
        print(f"  Invalid: {self.stats['invalid_packets']}")
        print(f"Last heartbeat: {time.ctime(self.stats['last_heartbeat']) if self.stats['last_heartbeat'] > 0 else 'Never'}")

    def send_control_command(self, motor_speed_a: int = 0, motor_speed_b: int = 0, stop: bool = False, brake: bool = False):
        """发送控制命令到ESP32"""
        command = {
            "motor": {
                "speedA": motor_speed_a,
                "speedB": motor_speed_b,
                "stop": stop,
                "brake": brake
            }
        }

        json_data = json.dumps(command).encode('utf-8')

        # 构建控制数据包
        magic = 0xDEADBEEF
        timestamp = int(time.time() * 1000)
        data_type = 5  # DATA_CONTROL
        length = len(json_data)

        packet = struct.pack('<IBHB', magic, timestamp, data_type, length) + json_data

        try:
            self.socket.sendto(packet, ('192.168.4.1', self.port))  # 假设ESP32 IP是192.168.4.1
            print(f"[Control] Sent: {command}")
        except Exception as e:
            print(f"[Error] Failed to send control command: {e}")

    def start(self):
        """启动接收器"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(1.0)  # 1秒超时

        print(f"🚀 ESP32 Data Receiver started on {self.host}:{self.port}")
        print("Waiting for ESP32 to connect...")
        print("\n可用命令:")
        print("  status - 显示统计信息")
        print("  imu - 显示最新IMU数据")
        print("  motor <speedA> <speedB> - 控制电机")
        print("  stop - 停止电机")
        print("  brake - 刹车电机")
        print("  quit - 退出")
        print("-" * 50)

        self.running = True

        # 启动输入线程
        input_thread = threading.Thread(target=self.input_handler, daemon=True)
        input_thread.start()

        last_stats_time = time.time()

        try:
            while self.running:
                try:
                    # 接收数据
                    data, addr = self.socket.recvfrom(2048)

                    # 检查是否是心跳包
                    if data.startswith(b'ESP32_ROBOT_HEARTBEAT'):
                        self.print_heartbeat(data.decode('utf-8', errors='ignore'))
                    else:
                        # 解析数据包
                        packet = self.parse_data_packet(data)
                        if packet:
                            self.handle_packet(packet)

                except socket.timeout:
                    pass
                except Exception as e:
                    print(f"[Error] Receive error: {e}")

                # 每10秒打印一次统计
                if time.time() - last_stats_time > 10:
                    self.print_statistics()
                    last_stats_time = time.time()

        except KeyboardInterrupt:
            print("\n\n[Info] Shutting down...")
        finally:
            self.running = False
            if self.socket:
                self.socket.close()
            self.print_statistics()

    def input_handler(self):
        """处理用户输入"""
        while self.running:
            try:
                user_input = input().strip().lower()

                if user_input == 'quit' or user_input == 'q':
                    self.running = False
                    break
                elif user_input == 'status' or user_input == 's':
                    self.print_statistics()
                elif user_input == 'imu' or user_input == 'i':
                    if self.latest_data['imu']:
                        self.print_imu_data(self.latest_data['imu'])
                    else:
                        print("No IMU data received yet")
                elif user_input.startswith('motor '):
                    parts = user_input.split()
                    if len(parts) == 3:
                        speed_a = int(parts[1])
                        speed_b = int(parts[2])
                        self.send_control_command(speed_a, speed_b)
                    else:
                        print("Usage: motor <speedA> <speedB>")
                elif user_input == 'stop':
                    self.send_control_command(stop=True)
                elif user_input == 'brake':
                    self.send_control_command(brake=True)
                elif user_input == 'help' or user_input == 'h':
                    print("可用命令: status, imu, motor <A> <B>, stop, brake, quit")

            except (EOFError, KeyboardInterrupt):
                self.running = False
                break
            except Exception as e:
                print(f"[Error] Input error: {e}")

def main():
    """主函数"""
    receiver = ESP32DataReceiver()
    receiver.start()

if __name__ == '__main__':
    main()
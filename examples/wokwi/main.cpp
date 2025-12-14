/**
 * @file main.cpp
 * @brief MAVLink 解析测试 - 验证 mock_lidar.py 能被正确解析
 */

#include <Arduino.h>
#include "UnitreeMavlink.h"

// 统计计数
unsigned long rx_bytes = 0;
unsigned long imu_count = 0;
unsigned long aux_count = 0;
unsigned long dist_count = 0;
unsigned long last_report = 0;

unitree::mav::UnitreeMavlinkParser parser;

// IMU 回调
void on_imu(const unitree::mav::ImuSample& imu) {
    imu_count++;
    if (imu_count % 50 == 1) {  // 每 50 个打印一次
        Serial.printf("[IMU #%lu] Q=(%.2f,%.2f,%.2f,%.2f)\n",
            imu_count, imu.quaternion[0], imu.quaternion[1], 
            imu.quaternion[2], imu.quaternion[3]);
    }
}

// LiDAR 辅助数据回调
void on_aux(const unitree::mav::LidarAuxPacket& aux) {
    aux_count++;
    Serial.printf("[AUX #%lu] h_angle_start=%.1f\n", aux_count, aux.com_horizontal_angle_start);
}

// LiDAR 距离数据回调
void on_dist(const unitree::mav::LidarDistancePacket& dist) {
    dist_count++;
    Serial.printf("[DIST #%lu] packet_id=%u, size=%u\n", dist_count, dist.packet_id, dist.payload_size);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== MAVLink Parser Test ===");
    Serial.println("Waiting for mock_lidar.py data...\n");
    
    // 设置回调
    parser.set_imu_callback(on_imu);
    parser.set_lidar_aux_callback(on_aux);
    parser.set_lidar_distance_callback(on_dist);
}

void loop() {
    // 读取串口数据并喂给解析器
    while (Serial.available() > 0) {
        uint8_t c = Serial.read();
        rx_bytes++;
        parser.feed(c);
    }
    
    // 每 3 秒报告状态
    if (millis() - last_report > 3000) {
        Serial.printf("[Status] RX=%lu bytes, IMU=%lu, AUX=%lu, DIST=%lu\n",
            rx_bytes, imu_count, aux_count, dist_count);
        last_report = millis();
    }
    
    delay(1);
}

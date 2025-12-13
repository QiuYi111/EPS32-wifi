/**
 * @file main.cpp
 * @brief Mock Unitree Lidar L1 for Wokwi Simulation
 * 
 * This MCU simulates the Unitree LiDAR L1 by generating MAVLink messages:
 * - IMU attitude data at 200Hz
 * - Point cloud (aux + distance) at 10Hz
 * 
 * Connect TX (GPIO17) to the main ESP32's RX (GPIO18)
 */

#include <Arduino.h>
#include <mavlink/SysMavlink/mavlink.h>

// ==================== 配置参数 ====================
// UART配置 - 与主项目匹配
#define UART_TX_PIN 17
#define UART_RX_PIN 18
#define UART_BAUD 2000000

// MAVLink系统标识
#define SYSTEM_ID 1
#define COMPONENT_ID 200  // IMU component

// 数据生成模式
enum SimulationMode {
    MODE_STATIC,   // 固定数据，用于基础功能验证
    MODE_DYNAMIC   // 动态数据，模拟真实运动
};

SimulationMode current_mode = MODE_DYNAMIC;

// 时序控制
#define IMU_INTERVAL_US 5000      // 200Hz = 5ms
#define CLOUD_INTERVAL_US 100000  // 10Hz = 100ms

// ==================== 全局状态 ====================
uint16_t packet_id = 0;
uint32_t last_imu_time = 0;
uint32_t last_cloud_time = 0;

// 动态模拟参数
float sim_time = 0.0f;
float rotation_angle = 0.0f;

// ==================== MAVLink 发送函数 ====================

/**
 * @brief 通过UART发送MAVLink消息
 */
void send_mavlink_message(mavlink_message_t* msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
    Serial1.write(buffer, len);
}

/**
 * @brief 生成并发送IMU姿态数据
 */
void send_imu_data() {
    mavlink_message_t msg;
    
    float quaternion[4];
    float angular_velocity[3];
    float linear_acceleration[3];
    
    if (current_mode == MODE_STATIC) {
        // 静态模式：固定姿态（水平）
        quaternion[0] = 1.0f;  // W
        quaternion[1] = 0.0f;  // X
        quaternion[2] = 0.0f;  // Y
        quaternion[3] = 0.0f;  // Z
        
        angular_velocity[0] = 0.0f;
        angular_velocity[1] = 0.0f;
        angular_velocity[2] = 0.0f;
        
        linear_acceleration[0] = 0.0f;
        linear_acceleration[1] = 0.0f;
        linear_acceleration[2] = 9.81f;  // 重力
    } else {
        // 动态模式：模拟缓慢摇摆
        float roll = 0.1f * sin(sim_time * 0.5f);   // 小幅滚转
        float pitch = 0.05f * sin(sim_time * 0.3f); // 小幅俯仰
        float yaw = rotation_angle;                  // 持续旋转
        
        // 欧拉角转四元数
        float cy = cos(yaw * 0.5f);
        float sy = sin(yaw * 0.5f);
        float cp = cos(pitch * 0.5f);
        float sp = sin(pitch * 0.5f);
        float cr = cos(roll * 0.5f);
        float sr = sin(roll * 0.5f);
        
        quaternion[0] = cr * cp * cy + sr * sp * sy;  // W
        quaternion[1] = sr * cp * cy - cr * sp * sy;  // X
        quaternion[2] = cr * sp * cy + sr * cp * sy;  // Y
        quaternion[3] = cr * cp * sy - sr * sp * cy;  // Z
        
        // 模拟角速度
        angular_velocity[0] = 0.1f * cos(sim_time * 0.5f);
        angular_velocity[1] = 0.05f * cos(sim_time * 0.3f);
        angular_velocity[2] = 0.5f;  // 持续旋转速度
        
        // 模拟加速度（重力 + 振动）
        linear_acceleration[0] = 0.1f * sin(sim_time * 2.0f);
        linear_acceleration[1] = 0.1f * cos(sim_time * 2.0f);
        linear_acceleration[2] = 9.81f + 0.05f * sin(sim_time * 5.0f);
    }
    
    mavlink_msg_ret_imu_attitude_data_packet_pack(
        SYSTEM_ID, COMPONENT_ID, &msg,
        packet_id,
        quaternion,
        angular_velocity,
        linear_acceleration
    );
    
    send_mavlink_message(&msg);
}

/**
 * @brief 生成并发送LiDAR辅助数据包
 */
void send_lidar_auxiliary_data() {
    mavlink_message_t msg;
    
    mavlink_ret_lidar_auxiliary_data_packet_t aux;
    memset(&aux, 0, sizeof(aux));
    
    aux.packet_id = packet_id;
    aux.payload_size = 120;  // 120个反射率点
    aux.lidar_work_status = 1;  // 正常工作
    aux.lidar_sync_delay_time = 0;
    aux.time_stamp_s_step = millis() / 1000;
    aux.time_stamp_us_step = (millis() % 1000) * 1000;
    aux.sys_rotation_period = 100000;  // 100ms一圈
    aux.com_rotation_period = 100000;
    
    // 水平角度参数
    aux.com_horizontal_angle_start = rotation_angle * 180.0f / PI;
    aux.com_horizontal_angle_step = 3.0f;  // 每个点3度
    
    // 垂直角度参数
    aux.sys_vertical_angle_start = -15.0f;  // 起始角度
    aux.sys_vertical_angle_span = 0.25f;    // 角度步进
    
    // 温度和传感器参数
    aux.apd_temperature = 25.0f;
    aux.imu_temperature = 28.0f;
    aux.dirty_index = 0.0f;
    aux.up_optical_q = 0.95f;
    aux.down_optical_q = 0.95f;
    aux.apd_voltage = 3.3f;
    
    // 校准参数
    aux.imu_angle_x_offset = 0.0f;
    aux.imu_angle_y_offset = 0.0f;
    aux.imu_angle_z_offset = 0.0f;
    aux.b_axis_dist = 30.0f;  // 30mm
    aux.theta_angle = 0.0f;
    aux.ksi_angle = 45.0f;
    
    // 生成反射率数据
    for (int i = 0; i < 120; i++) {
        if (current_mode == MODE_STATIC) {
            aux.reflect_data[i] = 128;  // 中等反射率
        } else {
            // 动态模式：模拟不同材质的反射
            aux.reflect_data[i] = 80 + (i % 40) * 3;
        }
    }
    
    mavlink_msg_ret_lidar_auxiliary_data_packet_encode(
        SYSTEM_ID, COMPONENT_ID, &msg, &aux
    );
    
    send_mavlink_message(&msg);
}

/**
 * @brief 生成并发送LiDAR距离数据包
 */
void send_lidar_distance_data() {
    mavlink_message_t msg;
    
    mavlink_ret_lidar_distance_data_packet_t dist;
    memset(&dist, 0, sizeof(dist));
    
    dist.packet_id = packet_id;
    dist.packet_cnt = 1;
    dist.payload_size = 240;  // 120个点 * 2字节
    
    // 生成距离数据（每个点2字节，小端序）
    for (int i = 0; i < 120; i++) {
        uint16_t distance_mm;
        
        if (current_mode == MODE_STATIC) {
            // 静态模式：固定距离
            distance_mm = 2000 + i * 10;  // 2-3.2米
        } else {
            // 动态模式：模拟真实场景
            float angle = (i / 120.0f) * 2.0f * PI;
            
            // 模拟一个矩形房间（4m x 3m）
            float x = cos(angle + rotation_angle);
            float y = sin(angle + rotation_angle);
            
            // 射线与房间边界的交点距离
            float dist_wall;
            if (abs(x) > abs(y)) {
                dist_wall = 2000.0f / abs(x);  // 左右墙 2m
            } else {
                dist_wall = 1500.0f / abs(y);  // 前后墙 1.5m
            }
            
            // 添加一些噪声
            distance_mm = (uint16_t)(dist_wall + random(-20, 20));
        }
        
        // 小端序存储
        dist.point_data[i * 2] = distance_mm & 0xFF;
        dist.point_data[i * 2 + 1] = (distance_mm >> 8) & 0xFF;
    }
    
    mavlink_msg_ret_lidar_distance_data_packet_encode(
        SYSTEM_ID, COMPONENT_ID, &msg, &dist
    );
    
    send_mavlink_message(&msg);
}

// ==================== Arduino 主函数 ====================

void setup() {
    // USB调试串口
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== Mock Unitree Lidar L1 ===");
    Serial.printf("Mode: %s\n", current_mode == MODE_STATIC ? "STATIC" : "DYNAMIC");
    
    // LiDAR模拟UART（连接到主ESP32）
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("[UART] Ready (baud=%d, TX=%d, RX=%d)\n", UART_BAUD, UART_TX_PIN, UART_RX_PIN);
    
    // 初始化随机种子
    randomSeed(analogRead(0));
    
    Serial.println("[Mock Lidar] Starting data generation...");
    Serial.println("  - IMU: 200Hz");
    Serial.println("  - PointCloud: 10Hz");
    
    last_imu_time = micros();
    last_cloud_time = micros();
}

void loop() {
    uint32_t now = micros();
    
    // 200Hz IMU数据
    if (now - last_imu_time >= IMU_INTERVAL_US) {
        send_imu_data();
        last_imu_time = now;
    }
    
    // 10Hz 点云数据
    if (now - last_cloud_time >= CLOUD_INTERVAL_US) {
        send_lidar_auxiliary_data();
        send_lidar_distance_data();
        last_cloud_time = now;
        
        // 更新包ID
        packet_id++;
        
        // 调试输出
        static uint32_t last_debug = 0;
        if (millis() - last_debug > 1000) {
            Serial.printf("[Mock] Packets sent: %u, Angle: %.1f°\n", 
                         packet_id, rotation_angle * 180.0f / PI);
            last_debug = millis();
        }
    }
    
    // 更新模拟时间
    sim_time += 0.001f;
    rotation_angle += 0.0005f;  // 缓慢旋转
    if (rotation_angle > 2.0f * PI) {
        rotation_angle -= 2.0f * PI;
    }
    
    // 避免过度占用CPU
    delayMicroseconds(100);
}

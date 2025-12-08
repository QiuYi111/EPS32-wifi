#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>

// 复用现有的优秀组件
#include "../lib/unitree_mavlink/UnitreeMavlink.h"
#include "../lib/unitree_mavlink/UnitreeMavlinkControl.h"
#include "../lib/motor/DualMotor.h"

// ==================== 配置参数 ====================
// UART配置（LiDAR通信）
#define UART_RX_PIN 18
#define UART_TX_PIN 17
#define UART_BAUD 2000000

// WiFi配置
#define WIFI_SSID "zhoujh22"
#define WIFI_PASSWORD "zjh040401"
#define UDP_PORT 8888

// 电机引脚（复用motor.cpp的定义）
#define MOTOR_IN1 21
#define MOTOR_IN2 20
#define MOTOR_IN3 1
#define MOTOR_IN4 2

// LED配置
#define LED_PIN 2
#define LED_COUNT 1

// ==================== 数据结构定义 ====================
// Linus式统一数据包格式 - 消除协议碎片化
struct __attribute__((packed)) DataPacket {
    uint32_t magic;          // 0xDEADBEEF - 帧同步
    uint32_t timestamp;      // Unix时间戳(ms)
    uint8_t type;           // 数据类型
    uint16_t length;        // 数据长度
    uint8_t data[];         // 数据负载
};

enum DataType {
    DATA_IMU = 1,
    DATA_POINTCLOUD = 2,
    DATA_MOTOR = 3,
    DATA_STATUS = 4,
    DATA_CONTROL = 5
};

// 电机状态数据包
struct __attribute__((packed)) MotorStatus {
    int16_t speedA;         // 电机A速度 (-1024~1024)
    int16_t speedB;         // 电机B速度 (-1024~1024)
    uint8_t stateA;         // 电机A状态 0=停止 1=正转 2=反转 3=刹车
    uint8_t stateB;         // 电机B状态
    uint16_t currentA;      // 电机A电流 (mA)
    uint16_t currentB;      // 电机B电流 (mA)
};

// 系统状态数据包
struct __attribute__((packed)) SystemStatus {
    float cpu_temp;         // CPU温度
    uint8_t wifi_rssi;      // WiFi信号强度
    uint16_t free_heap;     // 空闲内存
    uint8_t lidar_status;   // LiDAR状态
    uint32_t imu_count;     // IMU数据包计数
    uint32_t cloud_count;   // 点云数据包计数
};

// ==================== 全局变量 ====================
// 通信组件
AsyncUDP udp;
unitree::mav::UnitreeMavlinkParser mavlink_parser;
unitree::mav::LidarPipeline lidar_pipeline;

// 执行器组件
DualMotor motors(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4);
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// 状态变量
uint32_t last_heartbeat = 0;
uint32_t last_status_send = 0;
uint32_t imu_packet_count = 0;
uint32_t cloud_packet_count = 0;

MotorStatus motor_status = {0};
SystemStatus system_status = {0};

// ==================== 核心功能函数 ====================

/**
 * @brief 发送数据包到后端服务器
 * @param type 数据类型
 * @param data 数据指针
 * @param length 数据长度
 */
void send_data_packet(DataType type, const void* data, uint16_t length) {
    uint16_t packet_size = sizeof(DataPacket) + length;
    uint8_t* buffer = (uint8_t*)malloc(packet_size);

    if (!buffer) return;

    DataPacket* packet = (DataPacket*)buffer;
    packet->magic = 0xDEADBEEF;
    packet->timestamp = millis();
    packet->type = type;
    packet->length = length;

    memcpy(packet->data, data, length);

    udp.write(buffer, packet_size);
    free(buffer);
}

/**
 * @brief IMU数据回调函数 - 复用现有ImuSample结构
 */
void on_imu_data(const unitree::mav::ImuSample& imu) {
    imu_packet_count++;

    // 直接复用现有的优秀数据结构，无需转换
    send_data_packet(DATA_IMU, &imu, sizeof(imu));

    // 可选：串口调试输出
    static uint32_t last_debug = 0;
    if (millis() - last_debug > 1000) {
        Serial.printf("[IMU] Packet: %lu, Q: (%.2f,%.2f,%.2f,%.2f)\n",
                     imu.packet_id,
                     imu.quaternion[0], imu.quaternion[1],
                     imu.quaternion[2], imu.quaternion[3]);
        last_debug = millis();
    }
}

/**
 * @brief 点云数据回调函数 - 复用现有PointXYZI结构
 */
void on_cloud_data(uint16_t packet_id, const std::vector<unitree::mav::PointXYZI>& cloud) {
    cloud_packet_count++;

    // 发送点云数据 - 直接复用现有结构
    send_data_packet(DATA_POINTCLOUD, cloud.data(), cloud.size() * sizeof(unitree::mav::PointXYZI));

    static uint32_t last_debug = 0;
    if (millis() - last_debug > 1000) {
        Serial.printf("[PointCloud] Packet: %u, Points: %zu\n", packet_id, cloud.size());
        last_debug = millis();
    }
}

/**
 * @brief 处理控制命令
 */
void process_control_command(const uint8_t* data, uint16_t length) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, data);

    if (error) {
        Serial.printf("[Control] JSON parse error: %s\n", error.c_str());
        return;
    }

    // 电机控制命令
    if (doc["motor"].is<JsonObject>()) {
        JsonObject motor = doc["motor"];

        if (motor["speedA"].is<int>()) {
            int speedA = motor["speedA"];
            motors.setSpeedA(speedA);
            motor_status.speedA = speedA;
            motor_status.stateA = (speedA > 0) ? 1 : (speedA < 0) ? 2 : 0;
        }

        if (motor["speedB"].is<int>()) {
            int speedB = motor["speedB"];
            motors.setSpeedB(speedB);
            motor_status.speedB = speedB;
            motor_status.stateB = (speedB > 0) ? 1 : (speedB < 0) ? 2 : 0;
        }

        if (motor["stop"].is<bool>() && motor["stop"].as<bool>()) {
            motors.stopA();
            motors.stopB();
            motor_status.stateA = motor_status.stateB = 0;
            motor_status.speedA = motor_status.speedB = 0;
        }

        if (motor["brake"].is<bool>() && motor["brake"].as<bool>()) {
            motors.brakeA();
            motors.brakeB();
            motor_status.stateA = motor_status.stateB = 3;
            motor_status.speedA = motor_status.speedB = 0;
        }

        Serial.printf("[Control] Motor A: %d (%d), Motor B: %d (%d)\n",
                     motor_status.speedA, motor_status.stateA,
                     motor_status.speedB, motor_status.stateB);
    }
}

/**
 * @brief UDP数据包接收回调
 */
void on_udp_packet(AsyncUDPPacket packet) {
    if (packet.length() < sizeof(DataPacket)) {
        return;
    }

    DataPacket* dp = (DataPacket*)packet.data();

    // 验证数据包
    if (dp->magic != 0xDEADBEEF) {
        Serial.printf("[UDP] Invalid magic: 0x%08X\n", dp->magic);
        return;
    }

    if (dp->type == DATA_CONTROL) {
        process_control_command(dp->data, dp->length);
    }
}

/**
 * @brief 更新系统状态
 */
void update_system_status() {
    system_status.cpu_temp = temperatureRead();
    system_status.wifi_rssi = WiFi.RSSI();
    system_status.free_heap = ESP.getFreeHeap();
    system_status.imu_count = imu_packet_count;
    system_status.cloud_count = cloud_packet_count;
    system_status.lidar_status = (imu_packet_count > 0) ? 1 : 0; // 简化LiDAR状态检测
}

/**
 * @brief 设置LED状态指示
 */
void set_led_status(uint8_t r, uint8_t g, uint8_t b) {
    strip.setPixelColor(0, strip.Color(r, g, b));
    strip.show();
}

// ==================== Arduino主函数 ====================

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Robot Data Bridge ===");

    // 初始化LED状态指示
    strip.begin();
    strip.show();
    set_led_status(255, 0, 0); // 红色：初始化中

    // 初始化UART（LiDAR通信）
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("[UART] Ready (baud=%d, RX=%d, TX=%d)\n", UART_BAUD, UART_RX_PIN, UART_TX_PIN);

    // 初始化电机控制器
    motors.begin();
    motors.stopA();
    motors.stopB();
    Serial.println("[Motor] Dual motor system initialized");

    // 初始化MAVLink解析器 - 复用现有组件
    mavlink_parser.set_imu_callback(on_imu_data);
    mavlink_parser.set_lidar_aux_callback([](const unitree::mav::LidarAuxPacket& aux) {
        lidar_pipeline.handle_auxiliary(aux);
    });
    mavlink_parser.set_lidar_distance_callback([](const unitree::mav::LidarDistancePacket& dist) {
        lidar_pipeline.handle_distance(dist);
    });

    // 初始化点云处理器 - 复用现有组件
    lidar_pipeline.set_cloud_callback(on_cloud_data);
    Serial.println("[MAVLink] Parser and LiDAR pipeline initialized");

    // 连接WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WiFi] Connecting to ");
    Serial.print(WIFI_SSID);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        strip.setPixelColor(0, strip.Color(255, 128, 0)); // 橙色：连接中
        strip.show();
    }

    Serial.println();
    Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());

    // 初始化UDP
    if (udp.listen(UDP_PORT)) {
        udp.onPacket(on_udp_packet);
        Serial.printf("[UDP] Listening on port %d\n", UDP_PORT);
    }

    set_led_status(0, 255, 0); // 绿色：系统就绪
    Serial.println("[System] Ready! Starting main loop...");

    last_heartbeat = millis();
    last_status_send = millis();
}

void loop() {
    // 处理UART数据（LiDAR） - 复用现有解析器
    if (Serial1.available()) {
        mavlink_parser.feed(Serial1.read());
    }

    // 定期发送心跳包
    if (millis() - last_heartbeat > 5000) {
        const char* heartbeat = "ESP32_ROBOT_HEARTBEAT";
        udp.write((const uint8_t*)heartbeat, strlen(heartbeat));
        last_heartbeat = millis();
        Serial.println("[Heartbeat] Sent");
    }

    // 定期发送系统状态
    if (millis() - last_status_send > 1000) {
        update_system_status();
        send_data_packet(DATA_STATUS, &system_status, sizeof(system_status));

        // 发送电机状态
        send_data_packet(DATA_MOTOR, &motor_status, sizeof(motor_status));

        last_status_send = millis();

        // 串口调试输出
        Serial.printf("[Status] CPU:%.1f°C WiFi:%ddBM Heap:%dB IMU:%lu Cloud:%lu\n",
                     system_status.cpu_temp, system_status.wifi_rssi,
                     system_status.free_heap, system_status.imu_count, system_status.cloud_count);
    }

    // 处理WiFi断线重连
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Disconnected! Reconnecting...");
        set_led_status(255, 0, 0); // 红色：断线
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }

        Serial.println("[WiFi] Reconnected!");
        set_led_status(0, 255, 0); // 绿色：恢复连接
    }

    delay(1); // 避免过度占用CPU
}
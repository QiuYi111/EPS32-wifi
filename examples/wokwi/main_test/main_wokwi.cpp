/**
 * @file main_wokwi.cpp
 * @brief ESP32 Robot Data Bridge - Wokwi WebSocket Version
 * 
 * 基于 main.cpp，但适配 Wokwi 仿真环境：
 * - WiFi: 使用 Wokwi-GUEST 虚拟网络
 * - 通信: 使用 WebSocket 代替 UDP（浏览器兼容）
 * - Serial1: 通过 RFC2217 接收 Mock LiDAR 的 MAVLink 数据
 * 
 * 数据流:
 *   mock_lidar.py → RFC2217 → ESP32 Serial1 → MAVLink Parser
 *                                                    ↓
 *   WebUI ← WebSocket (ws_server.py) ← WebSocket ← ESP32
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// 复用现有的优秀组件
#include "../../../lib/unitree_mavlink/UnitreeMavlink.h"
#include "../../../lib/unitree_mavlink/UnitreeMavlinkControl.h"
#include "../../../lib/motor/DualMotor.h"

// ==================== 配置参数 ====================
// UART配置（LiDAR通信）- 与 mock_lidar 匹配
#define UART_RX_PIN 18
#define UART_TX_PIN 17
#define UART_BAUD 2000000

// WiFi配置 (Wokwi 虚拟网络)
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""

// WebSocket 服务器 (通过 wokwigw 网关连接到本机 ws_server.py)
#define WS_HOST "host.wokwi.internal"
#define WS_PORT 8888
#define WS_PATH "/"

// 电机引脚（仿真中不实际使用，但保持接口一致）
#define MOTOR_IN1 21
#define MOTOR_IN2 20
#define MOTOR_IN3 1
#define MOTOR_IN4 2

// ==================== 数据结构定义 ====================
// 与 main.cpp 保持一致
struct __attribute__((packed)) DataPacketHeader {
    uint32_t magic;          // 0xDEADBEEF - 帧同步
    uint32_t timestamp;      // Unix时间戳(ms)
    uint8_t type;           // 数据类型
    uint16_t length;        // 数据长度
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
    int16_t speedA;
    int16_t speedB;
    uint8_t stateA;
    uint8_t stateB;
    uint16_t currentA;
    uint16_t currentB;
};

// 系统状态数据包
struct __attribute__((packed)) SystemStatus {
    float cpu_temp;
    uint8_t wifi_rssi;
    uint16_t free_heap;
    uint8_t lidar_status;
    uint32_t imu_count;
    uint32_t cloud_count;
};

// ==================== 全局变量 ====================
// 通信组件
WebSocketsClient webSocket;
unitree::mav::UnitreeMavlinkParser mavlink_parser;
unitree::mav::LidarPipeline lidar_pipeline;

// 执行器组件
DualMotor motors(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4);

// 状态变量
bool wsConnected = false;
uint32_t last_heartbeat = 0;
uint32_t last_status_send = 0;
uint32_t imu_packet_count = 0;
uint32_t cloud_packet_count = 0;

MotorStatus motor_status = {0};
SystemStatus system_status = {0};

// ==================== 核心功能函数 ====================

/**
 * @brief 通过 WebSocket 发送数据包
 */
void send_data_packet(DataType type, const void* data, uint16_t length) {
    if (!wsConnected) return;
    
    uint16_t packet_size = sizeof(DataPacketHeader) + length;
    uint8_t* buffer = (uint8_t*)malloc(packet_size);
    
    if (!buffer) return;
    
    DataPacketHeader* header = (DataPacketHeader*)buffer;
    header->magic = 0xDEADBEEF;
    header->timestamp = millis();
    header->type = type;
    header->length = length;
    
    memcpy(buffer + sizeof(DataPacketHeader), data, length);
    
    webSocket.sendBIN(buffer, packet_size);
    free(buffer);
}

/**
 * @brief IMU数据回调函数
 */
void on_imu_data(const unitree::mav::ImuSample& imu) {
    imu_packet_count++;
    
    send_data_packet(DATA_IMU, &imu, sizeof(imu));
    
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
 * @brief 点云数据回调函数
 */
void on_cloud_data(uint16_t packet_id, const std::vector<unitree::mav::PointXYZI>& cloud) {
    cloud_packet_count++;
    
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
 * @brief WebSocket 事件处理
 */
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("[WS] Disconnected");
            wsConnected = false;
            break;
            
        case WStype_CONNECTED:
            Serial.printf("[WS] Connected to %s:%d\n", WS_HOST, WS_PORT);
            wsConnected = true;
            webSocket.sendTXT("[ESP32] WebSocket connected!");
            break;
            
        case WStype_TEXT:
            Serial.printf("[WS] Text: %s\n", (char*)payload);
            break;
            
        case WStype_BIN:
            // 二进制数据 - 解析 DataPacket
            if (length >= sizeof(DataPacketHeader)) {
                DataPacketHeader* header = (DataPacketHeader*)payload;
                if (header->magic == 0xDEADBEEF && header->type == DATA_CONTROL) {
                    process_control_command(payload + sizeof(DataPacketHeader), header->length);
                }
            }
            break;
            
        case WStype_ERROR:
            Serial.println("[WS] Error");
            break;
            
        default:
            break;
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
    system_status.lidar_status = (imu_packet_count > 0) ? 1 : 0;
}

// ==================== Arduino主函数 ====================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== ESP32 Robot Data Bridge (Wokwi) ===");
    Serial.println("Mode: WebSocket + Virtual Serial");
    
    // 初始化UART（LiDAR通信）- 接收 mock_lidar.py 的数据
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial.printf("[UART] Ready (baud=%d, RX=%d, TX=%d)\n", UART_BAUD, UART_RX_PIN, UART_TX_PIN);
    
    // 初始化电机控制器
    motors.begin();
    motors.stopA();
    motors.stopB();
    Serial.println("[Motor] Dual motor system initialized");
    
    // 初始化MAVLink解析器
    mavlink_parser.set_imu_callback(on_imu_data);
    mavlink_parser.set_lidar_aux_callback([](const unitree::mav::LidarAuxPacket& aux) {
        lidar_pipeline.handle_auxiliary(aux);
    });
    mavlink_parser.set_lidar_distance_callback([](const unitree::mav::LidarDistancePacket& dist) {
        lidar_pipeline.handle_distance(dist);
    });
    
    lidar_pipeline.set_cloud_callback(on_cloud_data);
    Serial.println("[MAVLink] Parser and LiDAR pipeline initialized");
    
    // 连接 Wokwi WiFi
    Serial.print("[WiFi] Connecting to ");
    Serial.print(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
        
        // 连接 WebSocket 服务器
        Serial.printf("[WS] Connecting to %s:%d%s\n", WS_HOST, WS_PORT, WS_PATH);
        webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
        webSocket.onEvent(webSocketEvent);
        webSocket.setReconnectInterval(5000);
    } else {
        Serial.println("\n[WiFi] Connection failed!");
    }
    
    Serial.println("[System] Ready! Starting main loop...");
    
    last_heartbeat = millis();
    last_status_send = millis();
}

void loop() {
    // WebSocket 事件处理
    webSocket.loop();
    
    // 处理UART数据（来自 mock_lidar.py 的 MAVLink 数据）
    while (Serial1.available()) {
        mavlink_parser.feed(Serial1.read());
    }
    
    // 定期发送心跳包
    if (millis() - last_heartbeat > 5000) {
        if (wsConnected) {
            webSocket.sendTXT("ESP32_ROBOT_HEARTBEAT");
        }
        last_heartbeat = millis();
        Serial.println("[Heartbeat] Sent");
    }
    
    // 定期发送系统状态
    if (millis() - last_status_send > 1000) {
        update_system_status();
        send_data_packet(DATA_STATUS, &system_status, sizeof(system_status));
        send_data_packet(DATA_MOTOR, &motor_status, sizeof(motor_status));
        last_status_send = millis();
        
        Serial.printf("[Status] WiFi:%ddBm Heap:%dB IMU:%lu Cloud:%lu\n",
                     system_status.wifi_rssi,
                     system_status.free_heap, system_status.imu_count, system_status.cloud_count);
    }
    
    // 处理WiFi断线重连
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Disconnected! Reconnecting...");
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 10) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\n[WiFi] Reconnected!");
        }
    }
    
    delay(1);
}

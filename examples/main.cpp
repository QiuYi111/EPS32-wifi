/**
 * @file main.cpp
 * @brief ESP32 WebUI Data Bridge
 *
 * 功能:
 *   1. 通过 UART 接收 Unitree LiDAR/IMU 数据 (MAVLink 协议)
 *   2. 通过 WebSocket 推送点云/IMU 数据到 WebUI (二进制)
 *   3. 接收 WebUI 的电机控制命令 (二进制)
 */

#include <Arduino.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

// 复用现有组件
#include "../lib/motor/DualMotor.h"
#include "../lib/unitree_mavlink/UnitreeMavlink.h"

// ==================== 配置参数 ====================

// WiFi AP 配置
#ifndef AP_SSID
#define AP_SSID "ESP32-Robot"
#endif
#ifndef AP_PASS
#define AP_PASS "12345678" // 至少8个字符
#endif

// UART 配置 (Unitree LiDAR)
#ifndef UNITREE_RX_PIN
#define UNITREE_RX_PIN 16
#endif
#ifndef UNITREE_TX_PIN
#define UNITREE_TX_PIN 15
#endif
#ifndef UNITREE_BAUD_RATE
#define UNITREE_BAUD_RATE 2000000 
#endif

// 电机引脚
#define MOTOR_IN1 6
#define MOTOR_IN2 7
#define MOTOR_IN3 4
#define MOTOR_IN4 5

// WebSocket 端口
#define WS_PORT 81

// ==================== 二进制协议定义 ====================

constexpr uint32_t PACKET_MAGIC = 0xDEADBEEF;

enum DataType : uint8_t {
  DATA_IMU = 1,
  DATA_POINTCLOUD = 2,
  DATA_MOTOR = 3,
  DATA_STATUS = 4,
  DATA_CONTROL = 5
};

struct __attribute__((packed)) DataPacket {
  uint32_t magic;
  uint32_t timestamp;
  uint8_t type;
  uint16_t length;
  // data[] follows
};

struct __attribute__((packed)) MotorControl {
  int16_t speedA; // -1024 ~ +1024
  int16_t speedB; // -1024 ~ +1024
  uint8_t flags;  // bit0: brake_A, bit1: brake_B
};

struct __attribute__((packed)) MotorStatus {
  int16_t speedA;
  int16_t speedB;
  uint8_t stateA; // 0=停止 1=正转 2=反转 3=刹车
  uint8_t stateB;
};

struct __attribute__((packed)) SystemStatus {
  float cpu_temp;
  int8_t wifi_rssi;
  uint16_t free_heap_kb;
  uint8_t lidar_status;
  uint32_t imu_count;
  uint32_t cloud_count;
};

// ==================== 全局变量 ====================

HardwareSerial &unitree_serial = Serial1;

// 通信组件
WebSocketsServer webSocket(WS_PORT);
unitree::mav::UnitreeMavlinkParser mavlink_parser;
unitree::mav::LidarPipeline lidar_pipeline;

// 电机
DualMotor motors(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4);
MotorStatus motor_status = {0};

// 状态
SystemStatus system_status = {0};
uint32_t last_status_send = 0;
bool ws_client_connected = false;

// ==================== 工具函数 ====================

/**
 * @brief 发送二进制数据包到所有 WebSocket 客户端
 */
void send_packet(DataType type, const void *data, uint16_t length) {
  if (!ws_client_connected)
    return;

  size_t packet_size = sizeof(DataPacket) + length;
  uint8_t *buffer = (uint8_t *)malloc(packet_size);
  if (!buffer)
    return;

  DataPacket *pkt = (DataPacket *)buffer;
  pkt->magic = PACKET_MAGIC;
  pkt->timestamp = millis();
  pkt->type = type;
  pkt->length = length;

  if (length > 0 && data) {
    memcpy(buffer + sizeof(DataPacket), data, length);
  }

  webSocket.broadcastBIN(buffer, packet_size);
  free(buffer);
}

/**
 * @brief 处理电机控制命令
 */
void handle_motor_control(const MotorControl *ctrl) {
  // 刹车优先
  if (ctrl->flags & 0x01) {
    motors.brakeA();
    motor_status.stateA = 3;
    motor_status.speedA = 0;
  } else {
    motors.setSpeedA(ctrl->speedA);
    motor_status.speedA = ctrl->speedA;
    motor_status.stateA = (ctrl->speedA > 0) ? 1 : (ctrl->speedA < 0) ? 2 : 0;
  }

  if (ctrl->flags & 0x02) {
    motors.brakeB();
    motor_status.stateB = 3;
    motor_status.speedB = 0;
  } else {
    motors.setSpeedB(ctrl->speedB);
    motor_status.speedB = ctrl->speedB;
    motor_status.stateB = (ctrl->speedB > 0) ? 1 : (ctrl->speedB < 0) ? 2 : 0;
  }

  Serial.printf("[Motor] A=%d(%d) B=%d(%d)\n", motor_status.speedA,
                motor_status.stateA, motor_status.speedB, motor_status.stateB);
}

// ==================== 回调函数 ====================

/**
 * @brief IMU 数据回调
 */
void on_imu_data(const unitree::mav::ImuSample &imu) {
  system_status.imu_count++;
  send_packet(DATA_IMU, &imu, sizeof(imu));
}

/**
 * @brief 点云数据回调
 */
void on_cloud_data(uint16_t packet_id,
                   const std::vector<unitree::mav::PointXYZI> &cloud) {
  system_status.cloud_count++;

  // 直接发送点云数据
  send_packet(DATA_POINTCLOUD, cloud.data(),
              cloud.size() * sizeof(unitree::mav::PointXYZI));

  static uint32_t last_log = 0;
  if (millis() - last_log > 1000) {
    Serial.printf("[Cloud] packet=%u points=%zu\n", packet_id, cloud.size());
    last_log = millis();
  }
}

/**
 * @brief WebSocket 事件回调
 */
void on_ws_event(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_CONNECTED:
    Serial.printf("[WS] Client #%u connected\n", num);
    ws_client_connected = true;
    break;

  case WStype_DISCONNECTED:
    Serial.printf("[WS] Client #%u disconnected\n", num);
    ws_client_connected = (webSocket.connectedClients() > 0);
    break;

  case WStype_BIN:
    // 解析二进制命令
    if (length >= sizeof(DataPacket)) {
      DataPacket *pkt = (DataPacket *)payload;

      if (pkt->magic != PACKET_MAGIC) {
        Serial.println("[WS] Invalid magic");
        return;
      }

      if (pkt->type == DATA_CONTROL && pkt->length >= sizeof(MotorControl)) {
        MotorControl *ctrl = (MotorControl *)(payload + sizeof(DataPacket));
        handle_motor_control(ctrl);
      }
    }
    break;

  default:
    break;
  }
}

/**
 * @brief 更新并发送系统状态
 */
void update_and_send_status() {
  system_status.cpu_temp = temperatureRead();
  system_status.wifi_rssi = WiFi.RSSI();
  system_status.free_heap_kb = ESP.getFreeHeap() / 1024;
  system_status.lidar_status = (system_status.imu_count > 0) ? 1 : 0;

  send_packet(DATA_STATUS, &system_status, sizeof(system_status));
  send_packet(DATA_MOTOR, &motor_status, sizeof(motor_status));
}

// ==================== Arduino 主函数 ====================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n=== ESP32 WebUI Data Bridge ===");

  // 初始化电机
  motors.begin();
  motors.stopA();
  motors.stopB();
  Serial.println("[Motor] Initialized");

  // 初始化 UART (Unitree)
  unitree_serial.begin(UNITREE_BAUD_RATE, SERIAL_8N1, UNITREE_RX_PIN,
                       UNITREE_TX_PIN);
  Serial.printf("[UART] Ready (baud=%d, RX=%d, TX=%d)\n", UNITREE_BAUD_RATE,
                UNITREE_RX_PIN, UNITREE_TX_PIN);

  // 设置 MAVLink 回调
  mavlink_parser.set_imu_callback(on_imu_data);
  mavlink_parser.set_lidar_aux_callback(
      [](const unitree::mav::LidarAuxPacket &aux) {
        lidar_pipeline.handle_auxiliary(aux);
      });
  mavlink_parser.set_lidar_distance_callback(
      [](const unitree::mav::LidarDistancePacket &dist) {
        lidar_pipeline.handle_distance(dist);
      });
  lidar_pipeline.set_cloud_callback(on_cloud_data);
  Serial.println("[MAVLink] Parser initialized");

  // 启动 WiFi AP 模式
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100); // 等待 AP 启动
  Serial.printf("[WiFi] AP Started - SSID: %s\n", AP_SSID);
  Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  // 启动 WebSocket 服务器
  webSocket.begin();
  webSocket.onEvent(on_ws_event);
  Serial.printf("[WS] Server started on port %d\n", WS_PORT);

  Serial.println("[System] Ready!\n");
}

void loop() {

  // 处理 WebSocket
  webSocket.loop();

  // 处理 Unitree 数据
  mavlink_parser.poll(unitree_serial);

  // 定期发送状态 (1Hz)
  if (millis() - last_status_send > 1000) {
    update_and_send_status();
    last_status_send = millis();

    Serial.printf("[Status] IMU=%lu Cloud=%lu Heap=%ukB RSSI=%d\n",
                  system_status.imu_count, system_status.cloud_count,
                  system_status.free_heap_kb, system_status.wifi_rssi);
  }

  // AP 模式下显示连接的客户端数
  static uint32_t last_client_check = 0;
  if (millis() - last_client_check > 5000) {
    Serial.printf("[WiFi] AP clients: %d\n", WiFi.softAPgetStationNum());
    last_client_check = millis();
  }

  delay(1);
}
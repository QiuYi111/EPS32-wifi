/**
 * @file main.cpp
 * @brief ESP32 LiDAR Bridge + Motor Control (混合架构)
 *
 * 功能:
 *   1. 透传 Unitree LiDAR MAVLink 数据到 WebSocket (原始二进制)
 *   2. 接收 WebSocket 的电机控制命令
 *   3. 通过 WebSocket 上报系统状态
 *
 * 架构说明:
 *   - LiDAR 数据不在 ESP32 解析，直接透传到 PC/WebUI
 *   - PC/WebUI 负责 MAVLink 解析和点云处理
 *   - 电机控制仍由 ESP32 处理
 */

#include <Arduino.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

// 电机驱动
#include "../lib/motor/DualMotor.h"

// ==================== 配置参数 ====================

// WiFi AP 配置
#ifndef AP_SSID
#define AP_SSID "ESP32-Robot"
#endif
#ifndef AP_PASS
#define AP_PASS "12345678" // 至少8个字符
#endif

// UART 配置 (Unitree LiDAR)
#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 15
#define LIDAR_BAUD_RATE 2000000

// 串口缓冲区大小 (关键参数!)
#define UART_RX_BUFFER_SIZE 65536 // 64KB

// 电机引脚
#define MOTOR_IN1 6
#define MOTOR_IN2 7
#define MOTOR_IN3 4
#define MOTOR_IN4 5

// WebSocket 端口
#define WS_PORT 81

// 透传缓冲区大小
#define FORWARD_BUFFER_SIZE 4096

// ==================== 二进制协议定义 ====================

// 数据包魔数
constexpr uint32_t PACKET_MAGIC = 0xDEADBEEF;

// 数据类型
enum PacketType : uint8_t {
  PKT_LIDAR_RAW = 1,     // LiDAR 原始数据 (透传)
  PKT_MOTOR_STATUS = 2,  // 电机状态
  PKT_SYSTEM_STATUS = 3, // 系统状态
  PKT_MOTOR_CONTROL = 4, // 电机控制命令 (来自客户端)
};

// 通用包头 (11 bytes)
struct __attribute__((packed)) PacketHeader {
  uint32_t magic;     // 0xDEADBEEF
  uint32_t timestamp; // millis()
  uint8_t type;       // PacketType
  uint16_t length;    // payload 长度
};

// 电机控制命令 (来自客户端)
struct __attribute__((packed)) MotorControlCmd {
  int16_t speed_a; // -1024 ~ +1024
  int16_t speed_b; // -1024 ~ +1024
  uint8_t flags;   // bit0: brake_a, bit1: brake_b
};

// 电机状态 (发送到客户端)
struct __attribute__((packed)) MotorStatusData {
  int16_t speed_a;
  int16_t speed_b;
  uint8_t state_a; // 0=停止 1=正转 2=反转 3=刹车
  uint8_t state_b;
};

// 系统状态 (发送到客户端)
struct __attribute__((packed)) SystemStatusData {
  float cpu_temp;
  int8_t wifi_rssi;
  uint16_t free_heap_kb;
  uint32_t lidar_bytes;    // 累计透传字节数
  uint32_t uptime_seconds; // 运行时间
  uint8_t ws_clients;      // WebSocket 客户端数
};

// ==================== 全局变量 ====================

// 串口
HardwareSerial &lidar_serial = Serial1;

// WebSocket 服务器
WebSocketsServer ws_server(WS_PORT);

// 电机
DualMotor motors(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4);

// 状态数据
MotorStatusData motor_status = {0};
SystemStatusData system_status = {0};

// 透传缓冲区 (静态分配，避免碎片)
static uint8_t forward_buffer[FORWARD_BUFFER_SIZE];

// WebSocket 发送缓冲区
static uint8_t ws_send_buffer[sizeof(PacketHeader) + 256];

// 统计
static uint32_t total_lidar_bytes = 0;
static uint32_t last_status_time = 0;
static uint32_t last_log_time = 0;

// 连接状态
static bool has_ws_client = false;

// ==================== 工具函数 ====================

/**
 * @brief 发送带包头的数据到所有 WebSocket 客户端
 */
void ws_send_packet(PacketType type, const void *data, uint16_t length) {
  if (!has_ws_client)
    return;
  if (sizeof(PacketHeader) + length > sizeof(ws_send_buffer))
    return;

  PacketHeader *hdr = (PacketHeader *)ws_send_buffer;
  hdr->magic = PACKET_MAGIC;
  hdr->timestamp = millis();
  hdr->type = type;
  hdr->length = length;

  if (length > 0 && data) {
    memcpy(ws_send_buffer + sizeof(PacketHeader), data, length);
  }

  ws_server.broadcastBIN(ws_send_buffer, sizeof(PacketHeader) + length);
}

/**
 * @brief 透传 LiDAR 原始数据 (带包头)
 */
void forward_lidar_data(const uint8_t *data, size_t length) {
  if (!has_ws_client || length == 0)
    return;

  // 对于 LiDAR 数据，添加简单包头后发送
  // 包头: magic(4) + timestamp(4) + type(1) + length(2) = 11 bytes
  size_t total = sizeof(PacketHeader) + length;

  // 使用动态分配大缓冲区
  uint8_t *buffer = (uint8_t *)malloc(total);
  if (!buffer)
    return;

  PacketHeader *hdr = (PacketHeader *)buffer;
  hdr->magic = PACKET_MAGIC;
  hdr->timestamp = millis();
  hdr->type = PKT_LIDAR_RAW;
  hdr->length = length;

  memcpy(buffer + sizeof(PacketHeader), data, length);
  ws_server.broadcastBIN(buffer, total);
  free(buffer);
}

/**
 * @brief 处理电机控制命令
 */
void handle_motor_control(const MotorControlCmd *cmd) {
  // 电机A
  if (cmd->flags & 0x01) {
    motors.brakeA();
    motor_status.state_a = 3;
    motor_status.speed_a = 0;
  } else {
    motors.setSpeedA(cmd->speed_a);
    motor_status.speed_a = cmd->speed_a;
    motor_status.state_a = (cmd->speed_a > 0) ? 1 : (cmd->speed_a < 0) ? 2 : 0;
  }

  // 电机B
  if (cmd->flags & 0x02) {
    motors.brakeB();
    motor_status.state_b = 3;
    motor_status.speed_b = 0;
  } else {
    motors.setSpeedB(cmd->speed_b);
    motor_status.speed_b = cmd->speed_b;
    motor_status.state_b = (cmd->speed_b > 0) ? 1 : (cmd->speed_b < 0) ? 2 : 0;
  }

  Serial.printf("[Motor] A=%d(%d) B=%d(%d)\n", motor_status.speed_a,
                motor_status.state_a, motor_status.speed_b,
                motor_status.state_b);
}

// ==================== WebSocket 回调 ====================

void on_ws_event(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_CONNECTED:
    Serial.printf("[WS] Client #%u connected from %s\n", num,
                  ws_server.remoteIP(num).toString().c_str());
    has_ws_client = true;
    break;

  case WStype_DISCONNECTED:
    Serial.printf("[WS] Client #%u disconnected\n", num);
    has_ws_client = (ws_server.connectedClients() > 0);
    // 断开时停止电机 (安全措施)
    motors.stopA();
    motors.stopB();
    motor_status = {0};
    break;

  case WStype_BIN:
    // 解析二进制命令
    if (length >= sizeof(PacketHeader)) {
      PacketHeader *hdr = (PacketHeader *)payload;

      if (hdr->magic != PACKET_MAGIC) {
        Serial.println("[WS] Invalid magic");
        return;
      }

      if (hdr->type == PKT_MOTOR_CONTROL &&
          hdr->length >= sizeof(MotorControlCmd)) {
        MotorControlCmd *cmd =
            (MotorControlCmd *)(payload + sizeof(PacketHeader));
        handle_motor_control(cmd);
      }
    }
    break;

  default:
    break;
  }
}

// ==================== 状态上报 ====================

void send_status() {
  // 更新系统状态
  system_status.cpu_temp = temperatureRead();
  system_status.wifi_rssi = WiFi.RSSI();
  system_status.free_heap_kb = ESP.getFreeHeap() / 1024;
  system_status.lidar_bytes = total_lidar_bytes;
  system_status.uptime_seconds = millis() / 1000;
  system_status.ws_clients = ws_server.connectedClients();

  // 发送状态
  ws_send_packet(PKT_SYSTEM_STATUS, &system_status, sizeof(system_status));
  ws_send_packet(PKT_MOTOR_STATUS, &motor_status, sizeof(motor_status));
}

// ==================== Arduino 主函数 ====================

void setup() {
  // 调试串口
  Serial.begin(115200);
  delay(500);

  Serial.println("\n========================================");
  Serial.println("  ESP32 LiDAR Bridge + Motor Control");
  Serial.println("  (Hybrid Architecture)");
  Serial.println("========================================\n");

  // 初始化电机
  motors.begin();
  motors.stopA();
  motors.stopB();
  Serial.println("[Motor] Initialized");

  // 初始化 LiDAR 串口 (关键：先设置缓冲区！)
  lidar_serial.setRxBufferSize(UART_RX_BUFFER_SIZE);
  lidar_serial.begin(LIDAR_BAUD_RATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  Serial.printf("[UART] LiDAR ready (baud=%d, RX=%d, TX=%d, buf=%dKB)\n",
                LIDAR_BAUD_RATE, LIDAR_RX_PIN, LIDAR_TX_PIN,
                UART_RX_BUFFER_SIZE / 1024);

  // 启动 WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  Serial.printf("[WiFi] AP Started - SSID: %s\n", AP_SSID);
  Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  // 启动 WebSocket 服务器
  ws_server.begin();
  ws_server.onEvent(on_ws_event);
  Serial.printf("[WS] Server started on port %d\n", WS_PORT);

  Serial.println("\n[System] Ready! Waiting for clients...\n");
}

void loop() {
  // ========== 最高优先级: WebSocket 事件 (电机命令) ==========
  // 电机命令是低频关键指令，必须优先处理！
  ws_server.loop();

  // ========== 高优先级: 透传 LiDAR 数据 ==========
  int uart_available = lidar_serial.available();
  if (uart_available > 0) {
    // 限制单次读取量
    int read_len = min(uart_available, (int)sizeof(forward_buffer));
    lidar_serial.readBytes(forward_buffer, read_len);

    // 统计
    total_lidar_bytes += read_len;

    // 透传到 WebSocket
    if (has_ws_client) {
      forward_lidar_data(forward_buffer, read_len);
    }
  }

  // ========== 低优先级: 状态上报 (1Hz) ==========
  uint32_t now = millis();
  if (now - last_status_time >= 1000) {
    send_status();
    last_status_time = now;
  }

  // ========== 调试日志 (5s) ==========
  if (now - last_log_time >= 5000) {
    Serial.printf("[Stats] Heap=%uKB, LiDAR=%luKB, Clients=%d\n",
                  ESP.getFreeHeap() / 1024, total_lidar_bytes / 1024,
                  ws_server.connectedClients());
    last_log_time = now;
  }

  // 不要 delay，让 loop 尽快返回以处理串口数据
}
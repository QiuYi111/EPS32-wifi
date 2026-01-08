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

#include <Adafruit_NeoPixel.h>
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

// UART 配置 (OpenMV)
#define OPENMV_RX_PIN 18
#define OPENMV_TX_PIN 17
#define OPENMV_BAUD_RATE 921600

// RGB LED (NeoPixel)
#define RGB_LED_PIN 48
#define RGB_LED_COUNT 1

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
  PKT_CAM_JPEG = 5,      // OpenMV JPEG 图片
  PKT_MIXED_DATA = 6,    // 混合数据: [CamLen 4B] + [CamData] + [LidarData...]
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
HardwareSerial &cam_serial = Serial2;

// LED
Adafruit_NeoPixel status_led(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
uint32_t led_off_time = 0;

// WebSocket 服务器
WebSocketsServer ws_server(WS_PORT);

// 电机
DualMotor motors(MOTOR_IN1, MOTOR_IN2, MOTOR_IN3, MOTOR_IN4);

// 状态数据
MotorStatusData motor_status = {0};
SystemStatusData system_status = {0};

// 透传缓冲区 (静态分配，避免碎片)
// static uint8_t forward_buffer[FORWARD_BUFFER_SIZE]; // 废弃，改用 staging

// LiDAR 暂存缓冲区 (32KB, 足够容纳 ~150ms @ 2Mbps 的数据)
#define LIDAR_STAGING_SIZE 32768
static uint8_t lidar_staging_buffer[LIDAR_STAGING_SIZE];
static size_t lidar_staging_len = 0;

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
 * @brief 暂存 LiDAR 数据
 */
void buffer_lidar_data(const uint8_t *data, size_t length) {
  if (length == 0)
    return;

  // 如果缓冲区不够，先强制发送已有的(或者丢弃旧的?
  // 为了简单，丢弃旧的并报错，因为理论上不会满)
  if (lidar_staging_len + length > LIDAR_STAGING_SIZE) {
    Serial.println("[LiDAR] Buffer overflow! Dropping old data.");
    lidar_staging_len = 0; // 简单复位，防止死锁
  }

  memcpy(lidar_staging_buffer + lidar_staging_len, data, length);
  lidar_staging_len += length;
}

/**
 * @brief 发送混合数据包 (Cam + Lidar)
 *
 * 格式:
 * Header (11)
 * CamSize (4, LE)
 * CamData (N)
 * LidarData (M)
 */
void flush_mixed_data(const uint8_t *cam_data, size_t cam_len) {
  if (!has_ws_client) {
    lidar_staging_len = 0; // 没有客户端就清空
    return;
  }

  size_t total_payload = 4 + cam_len + lidar_staging_len;
  size_t total_packet = sizeof(PacketHeader) + total_payload;

  uint8_t *buffer = (uint8_t *)malloc(total_packet);
  if (!buffer) {
    Serial.println("[Mix] Malloc failed");
    return;
  }

  PacketHeader *hdr = (PacketHeader *)buffer;
  hdr->magic = PACKET_MAGIC;
  hdr->timestamp = millis();
  hdr->type = PKT_MIXED_DATA;
  hdr->length = total_payload; // Payload 长度

  uint8_t *ptr = buffer + sizeof(PacketHeader);

  // 1. Cam Size (4 bytes)
  uint32_t c_len = (uint32_t)cam_len;
  memcpy(ptr, &c_len, 4);
  ptr += 4;

  // 2. Cam Data
  if (cam_len > 0 && cam_data) {
    memcpy(ptr, cam_data, cam_len);
    ptr += cam_len;
  }

  // 3. Lidar Data
  if (lidar_staging_len > 0) {
    memcpy(ptr, lidar_staging_buffer, lidar_staging_len);
  }

  // 发送
  ws_server.broadcastBIN(buffer, total_packet);
  free(buffer);

  // 清空 LiDAR 缓冲区
  lidar_staging_len = 0;
}

/**
 * @brief 透传 Camera JPEG 数据 (带包头)
 */
void forward_cam_data(const uint8_t *data, size_t length) {
  if (!has_ws_client || length == 0)
    return;

  size_t total = sizeof(PacketHeader) + length;
  uint8_t *buffer = (uint8_t *)malloc(total);
  if (!buffer)
    return;

  PacketHeader *hdr = (PacketHeader *)buffer;
  hdr->magic = PACKET_MAGIC;
  hdr->timestamp = millis();
  hdr->type = PKT_CAM_JPEG;
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
  // Serial.begin(115200);
  // delay(500);

  // Serial.println("\n========================================");
  // Serial.println("  ESP32 LiDAR Bridge + Motor Control");
  // Serial.println("  (Hybrid Architecture)");
  // Serial.println("========================================\n");

  // // 初始化电机

  // // Serial.println("[Motor] Initialized");
  // // Serial.println("[TEST] Force Motor A RUN 1000 for 1s...");
  // motors.setSpeedA(1000); // 接近全速
  // motors.setSpeedB(1000);
  // delay(1000); // 堵塞式延时，观察电机动不动
  // motors.stopA();
  // motors.stopB();
  // Serial.println("[TEST] Motor STOP");

  // 初始化 LiDAR 串口 (关键：先设置缓冲区！)
  lidar_serial.setRxBufferSize(UART_RX_BUFFER_SIZE);
  lidar_serial.begin(LIDAR_BAUD_RATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  status_led.begin(); // Make sure to begin LED before using it
  status_led.setBrightness(50);
  status_led.setPixelColor(0, status_led.Color(0, 255, 0));
  status_led.show();

  // 初始化 OpenMV 串口
  cam_serial.setRxBufferSize(UART_RX_BUFFER_SIZE); // 同样给大缓存
  cam_serial.begin(OPENMV_BAUD_RATE, SERIAL_8N1, OPENMV_RX_PIN, OPENMV_TX_PIN);

  // Serial.printf("[UART] LiDAR ready (baud=%d, RX=%d, TX=%d, buf=%dKB)\n",
  //               LIDAR_BAUD_RATE, LIDAR_RX_PIN, LIDAR_TX_PIN,
  //               UART_RX_BUFFER_SIZE / 1024);

  delay(30000);
  status_led.setPixelColor(0, status_led.Color(0, 255, 0));
  status_led.show();
  // 启动 WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  motors.begin();
  motors.stopA();
  motors.stopB();
  // Serial.printf("[WiFi] AP Started - SSID: %s\n", AP_SSID);
  // Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  // 启动 WebSocket 服务器
  ws_server.begin();
  ws_server.onEvent(on_ws_event);
  // Serial.printf("[WS] Server started on port %d\n", WS_PORT);

  // Serial.println("\n[System] Ready! Waiting for clients...\n");
  delay(100);
}

void loop() {
  // ========== 1. WebSocket 事件 (电机命令) ==========
  ws_server.loop();

  // ========== 2. 优化：交替处理或优先相机 ==========
  // 之前的逻辑是优先处理 LiDAR，直到 LiDAR 没数据。
  // 由于 LiDAR 数据量大且持续 (2Mbps)，这可能导致相机数据处理被饥饿。

  // 检查相机 (优先检查，确保不丢头)
  static enum { STATE_WaitSize, STATE_ReadingBody } cam_state = STATE_WaitSize;
  static uint32_t cam_body_len = 0;
  static uint32_t cam_read_count = 0;
  static uint8_t *cam_img_buffer = NULL;

  // 每次 Loop 最多处理一定量的相机数据，避免阻塞太久
  // 但是对于图片，我们希望尽快读完
  if (cam_serial.available() >= 4 && cam_state == STATE_WaitSize) {
    uint8_t size_buf[4];
    cam_serial.readBytes(size_buf, 4);
    cam_body_len = size_buf[0] | (size_buf[1] << 8) | (size_buf[2] << 16) |
                   (size_buf[3] << 24);

    if (cam_body_len > 0 && cam_body_len < 100000) {
      if (cam_img_buffer)
        free(cam_img_buffer);
      cam_img_buffer = (uint8_t *)malloc(cam_body_len);
      if (cam_img_buffer) {
        cam_state = STATE_ReadingBody;
        cam_read_count = 0;
        // 【关键修改】用户要求只发送“视频帧时刻”的点云。
        // 因此，当检测到新的一帧图片开始时，清空之前的 LiDAR 缓存。
        // 这样发送出去的混合包里，LiDAR 数据仅仅包含“接收图片期间”产生的数据。
        lidar_staging_len = 0;
      } else {
        // 内存不足
      }
    }
  }

  if (cam_state == STATE_ReadingBody && cam_img_buffer) {
    // 尽可能多读，但不要死循环卡死整个系统
    size_t avail = cam_serial.available();
    if (avail > 0) {
      size_t to_read = min((uint32_t)avail, cam_body_len - cam_read_count);
      // 限制每次 loop 读的上限？不，串口读取很快，读完缓冲区即可
      cam_serial.readBytes(cam_img_buffer + cam_read_count, to_read);
      cam_read_count += to_read;
    }

    if (cam_read_count >= cam_body_len) {
      // 接收完毕 -> 发送混合包
      flush_mixed_data(cam_img_buffer, cam_body_len);

      status_led.setPixelColor(0, status_led.Color(0, 255, 0));
      status_led.show();
      led_off_time = millis() + 100;
      free(cam_img_buffer);
      cam_img_buffer = NULL;
      cam_state = STATE_WaitSize;
    }
  }

  // ========== 3. 缓存 LiDAR 数据 ==========
  int uart_available = lidar_serial.available();
  if (uart_available > 0) {
    // 临时缓冲区读出来
    uint8_t temp_buf[2048];
    int read_len = min(uart_available, (int)sizeof(temp_buf));
    lidar_serial.readBytes(temp_buf, read_len);
    total_lidar_bytes += read_len;

    // 存入 Staging Buffer
    buffer_lidar_data(temp_buf, read_len);
  }

  // 保护：如果 LiDAR 缓冲区太满 (例如相机很久没发图)，强制单独发送 LiDAR
  // 这里为了复用 flush_mixed_data，我们可以发一个空的相机包
  if (lidar_staging_len > 30000) {
    flush_mixed_data(NULL, 0);
  }

  // LED 自动关闭逻辑
  if (led_off_time > 0 && millis() > led_off_time) {
    status_led.setPixelColor(0, status_led.Color(0, 0, 0));
    status_led.show();
    led_off_time = 0;
  }
  ws_server.loop();

  // ========== 4. 状态上报 (1Hz) ==========
  uint32_t now = millis();
  if (now - last_status_time >= 1500) {
    send_status();
    last_status_time = now;
  }
}

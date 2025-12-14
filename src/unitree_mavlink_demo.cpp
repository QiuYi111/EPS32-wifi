#include <Arduino.h>
#include <WiFi.h>

// ================= 配置 =================
const char* ssid     = "ESP32-Lidar-Bridge";
const char* password = "password123";
const uint16_t tcp_port = 8888;

// 雷达串口配置 (必须 2M)
#define LIDAR_BAUD 2000000
#define RX_PIN 16
#define TX_PIN 15

// ================= 全局对象 =================
WiFiServer server(tcp_port);
WiFiClient client;
HardwareSerial& LidarSerial = Serial1;

// 巨大的串口接收缓冲区 (64KB)
const size_t RX_BUF_SIZE = 65536; 
// WiFi 发送缓冲区 (4KB 分块发送)
uint8_t wifi_buf[4096]; 

void setup() {
  // 1. 调试串口 (Serial0)
  Serial.begin(115200);
  
  // 2. 雷达串口 (Serial1) - 关键：开启大缓冲
  LidarSerial.setRxBufferSize(RX_BUF_SIZE);
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  // 3. 启动 WiFi AP 模式 (最稳，甚至不需要路由器)
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  Serial.println("\n=== ESP32 Lidar Transparent Bridge ===");
  Serial.printf("AP Started: %s\n", ssid);
  Serial.printf("IP Address: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("TCP Server listening on port %d\n", tcp_port);
  
  server.begin();
}

void loop() {
  // 等待客户端连接
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("New TCP Client connected!");
      // 清空串口积压的数据，防止旧数据干扰
      while(LidarSerial.available()) LidarSerial.read();
    }
    delay(10);
    return;
  }

  // === 核心透传逻辑 ===
  
  // 1. 检查串口有多少数据
  int len = LidarSerial.available();
  if (len > 0) {
    // 限制单次搬运最大量，防止撑爆 wifi_buf
    if (len > sizeof(wifi_buf)) len = sizeof(wifi_buf);
    
    // 2. 从串口读出来
    LidarSerial.readBytes(wifi_buf, len);
    Serial.printf("%02X ", c);
    
    // 3. 原封不动发给 WiFi
    // write 是阻塞的，如果网络卡，这里会卡一下，但串口有64K缓冲撑着
    client.write(wifi_buf, len);
  }
  
  // 4. (可选) 反向透传：如果电脑发指令给雷达（比如唤醒）
  while (client.available()) {
    LidarSerial.write(client.read());
  }
}
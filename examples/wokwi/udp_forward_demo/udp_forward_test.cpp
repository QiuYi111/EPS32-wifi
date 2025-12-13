/**
 * @file udp_forward_test.cpp
 * @brief Wokwi Virtual Serial + UDP Forwarding Demo
 * 
 * 学习目标:
 *   1. 理解 Wokwi 虚拟串口: 通过 wokwi-cli --interactive 接收 Python 发送的数据
 *   2. 理解 Wokwi UDP 转发: 通过 wokwigw 网关将数据转发到本机 Python 客户端
 * 
 * 数据流:
 *   Python Sender → stdin → wokwi-cli → ESP32 (Serial.read)
 *                                          ↓
 *   Python Receiver ← UDP ← wokwigw ← ESP32 (UDP.send to host.wokwi.internal)
 * 
 * Usage:
 *   1. 复制此文件到 src/main.cpp
 *   2. 编译: pio run
 *   3. 运行 demo: uv run examples/wokwi/udp_forward_demo/gateway_demo.py
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ==================== 配置参数 ====================

// WiFi 配置 (Wokwi 虚拟网络)
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// UDP 转发目标
// host.wokwi.internal 是 Wokwi 提供的特殊域名，指向运行网关的本机
const char* UDP_HOST = "host.wokwi.internal";
const uint16_t UDP_PORT = 8888;

// 是否启用 UDP 转发 (设为 false 仅测试串口)
bool enableUDP = true;

// 串口缓冲区
#define BUFFER_SIZE 256
char rxBuffer[BUFFER_SIZE];
int rxIndex = 0;

// 消息计数器
unsigned long messageCount = 0;

// WiFi UDP 对象
WiFiUDP udp;

// ==================== WiFi 连接 ====================

void connectWiFi() {
    Serial.println("[WIFI] 开始连接 WiFi...");
    Serial.print("[WIFI] SSID: ");
    Serial.println(WIFI_SSID);
    
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println();
        Serial.println("[WIFI] ✓ WiFi 连接成功!");
        Serial.print("[WIFI] IP 地址: ");
        Serial.println(WiFi.localIP());
        Serial.print("[WIFI] 网关地址: ");
        Serial.println(WiFi.gatewayIP());
    } else {
        Serial.println();
        Serial.println("[WIFI] ✗ WiFi 连接失败!");
        enableUDP = false;  // 禁用 UDP
    }
}

// ==================== UDP 发送 ====================

bool sendUdpMessage(const char* message) {
    if (!enableUDP) {
        Serial.println("[UDP] 已禁用");
        return false;
    }
    
    Serial.print("[UDP] 发送到 ");
    Serial.print(UDP_HOST);
    Serial.print(":");
    Serial.println(UDP_PORT);
    
    // 开始 UDP 包
    int result = udp.beginPacket(UDP_HOST, UDP_PORT);
    if (result == 1) {
        // 构建消息
        char udpMessage[512];
        snprintf(udpMessage, sizeof(udpMessage), 
                 "[ESP32->UDP #%lu] %s", 
                 messageCount, message);
        
        // 发送数据
        size_t written = udp.print(udpMessage);
        
        // 结束并发送
        result = udp.endPacket();
        if (result == 1) {
            Serial.print("[UDP] ✓ 发送成功 (");
            Serial.print(written);
            Serial.println(" bytes)");
            return true;
        } else {
            Serial.println("[UDP] ✗ endPacket 失败");
        }
    } else {
        Serial.println("[UDP] ✗ beginPacket 失败 - 跳过 UDP");
        enableUDP = false;  // 禁用后续 UDP 尝试
    }
    return false;
}

// ==================== 主程序 ====================

void setup() {
    // 初始化 USB Serial (用于 Wokwi console 输出)
    Serial.begin(115200);
    
    // 等待串口初始化
    delay(1000);
    
    // 欢迎信息
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════════╗");
    Serial.println("║   Wokwi Virtual Serial + UDP Forwarding Demo         ║");
    Serial.println("║   虚拟串口 + UDP 转发学习                            ║");
    Serial.println("╠══════════════════════════════════════════════════════╣");
    Serial.println("║   功能:                                              ║");
    Serial.println("║   1. 接收来自 Python 的串口消息                      ║");
    Serial.println("║   2. 通过 UDP 转发到 Python 客户端                   ║");
    Serial.println("╚══════════════════════════════════════════════════════╝");
    Serial.println();
    
    // 连接 WiFi
    connectWiFi();
    
    Serial.println();
    Serial.println("[INFO] 系统就绪! 等待接收串口消息...");
    if (enableUDP) {
        Serial.print("[INFO] UDP 转发目标: ");
        Serial.print(UDP_HOST);
        Serial.print(":");
        Serial.println(UDP_PORT);
    } else {
        Serial.println("[INFO] UDP 已禁用，仅显示串口消息");
    }
    Serial.println();
}

void loop() {
    // 检查是否有串口数据可读
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        // 检测行结束符
        if (c == '\n') {
            // 处理完整的一行
            if (rxIndex > 0) {
                // 空终止字符串
                rxBuffer[rxIndex] = '\0';
                
                // 移除可能的 \r
                if (rxIndex > 0 && rxBuffer[rxIndex - 1] == '\r') {
                    rxBuffer[rxIndex - 1] = '\0';
                }
                
                // 增加消息计数
                messageCount++;
                
                // 在 console 中显示接收到的消息
                Serial.println("┌─────────────────────────────────────────────────────┐");
                Serial.print  ("│ [#");
                Serial.print(messageCount);
                Serial.print("] 串口收到: ");
                Serial.println(rxBuffer);
                Serial.println("│");
                
                // 尝试通过 UDP 转发
                sendUdpMessage(rxBuffer);
                
                Serial.println("└─────────────────────────────────────────────────────┘");
                Serial.println();
            }
            
            // 重置缓冲区
            rxIndex = 0;
        }
        // 处理回车符
        else if (c == '\r') {
            // 忽略，等待 \n
        }
        // 存储正常字符
        else if (rxIndex < BUFFER_SIZE - 1) {
            rxBuffer[rxIndex++] = c;
        }
    }
    
    // 防止忙等待
    delay(10);
}

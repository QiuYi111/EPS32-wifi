/**
 * @file websocket_test.cpp
 * @brief Wokwi Virtual Serial + WebSocket Forwarding Demo
 * 
 * 学习目标:
 *   1. 理解 Wokwi 虚拟串口: 通过 RFC2217 接收 Python 发送的数据
 *   2. 理解 WebSocket 转发: 通过 wokwigw 网关将数据转发到本机 WebSocket 服务器
 * 
 * 数据流:
 *   Python Sender → RFC2217 → ESP32 (Serial.read)
 *                                ↓
 *   Python WebSocket Server ← WebSocket ← ESP32 (WebSocket.send to host.wokwi.internal)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>

// ==================== 配置参数 ====================

// WiFi 配置 (Wokwi 虚拟网络)
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASSWORD = "";

// WebSocket 服务器 (通过 wokwigw 网关连接到本机)
const char* WS_HOST = "host.wokwi.internal";
const uint16_t WS_PORT = 8888;
const char* WS_PATH = "/";

// 串口缓冲区
#define BUFFER_SIZE 256
char rxBuffer[BUFFER_SIZE];
int rxIndex = 0;

// 消息计数器
unsigned long messageCount = 0;

// WebSocket 客户端
WebSocketsClient webSocket;

// WebSocket 连接状态
bool wsConnected = false;

// ==================== WebSocket 事件处理 ====================

void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.println("[WS] 断开连接");
            wsConnected = false;
            break;
            
        case WStype_CONNECTED:
            Serial.print("[WS] ✓ 已连接到 ");
            Serial.print(WS_HOST);
            Serial.print(":");
            Serial.println(WS_PORT);
            wsConnected = true;
            
            // 发送连接消息
            webSocket.sendTXT("[ESP32] WebSocket 连接成功!");
            break;
            
        case WStype_TEXT:
            Serial.print("[WS] 收到消息: ");
            Serial.println((char*)payload);
            break;
            
        case WStype_ERROR:
            Serial.println("[WS] 错误");
            break;
            
        default:
            break;
    }
}

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
    } else {
        Serial.println();
        Serial.println("[WIFI] ✗ WiFi 连接失败!");
    }
}

// ==================== 发送 WebSocket 消息 ====================

void sendWebSocketMessage(const char* message) {
    if (!wsConnected) {
        Serial.println("[WS] 未连接，跳过发送");
        return;
    }
    
    Serial.print("[WS] 发送消息...");
    
    // 构建消息
    char wsMessage[512];
    snprintf(wsMessage, sizeof(wsMessage), 
             "[ESP32->WS #%lu] %s", 
             messageCount, message);
    
    // 发送
    webSocket.sendTXT(wsMessage);
    Serial.println(" ✓");
}

// ==================== 主程序 ====================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // 欢迎信息
    Serial.println();
    Serial.println("╔══════════════════════════════════════════════════════╗");
    Serial.println("║   Wokwi Virtual Serial + WebSocket Demo              ║");
    Serial.println("║   虚拟串口 + WebSocket 转发学习                      ║");
    Serial.println("╠══════════════════════════════════════════════════════╣");
    Serial.println("║   功能:                                              ║");
    Serial.println("║   1. 接收来自 Python 的串口消息                      ║");
    Serial.println("║   2. 通过 WebSocket 转发到 Python 服务器             ║");
    Serial.println("╚══════════════════════════════════════════════════════╝");
    Serial.println();
    
    // 连接 WiFi
    connectWiFi();
    
    if (WiFi.status() == WL_CONNECTED) {
        // 连接 WebSocket 服务器
        Serial.println();
        Serial.print("[WS] 连接 WebSocket 服务器: ");
        Serial.print(WS_HOST);
        Serial.print(":");
        Serial.println(WS_PORT);
        
        webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
        webSocket.onEvent(webSocketEvent);
        webSocket.setReconnectInterval(5000);
        
        Serial.println("[INFO] 系统就绪! 等待接收串口消息...");
        Serial.println();
    }
}

void loop() {
    // WebSocket 事件处理
    webSocket.loop();
    
    // 检查 WiFi 连接
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WARN] WiFi 断开，正在重连...");
        connectWiFi();
        delay(1000);
        return;
    }
    
    // 检查串口数据
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n') {
            if (rxIndex > 0) {
                rxBuffer[rxIndex] = '\0';
                
                // 移除 \r
                if (rxIndex > 0 && rxBuffer[rxIndex - 1] == '\r') {
                    rxBuffer[rxIndex - 1] = '\0';
                }
                
                messageCount++;
                
                // 显示收到的消息
                Serial.println("┌─────────────────────────────────────────────────────┐");
                Serial.print  ("│ [#");
                Serial.print(messageCount);
                Serial.print("] 串口收到: ");
                Serial.println(rxBuffer);
                Serial.println("│");
                
                // 通过 WebSocket 转发
                sendWebSocketMessage(rxBuffer);
                
                Serial.println("└─────────────────────────────────────────────────────┘");
                Serial.println();
            }
            rxIndex = 0;
        }
        else if (c == '\r') {
            // 忽略
        }
        else if (rxIndex < BUFFER_SIZE - 1) {
            rxBuffer[rxIndex++] = c;
        }
    }
    
    delay(10);
}

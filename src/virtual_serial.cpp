/**
 * @file virtual_serial.cpp
 * @brief Virtual Serial Port Demo for Wokwi Simulation
 * 
 * This demo receives data from a Python script via Wokwi's virtual serial port
 * and displays it in the Wokwi console.
 * 
 * 虚拟串口学习 Demo:
 *   - 通过 RFC2217 协议与外部 Python 脚本通信
 *   - 接收 Python 发送的 "Hello World!" 并显示在 console
 * 
 * Usage:
 *   1. 编译: pio run
 *   2. 启动 Wokwi: wokwi-cli examples/wokwi/virtual_serial
 *   3. 运行 Python 脚本: uv run examples/wokwi/virtual_serial/serial_sender.py
 *   4. 在 Wokwi console 中观察输出
 */

#include <Arduino.h>

// 接收缓冲区
#define BUFFER_SIZE 256
char rxBuffer[BUFFER_SIZE];
int rxIndex = 0;

// 消息计数器
unsigned long messageCount = 0;

void setup() {
    // 初始化 USB Serial (用于 Wokwi console 输出)
    Serial.begin(115200);
    
    // 等待串口初始化
    delay(1000);
    
    // 欢迎信息
    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║   Wokwi Virtual Serial Demo              ║");
    Serial.println("║   虚拟串口学习测试                       ║");
    Serial.println("╠══════════════════════════════════════════╣");
    Serial.println("║   等待接收来自 Python 的消息...          ║");
    Serial.println("║   Waiting for messages from Python...    ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();
    Serial.println("[INFO] Serial port initialized at 115200 baud");
    Serial.println("[INFO] Ready to receive data via RFC2217 virtual serial");
    Serial.println();
}

void loop() {
    // 检查是否有数据可读
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
                Serial.println("┌─────────────────────────────────────────┐");
                Serial.print  ("│ [#");
                Serial.print(messageCount);
                Serial.print("] 收到消息: ");
                Serial.println(rxBuffer);
                Serial.print  ("│ Received: ");
                Serial.println(rxBuffer);
                Serial.println("└─────────────────────────────────────────┘");
                
                // 发送确认回复给 Python
                Serial.print("[ACK] Message #");
                Serial.print(messageCount);
                Serial.println(" received successfully");
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

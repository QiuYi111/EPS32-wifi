#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(2000);  // 给足够时间让串口就绪

    Serial.println("=== ESP32 Test Start ===");
    Serial.println("Hello World!");

    // 闪烁LED证明程序在运行
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.println("Setup complete!");
}

void loop() {
    Serial.println("Loop running...");

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}
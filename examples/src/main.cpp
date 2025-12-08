#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== ESP32 Basic Test ===");
    Serial.println("Hello from ESP32-S3!");

    pinMode(2, OUTPUT);
}

void loop() {
    Serial.println("Loop test...");
    digitalWrite(2, HIGH);
    delay(500);
    digitalWrite(2, LOW);
    delay(500);
}
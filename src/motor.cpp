#include <Arduino.h>

// 引脚定义
#define IN1 21
#define IN2 20
#define IN3 1
#define IN4 2

// LEDC 通道
#define CH_A1 0   // Motor A 使用 IN1 的 PWM
#define CH_A2 1   // Motor A 使用 IN2 的 PWM
#define CH_B1 2   // Motor B 使用 IN3 的 PWM
#define CH_B2 3   // Motor B 使用 IN4 的 PWM

// PWM 设置
const int freq = 20000;       // 20kHz，避免可闻噪音
const int resolution = 10;    // 10bit，占空比范围 0~1023

void setup() {
    // 配置 PWM 通道
    ledcSetup(CH_A1, freq, resolution);
    ledcSetup(CH_A2, freq, resolution);
    ledcSetup(CH_B1, freq, resolution);
    ledcSetup(CH_B2, freq, resolution);

    // 绑定引脚到通道
    ledcAttachPin(IN1, CH_A1);
    ledcAttachPin(IN2, CH_A2);
    ledcAttachPin(IN3, CH_B1);
    ledcAttachPin(IN4, CH_B2);

    // 初始停止
    ledcWrite(CH_A1, 0);
    ledcWrite(CH_A2, 0);
    ledcWrite(CH_B1, 0);
    ledcWrite(CH_B2, 0);
}

// 电机 A 控制
void motorA_forward(int speed) {
    ledcWrite(CH_A1, speed);
    ledcWrite(CH_A2, 0);
}

void motorA_backward(int speed) {
    ledcWrite(CH_A1, 0);
    ledcWrite(CH_A2, speed);
}

void motorA_stop() {
    ledcWrite(CH_A1, 0);
    ledcWrite(CH_A2, 0);
}

void motorA_brake() {
    ledcWrite(CH_A1, 1023);
    ledcWrite(CH_A2, 1023);
}

// 电机 B 控制
void motorB_forward(int speed) {
    ledcWrite(CH_B1, speed);
    ledcWrite(CH_B2, 0);
}

void motorB_backward(int speed) {
    ledcWrite(CH_B1, 0);
    ledcWrite(CH_B2, speed);
}

void motorB_stop() {
    ledcWrite(CH_B1, 0);
    ledcWrite(CH_B2, 0);
}

void motorB_brake() {
    ledcWrite(CH_B1, 1023);
    ledcWrite(CH_B2, 1023);
}

void loop() {
    // 示例：A 正转 50%，B 反转 80%
    motorA_forward(1000);
    motorB_backward(1000);  
    delay(2000);
    
    motorA_stop();
    motorB_stop();
    delay(1000);

    // A、B 刹车
    motorA_brake();
    motorB_brake();
    delay(500);
}

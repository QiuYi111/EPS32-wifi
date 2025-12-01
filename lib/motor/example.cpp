#include <Arduino.h>
#include "DualMotor.h"

// IN1, IN2, IN3, IN4, reverseA, reverseB
DualMotor motors(7, 6, 5, 4, true, false);

void setup() {
    motors.begin();
}

void loop() {
    int duty = motors.maxDuty() * 0.6;  // 60% 占空比

    // 逻辑上的“前进”：A、B 同方向正转
    motors.setSpeedA(duty);   // 因为 reverseA=true，实际物理方向已修正
    motors.setSpeedB(duty);
    delay(2000);

    // “后退”
    motors.setSpeedA(-duty);
    motors.setSpeedB(-duty);
    delay(2000);

    // 停止
    motors.stopA();
    motors.stopB();
    delay(1000);

    // 刹车
    motors.brakeA();
    motors.brakeB();
    delay(500);
}

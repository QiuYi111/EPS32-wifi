#pragma once
#include <Arduino.h>

class DualMotor {
public:
    /**
     * @param in1   Motor A IN1 引脚
     * @param in2   Motor A IN2 引脚
     * @param in3   Motor B IN3 引脚
     * @param in4   Motor B IN4 引脚
     * @param reverseA Motor A 是否反转（接线反了就 true）
     * @param reverseB Motor B 是否反转
     * @param freq  PWM 频率
     * @param resolution PWM 分辨率（bit）
     */
    DualMotor(int in1, int in2, int in3, int in4,
              bool reverseA = false,
              bool reverseB = false,
              int freq = 20000,
              int resolution = 10);

    void begin();

    // 速度范围：-maxDuty ~ +maxDuty（负数 = 反转）
    void setSpeedA(int speed);
    void setSpeedB(int speed);

    void stopA();   // 低电平停止（IN1/IN2 = 0）
    void stopB();
    void brakeA();  // 刹车（IN1/IN2 = 1）
    void brakeB();

    int maxDuty() const;

private:
    void driveMotor(int ch1, int ch2, int pin1, int pin2,
                    int speed, bool reverse);

    int _in1, _in2, _in3, _in4;
    bool _revA, _revB;
    int _freq, _res;
    int _maxDuty;
    bool _initialized;
};

#include "DualMotor.h"

// 固定使用 LEDC 通道 0~3
static const int CH_A1 = 0;
static const int CH_A2 = 1;
static const int CH_B1 = 2;
static const int CH_B2 = 3;

DualMotor::DualMotor(int in1, int in2, int in3, int in4,
                     bool reverseA,
                     bool reverseB,
                     int freq,
                     int resolution)
    : _in1(in1), _in2(in2), _in3(in3), _in4(in4),
      _revA(reverseA), _revB(reverseB),
      _freq(freq), _res(resolution),
      _initialized(false)
{
    _maxDuty = (1 << _res) - 1;  // 例如 10bit -> 0~1023
}

void DualMotor::begin() {
    // 设置 PWM 通道
    ledcSetup(CH_A1, _freq, _res);
    ledcSetup(CH_A2, _freq, _res);
    ledcSetup(CH_B1, _freq, _res);
    ledcSetup(CH_B2, _freq, _res);

    // 绑定引脚
    ledcAttachPin(_in1, CH_A1);
    ledcAttachPin(_in2, CH_A2);
    ledcAttachPin(_in3, CH_B1);
    ledcAttachPin(_in4, CH_B2);

    // 全部停止
    stopA();
    stopB();

    _initialized = true;
}

int DualMotor::maxDuty() const {
    return _maxDuty;
}

void DualMotor::driveMotor(int ch1, int ch2, int pin1, int pin2,
                           int speed, bool reverse)
{
    if (!_initialized) return;

    // 方向翻转处理（接反就翻转逻辑）
    if (reverse) {
        speed = -speed;
    }

    // 限幅
    if (speed > _maxDuty)  speed = _maxDuty;
    if (speed < -_maxDuty) speed = -_maxDuty;

    if (speed > 0) {
        // 正转: ch1 = PWM, ch2 = 0
        ledcWrite(ch1, speed);
        ledcWrite(ch2, 0);
    } else if (speed < 0) {
        // 反转: ch1 = 0, ch2 = PWM
        int duty = -speed;
        ledcWrite(ch1, 0);
        ledcWrite(ch2, duty);
    } else {
        // 停止: 两路都 0
        ledcWrite(ch1, 0);
        ledcWrite(ch2, 0);
    }
}

void DualMotor::setSpeedA(int speed) {
    driveMotor(CH_A1, CH_A2, _in1, _in2, speed, _revA);
}

void DualMotor::setSpeedB(int speed) {
    driveMotor(CH_B1, CH_B2, _in3, _in4, speed, _revB);
}

void DualMotor::stopA() {
    if (!_initialized) return;
    ledcWrite(CH_A1, 0);
    ledcWrite(CH_A2, 0);
}

void DualMotor::stopB() {
    if (!_initialized) return;
    ledcWrite(CH_B1, 0);
    ledcWrite(CH_B2, 0);
}

void DualMotor::brakeA() {
    if (!_initialized) return;
    ledcWrite(CH_A1, _maxDuty);
    ledcWrite(CH_A2, _maxDuty);
}

void DualMotor::brakeB() {
    if (!_initialized) return;
    ledcWrite(CH_B1, _maxDuty);
    ledcWrite(CH_B2, _maxDuty);
}

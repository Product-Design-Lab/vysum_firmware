#pragma once

#include <Arduino.h>
#include <cstdint>

class MotorDriver
{
private:
    // pins
    uint8_t PWM_pin = 0;
    uint8_t PWM_pin2 = 0;
    uint8_t PWM_pin_count = 0;
    uint8_t DIR_pin = 0;
    uint8_t DIR_pin2 = 0;
    uint8_t dir_pin_count = 0;
    uint8_t VISEN_pin = 0;
    uint8_t has_visen_pin = 0;
    uint8_t enable_pin = 0;
    uint8_t has_enable_pin = 0;

    // PWM
    uint32_t max_pwm = 255;
    float target_pwm_f32 = 0;
    uint32_t target_pwm_u32 = 0;

    // Direction
    enum
    {
        BACKWARD = 0,
        FORWARD = 1
    };
    bool direction = 0;

    // Misllaneous
    bool is_enabled = 1;
    int current_feedback = 0;

    bool debug_enabled = 0;

public:
    MotorDriver();
    ~MotorDriver();

    void setPwmPin(const uint8_t pin);
    void setPwmPin(const uint8_t pin1, const uint8_t pin2);
    void setDirPin(const uint8_t pin);
    void setDirPin(const uint8_t pin1, const uint8_t pin2);
    void setDirNoPin();
    void setVisenPin(const uint8_t pin); // current sensing pin using analog voltage
    void setVisenNoPin();
    void setEnablePin(const uint8_t pin); // or sleep pin
    void setEnableNoPin();

    void setEnable(bool enable);
    void setMaxPwm(const uint32_t max_pwm);
    void runMotor(const float pwm); //-1 to 1

    int getCurrent();

    void setDebug(bool enable);
};
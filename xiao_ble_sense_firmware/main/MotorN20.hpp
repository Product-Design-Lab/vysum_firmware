#pragma once

#include <Arduino.h>

#include <cstdint>

namespace MotorN20
{
    enum control_mode_e
    {
        CONTROL_PWM,
        CONTROL_POSITION,
        CONTROL_CLAMP,
        CONTROL_SQUEEZE,
        CONTROL_RELEASE
    };

    int8_t init();
    int8_t deinit();
    

    // setter
    void set_mode(const control_mode_e mode);
    void set_clamp_pwm(const float percent);
    void set_squeeze_pwm(const float percent);
    void set_target_pwm(const float percent);
    void set_max_position(const float position);
    void set_target_position(const float position);
    void set_current_position(const float position);
    void set_gain(const float K);
    void enable();
    void disable();

    uint16_t get_current_pwm();
    float get_current_speed();
    float get_current_position();

    void clamp();
    void squeeze();
    void release();

} // namespace MotorN20

#pragma once

// position control

#include <Arduino.h>
#include <cstdint>

namespace PDL_N20_Motor_Control
{

    void setPin(const uint8_t ena, const uint8_t enb, const uint8_t PWM_pin, const uint8_t DIR_pin);
    void setPin(const uint8_t ena, const uint8_t enb, const uint8_t PWM_pin, const uint8_t DIR_pin1, const uint8_t DIR_pin2);

    void setMaxPwm(const uint32_t max_pwm);
    float getPwmPercent(); // percent
    void setPwmPercent(const float percent);

    void setPositionLimits(const int32_t max_pos, const int32_t min_pos); // encoder count value
    void setTargetPosition(const int32_t target_position);

    int32_t getCurrentPosition();
    void setCurrentPosition(const int32_t current_position);

    void setGain(const float K);

    void enable();
    void disable();

    void enableDebug();
    void disableDebug();

    float getCurrentSpeed();

    void init();

    void deinit();

} // namespace PDL_N20_Motor_Control
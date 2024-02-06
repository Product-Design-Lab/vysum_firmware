#pragma once

// position control

#include <Arduino.h>
#include <cstdint>

namespace PDL_N20_Motor_Control
{

    void setPin(uint8_t ena, uint8_t enb, uint8_t PWM_pin, uint8_t DIR_pin);
    void setPin(uint8_t ena, uint8_t enb, uint8_t PWM_pin, uint8_t DIR_pin1, uint8_t DIR_pin2);

    void setMaxPwm(uint32_t max_pwm);
    float getPwmPercent(); // percent
    void setPwmPercent(float percent);

    void setPositionLimits(int32_t max_pos, int32_t min_pos); // encoder count value
    void setTargetPosition(int32_t target_position);

    int32_t getCurrentPosition();
    void setCurrentPosition(int32_t current_position);

    void setGain(float K);

    void enable();
    void disable();

    void enableDebug();
    void disableDebug();

    float getCurrentSpeed();

    void init();

    void deinit();

} // namespace PDL_N20_Motor_Control
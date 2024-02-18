#pragma once

#include "motor_driver.h"
#include "RotaryEncoder.h"
#include "FreeRTOS.h"
#include "task.h"

class MotorController
{
private:
    enum control_mode_e
    {
        CONTROL_PWM,
        CONTROL_POSITION
    };
    control_mode_e control_mode = CONTROL_PWM;

    MotorDriver &motor;
    HwRotaryEncoder &encoder;
    TaskHandle_t motorTaskHandle = NULL;
    int32_t max_pos;
    int32_t min_pos;
    int32_t target_position;
    int32_t current_position;
    float current_speed;
    float Kp;
    float error;
    float u;
    bool debug_enabled = false;

    static void motorTaskWrapper(void *parameter);
    void motorTask();

public:
     MotorController(MotorDriver& motor, HwRotaryEncoder& encoder);
    ~MotorController();

    // take pointer to motor driver object
    void setMotorDriver(MotorDriver *motor);
    void setEncoder(HwRotaryEncoder *encoder);

    void setPositionLimits(const int32_t max_pos, const int32_t min_pos); // encoder count value
    void setTargetPosition(const int32_t target_position);

    void setPwm(float u);

    int32_t getCurrentPosition();
    void setCurrentPosition(const int32_t current_position);

    void setGain(const float K);

    void setDebug(const bool debug);
    void printDebug();

    void start(uint8_t priority = 1);
};

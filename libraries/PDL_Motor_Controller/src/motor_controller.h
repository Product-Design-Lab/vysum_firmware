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
    float Kp, Ki;
    float error, error_integral;
    float u;
    TickType_t xFrequency = 10; // loop delay in Ticks, note that 1 Tick is not always 1 ms, refer to FreeRTOSconfig.h
    bool debug_enabled = false;

    static void motorTaskWrapper(void *parameter);
    void motorTask();

public:
    MotorController(MotorDriver &motor, HwRotaryEncoder &encoder);
    ~MotorController();

    void setPositionLimits(const int32_t max_pos, const int32_t min_pos); // encoder count value
    void setTargetPosition(const int32_t target_position);

    int32_t getCurrentPosition();
    void setCurrentPosition(const int32_t current_position);
    float getCurrentSpeed();

    void setPwm(float u);
    void setGain(const float Kp, const float Ki = 0.0);
    void setLoopDelay(const uint32_t delay_ms);

    void setDebug(const bool debug);
    void printDebug();

    void start(uint8_t priority = 1);
    void pause();
};

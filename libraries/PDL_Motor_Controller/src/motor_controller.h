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
    int32_t max_pos = 0;
    int32_t min_pos = 0;
    int32_t target_position = 0;
    int32_t current_position = 0;

    uint32_t position_torlerance = 100; // position within this range is considered reached
    bool target_reached = false;
    bool onTargetReachCalled = false;

    uint32_t stall_threshold_ms = 500; // in ms
    bool motor_stalled = false;
    bool onMotorStallCalled = false;

    float current_speed = 0;
    float Kp = 0, Ki = 0, Kd = 0;
    float error = 0, error_integral = 0, error_derivative = 0;
    float u = 0;
    TickType_t xFrequency = 10; // loop delay in Ticks, note that 1 Tick is not always 1 ms, refer to FreeRTOSconfig.h
    uint8_t debug_option = 0;

    static void motorTaskWrapper(void *parameter);
    void motorTask();
    void checkTargetReach();
    void checkMotorStall();
    void pid_position_control();

public:
    enum debug_status_e
    {
        DEBUG_OFF,
        DEBUG_CURRENT,
        DEBUG_CONTROL_LOOP,
        DEBUG_EVENT,
        DEBUG_MAX,
    };

    MotorController(MotorDriver &motor, HwRotaryEncoder &encoder) : motor(motor), encoder(encoder){};
    ~MotorController(){};

    void setPositionLimits(const int32_t max_pos, const int32_t min_pos); // encoder count value
    void setTargetPosition(const int32_t target_position);
    void setPositonTolerance(const uint32_t position_torlerance) { this->position_torlerance = position_torlerance; }

    void setStallThreshold(const uint32_t stall_threshold_ms) { this->stall_threshold_ms = stall_threshold_ms; }

    int32_t getCurrentPosition() { return current_position; };
    void setCurrentPosition(const int32_t current_position);
    float getCurrentSpeed() { return current_speed; };
    bool isMotorStalled() { return motor_stalled; };
    bool isTargetReached() { return target_reached; };

    void setPwm(float u);
    void setGain(const float Kp, const float Ki = 0.0, const float Kd = 0.0);
    void setLoopDelay(const uint32_t delay_ms) { this->xFrequency = pdMS_TO_TICKS(delay_ms); };
    void setDebug(const uint8_t debug) { this->debug_option = debug; };
    void printDebug();

    void start(uint8_t priority = 1);
    void pause();

    // Motor Event callbacks

    typedef void (*MotorEventCallback)();
    MotorEventCallback onMotorStall = NULL;
    MotorEventCallback onTargetReach = NULL;

    void setOnMotorStall(MotorEventCallback callback);
    void setOnTargetReach(MotorEventCallback callback);
};

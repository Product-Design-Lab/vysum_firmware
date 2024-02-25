#pragma once

#include <Arduino.h>
#include "FreeRTOS.h"
#include "timers.h"

#define MAX_PIN_NUM 2 // need to manually add static ISR functions if this number is changed

class AsyncButtonGroup
{
public:
    enum ButtonState
    {
        BUTTON_IDLE,
        BUTTON_DEBOUCE,
        BUTTON_SHORT_PRESS,
        BUTTON_LONG_PRESS,
        BUTTON_MAX_STATES
    };

private:
    //create a static array of pointers to the class instances
    static AsyncButtonGroup *instances[MAX_PIN_NUM];
    static uint8_t _instance_count;

    uint8_t _idx;
    uint8_t _pin;
    uint32_t _debounceTime;
    uint32_t _longPressTime;
    bool _idle_logic_level;
    int _short_press_count;
    int _long_press_count;
    uint8_t _state;
    uint8_t _output_state;

    xTimerHandle _timerHandle1;
    xTimerHandle _timerHandle2;



    void _gpioCallback();
    static void _gpioCallback_instance1();
    static void _gpioCallback_instance2();

    void _TimerCallback(TimerHandle_t xTimer);
    static void _TimerCallback_instance1(TimerHandle_t xTimer);
    static void _TimerCallback_instance2(TimerHandle_t xTimer);

    void set_init_state();
    void set_debounce_state();
    void set_short_press_state();
    void set_long_press_state();

public:
    AsyncButtonGroup();
    void setPin(uint8_t pin);
    void setPin(uint8_t pin, bool idle_logic_level = HIGH);
    void setDebounceTime(uint32_t ms);
    void setLongPressTime(uint32_t ms);
    void setIdleLogicLevel(bool logic_level);
    uint8_t getState(int *short_press_count, int *long_press_count);
    uint8_t getState();

    void init();
};
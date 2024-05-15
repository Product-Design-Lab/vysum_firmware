#pragma once

#include <Arduino.h>
#include "FreeRTOS.h"
#include "timers.h"

#define MAX_PIN_NUM 2

class AsyncButtonGroup
{
public:
    using ButtonCallback = void (*)();  // Define a function pointer type for callbacks

    enum ButtonState
    {
        BUTTON_IDLE,
        BUTTON_DEBOUNCE,
        BUTTON_SHORT_PRESS,
        BUTTON_LONG_PRESS,
        BUTTON_MAX_STATES
    };

private:
    static AsyncButtonGroup *instances[MAX_PIN_NUM];
    static uint8_t instance_count;

    uint8_t idx;
    uint8_t pin;
    uint32_t debounceTime;
    uint32_t longPressTime;
    bool idle_logic_level;
    int short_press_count;
    int long_press_count;
    uint8_t state;
    uint8_t output_state;

    TimerHandle_t timerHandle;
    ButtonCallback shortPressCallback; // Callback for short press
    ButtonCallback longPressCallback;  // Callback for long press

    void gpioCallback();
    static void gpioCallbackInstance1();
    static void gpioCallbackInstance2();

    void timerCallback(TimerHandle_t xTimer);
    static void timerCallbackInstance1(TimerHandle_t xTimer);
    static void timerCallbackInstance2(TimerHandle_t xTimer);

    void setInitialState();
    void setDebounceState();
    void setShortPressState();
    void setLongPressState();

public:
    AsyncButtonGroup();
    void setPin(uint8_t pin, bool idle_logic_level = HIGH);
    void setDebounceTime(uint32_t ms);
    void setLongPressTime(uint32_t ms);
    void setIdleLogicLevel(bool logic_level);
    uint8_t getState(int *short_press_count, int *long_press_count);
    uint8_t getState();
    void init();

    void setShortPressCallback(ButtonCallback callback); // Set short press callback
    void setLongPressCallback(ButtonCallback callback);  // Set long press callback
};

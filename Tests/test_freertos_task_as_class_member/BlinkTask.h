// Ensure this header file is only included once in the compile process
#pragma once

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

class BlinkTask {
public:
    BlinkTask(uint8_t ledPin, int blinkIntervalMs);
    void start();

private:
    static void blinkTaskWrapper(void *parameter);
    void blinkTask(); // The actual task that will blink the LED

    uint8_t _ledPin;
    int _blinkIntervalMs;
    TaskHandle_t _taskHandle; // Handle for the created FreeRTOS task
};

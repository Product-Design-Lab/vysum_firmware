#include "BlinkTask.h"

BlinkTask::BlinkTask(uint8_t ledPin, int blinkIntervalMs) : _ledPin(ledPin), _blinkIntervalMs(blinkIntervalMs) {
    pinMode(_ledPin, OUTPUT);
}

void BlinkTask::start() {
    xTaskCreate(
        BlinkTask::blinkTaskWrapper, // Task function wrapper
        "BlinkTask",                 // Name of the task (for debugging)
        1024,                        // Stack size (bytes)
        this,                        // Parameter passed to the task
        1,                           // Priority
        &_taskHandle                 // Task handle
    );
}

void BlinkTask::blinkTaskWrapper(void *parameter) {
    // Cast the parameter back to the correct class instance and call the member function
    static_cast<BlinkTask*>(parameter)->blinkTask();
}

void BlinkTask::blinkTask() {
    while (1) {
        digitalWrite(_ledPin, HIGH);
        vTaskDelay(_blinkIntervalMs);
        digitalWrite(_ledPin, LOW);
        vTaskDelay(_blinkIntervalMs);
    }
}

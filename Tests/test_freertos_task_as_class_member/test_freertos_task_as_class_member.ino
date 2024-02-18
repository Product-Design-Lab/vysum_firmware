#include "BlinkTask.h"

BlinkTask *blinkTask;       // Pointer for dynamically allocated BlinkTask
BlinkTask blinkTask2(12, 250); // Stack-allocated BlinkTask instance

void setup() {
    blinkTask = new BlinkTask(11, 500); // Dynamically allocate and start the first blinking task
    blinkTask->start();
    blinkTask2.start(); // Start the second blinking task
}

void loop() {
    delay(100); // Main loop does nothing, just delays
}

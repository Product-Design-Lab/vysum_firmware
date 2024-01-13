#pragma once

#include <cstdbool>

#define IMU_TASK_PRIORITY 1



// rtos task stack size
#define IMU_TASK_STACK_SIZE 1024


//rtos task delay
#define IMU_LOOP_DELAY_MS 10

namespace IMU {
    void init(float alphaValue = 0.5f);
    void deinit();
    bool isVertical();
}

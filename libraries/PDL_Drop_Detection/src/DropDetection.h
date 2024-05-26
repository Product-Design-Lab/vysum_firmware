#pragma once

#include "APDS_Data.h"
#include <Arduino_APDS9960.h>
#include "FreeRTOS.h"
#include "task.h"

class WaterDropSensor {
public:
    enum DEBUG_FLAG {
        DEBUG_NONE,
        DEBUG_INFO,
        DEBUG_RAW,
        DEBUG_CALIB,
        DEBUG_ZEROING,
        DEBUG_LOWPASS,
        DEBUG_DOT,
        DEBUG_LR,
        DEBUG_CROSSING_STATE_PRINT,
        DEBUG_CROSSING_STATE_PLOT,
        DEBUG_FREQ,
        DEBUG_MAX
    };

    WaterDropSensor();
    ~WaterDropSensor();

    void init(uint32_t priority = 1);
    void deinit();
    void resume();
    void pause();
    int getDropCount() const;
    void setDropCount(int count);

    void setDebug(uint8_t debug);
    void setCrossCountTrigThreshold(int threshold);
    void setDebouceWindowSize(uint8_t count);
    void setLoopDelayMs(uint32_t ms);

    void setBoundsLR(uint8_t bound);
    void setBoundsLP(uint8_t bound);
    void setBoundsDot(uint8_t bound);

private:
    void printDebug();
    static void dropDetectionTask(void *pvParameters);
    void runMainTaskLogic();

    constexpr static uint8_t DEFAULT_LR_THRESHOLD = 4;
    constexpr static uint8_t DEFAULT_LP_THRESHOLD = 6;
    constexpr static float DEFAULT_DOT_THRESHOLD = 2.5;
    constexpr static uint32_t TASK_STACK_SIZE = 2048;
    constexpr static uint8_t MAX_LOOP_DELAY_MS = 80;
    constexpr static int DEFAULT_DEBOUNCE_WINDOW_SIZE = 100;
    constexpr static int DEFAULT_CROSS_COUNT_TRIG_THRESHOLD = 2;

    int debounceWindowSize;
    int crossingCountTrigThreshhold;
    uint8_t debugFlag;
    int dropCount;

    APDS_Data data;
    APDS_Data::data_crossing_state_t crossing_state;

    StaticTask_t xTaskBuffer;
    StackType_t xStack[TASK_STACK_SIZE];
    TaskHandle_t dropDetectionTaskHandle;
    eTaskState task_state;

    TickType_t X_FREQUENCY;
    TickType_t xLastWakeTime;
};

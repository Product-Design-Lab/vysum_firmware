#pragma once

#include "APDS_Data.h"
#include <Arduino_APDS9960.h>

namespace APDS_DropSensor
{
    enum DEBUG_FLAG
    {
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

    void init(uint32_t priority = 1);
    void deinit();
    void resume();
    void pause();
    int get_drop_count();
    void set_drop_count(int count);

    void setDebug(uint8_t debug);
    void setCrossCountTrigThreshold(int threshold);
    void setDebouceWindowSize(uint8_t count);
    void setLoopDelayMs(uint32_t ms);

    void setBoundsLR(uint8_t bound);
    void setBoundsLP(uint8_t bound);
    void setBoundsDot(uint8_t bound);
}

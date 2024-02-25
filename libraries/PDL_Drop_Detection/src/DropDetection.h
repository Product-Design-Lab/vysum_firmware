#pragma once

// C includes

// Arduino includes



// project includes
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
        DEBUG_CORSSING_STATE_PLOT,
        DEBUG_FREQ,
        DEBUG_MAX
    };

    void init(uint32_t priority = 1);
    void deinit();
    void resume();
    void pause();
    int get_drop_count();
    void set_drop_count(const int count);

    void setDebug(uint8_t debug);

};

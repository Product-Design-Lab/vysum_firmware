#pragma once

// C includes

// Arduino includes



// project includes
#include "APDS_Data.h"
#include <Arduino_APDS9960.h>

namespace APDS_DropSensor
{

    void init();
    void deinit();
    void start();
    void stop();
    int get_drop_count();
    void set_drop_count(const int count);

};

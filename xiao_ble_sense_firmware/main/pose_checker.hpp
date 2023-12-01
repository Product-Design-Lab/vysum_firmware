#pragma once

#include <cstdbool>

namespace IMU {
    void init(float alphaValue = 0.5f);
    void deinit();
    bool isVertical();
}

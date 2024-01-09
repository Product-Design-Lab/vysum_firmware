#pragma once

#include <cstdint>

namespace DIAG
{

    enum : uint32_t
    {
        D_OFF = 0,
        D_MOTOR = 1 << 0,
        D_IMU = 1 << 1,
        D_DROP = 1 << 2,
        D_BUTTON = 1 << 3,
        DIAG_OPT_5 = 1 << 4,
        DIAG_OPT_MAX,
    };

    int8_t init();
    int8_t deinit();

    uint32_t get_opt();

} // namespace DIAG
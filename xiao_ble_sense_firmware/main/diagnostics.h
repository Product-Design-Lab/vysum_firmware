#pragma once

#include <stdint.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C"
{
#endif

    enum : uint32_t
    {
        DIAG_OFF = 0,
        DIAG_OPT_1 = 1 << 0,
        DIAG_OPT_2 = 1 << 1,
        DIAG_OPT_3 = 1 << 2,
        DIAG_OPT_4 = 1 << 3,
        DIAG_OPT_5 = 1 << 4,
        DIAG_OPT_MAX,
    };

    int8_t DIAG_init();
    int8_t DIAG_deinit();

    uint32_t DIAG_get_opt();

    void serialPrintf(const char *format, ...);

#ifdef __cplusplus
}
#endif
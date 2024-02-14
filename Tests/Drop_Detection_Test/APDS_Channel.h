#pragma once

#include <cstdint>
#include <MovingAverage.h>

#define MAX_SAMPLES 32
#define MOVING_AVG_SAMPLES 5

class APDS_Channel
{
public:
    MovingAverage<int16_t, MOVING_AVG_SAMPLES> LP_filter;
    uint8_t count;
    uint8_t calibValue;
    uint8_t buffer[MAX_SAMPLES]; // buffer for raw data
    uint8_t raw_u8[MAX_SAMPLES]; // data will be copied here to prevent race condition
    int16_t raw_i16[MAX_SAMPLES];
    float lp[MAX_SAMPLES]; // lowpass
    float last_val;
    float dot[MAX_SAMPLES]; // temporal difference
    float up_b_lp, low_b_lp, up_b_dot, low_b_dot;

public:
    // constructor
    APDS_Channel() {}
    
    void calib(const bool is_initial);
    void copy_buffer();
    void zero_offset();
    void lowpass();
    void diff();
    void set_bounds_lp(const float up_b_lp, const float low_b_lp);
    void set_bounds_dot(const float up_b_dot, const float low_b_dot);
    uint8_t check_crossing();
};

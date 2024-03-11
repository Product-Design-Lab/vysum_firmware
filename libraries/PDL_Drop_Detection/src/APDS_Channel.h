#pragma once

#include <cstdint>
#include <MovingAverage.h>

#define MAX_SAMPLES 32
#define MOVING_AVG_SAMPLES 3

class APDS_Channel
{
public:
    union channel_crossing_state_t
    {
        struct
        {
            bool LP_CROSS_UPPER_BOUND : 1;
            bool LP_CROSS_LOWER_BOUND : 1;
            bool DOT_CROSS_UPPER_BOUND : 1;
            bool DOT_CROSS_LOWER_BOUND : 1;
        } __attribute__((packed));
        uint8_t state : 4;
    };

public:
    uint8_t raw_u8[MAX_SAMPLES];
    int16_t raw_i16[MAX_SAMPLES];
    float lp[MAX_SAMPLES];  // low pass filter
    float dot[MAX_SAMPLES]; // temporal difference
    uint8_t calibValue;
    uint8_t count;

private:
    MovingAverage<int16_t, MOVING_AVG_SAMPLES> LP_filter;
    float last_val;
    float up_b_lp, low_b_lp, up_b_dot, low_b_dot;

public:
    // constructor
    APDS_Channel() {}

    void calib(const bool is_initial);
    void zero_offset();
    void lowpass();
    void diff();
    void set_bounds_lp(const float up_b_lp, const float low_b_lp);
    void set_bounds_dot(const float up_b_dot, const float low_b_dot);
    channel_crossing_state_t check_crossing_state();
};

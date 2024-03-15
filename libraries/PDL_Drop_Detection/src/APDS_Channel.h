#pragma once

#include <cstdint>
#include <MovingAverage.h>

#define MAX_SAMPLES 32
#define MOVING_AVG_SAMPLES 4

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

private:
    uint8_t raw_u8[MAX_SAMPLES];
    int raw_i32[MAX_SAMPLES];
    int lp[MAX_SAMPLES];  // low pass filter
    int dot[MAX_SAMPLES]; // temporal difference
    uint8_t calibValue;

    uint8_t count;
    MovingAverage<int, MOVING_AVG_SAMPLES> LP_filter;
    int last_val;
    int up_b_lp, low_b_lp, up_b_dot, low_b_dot;
    void calib(const bool is_initial);
    void zero_offset();
    void lowpass();
    void diff();
    channel_crossing_state_t check_crossing_state();

public:
    // constructor
    APDS_Channel() {}

    channel_crossing_state_t process_single_channel(int sample_count);
    void set_bounds_lp(const int up_b_lp, const int low_b_lp);
    void set_bounds_dot(const int up_b_dot, const int low_b_dot);

    uint8_t *get_raw_u8() { return raw_u8; }
    const int *get_raw_i32() { return raw_i32; }
    const int *get_lp() { return lp; }
    const int *get_dot() { return dot; }
    uint8_t get_calibValue() { return calibValue; }
};

#pragma once

#include "APDS_Channel.h"

class APDS_Data
{
public:
    union channel_pair_crossing_state_t
    {
        struct
        {
            bool RISE_OVER_UPPER_BOUND : 1;
            bool FALL_BELOW_UPPER_BOUND : 1;
            bool RISE_OVER_LOWER_BOUND : 1;
            bool FALL_BELOW_LOWER_BOUND : 1;
        } __attribute__((packed));
        uint8_t state : 4;
    };

    /* check if the data crosses the upper and lower bounds
    state bit field:
     | reserved | lr_state | r_state | l_state | d_state | u_state |
     |  32 - 20 |  19 - 16 | 15 - 12 |  11 - 8 |  7 - 4  |  3 - 0  |
    */
    union data_corssing_state_t
    {
        struct
        {
            APDS_Channel::channel_crossing_state_t u;
            APDS_Channel::channel_crossing_state_t d;
            APDS_Channel::channel_crossing_state_t l;
            APDS_Channel::channel_crossing_state_t r;
            channel_pair_crossing_state_t lr;
        } __attribute__((packed));
        uint32_t state : 20;
    };

public:
    APDS_Channel u;
    APDS_Channel d;
    APDS_Channel l;
    APDS_Channel r;

    int sample_count;

private:
    float lr_diff[MAX_SAMPLES];
    float lr_diff_prev;
    float up_b_lr, low_b_lr;
    data_corssing_state_t crossing_state;
    void compute_lr_diff();
    channel_pair_crossing_state_t check_lr_crossing_state();

public:
    // constructor
    APDS_Data();
    void set_bounds_lr(const float up_b_lr, const float low_b_lr);
    void calib(const bool is_initial);
    // void copy_buffer();
    void process();
    data_corssing_state_t get_crossing_state();

    void printRaw();
    void printCalib();
    void printRaw_i16();
    void printLP();
    void printDot();
    void printLR();
    void printCrossingState(uint32_t val);
    void plotCrossingState(uint32_t val);
};

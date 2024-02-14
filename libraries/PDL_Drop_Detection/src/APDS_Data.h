#pragma once

#include "APDS_Channel.h"

class APDS_Data
{
public:
    APDS_Channel u;
    APDS_Channel d;
    APDS_Channel l;
    APDS_Channel r;
    int sample_count;
    float lr_diff[MAX_SAMPLES];
    float lr_diff_prev;
    float up_b_lr, low_b_lr;
    uint32_t crossing_state;

public:
    // constructor
    APDS_Data();
    void set_bounds_lr(const float up_b_lr, const float low_b_lr);
    void calib(const bool is_initial);
    void copy_buffer();
    void process();
    uint32_t get_crossing_state();

    void printRaw();
    void printCalib();
    void printRaw_i16();
    void printLP();
    void printDot();
    void printLR();
    void printCrossingState(uint32_t val);
    void plotCrossingState(uint32_t val);
};

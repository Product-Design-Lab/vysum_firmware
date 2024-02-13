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
    float up_b_lr, low_b_lr;

public:
    // constructor
    APDS_Data();
    void set_bounds_lr(const float up_b_lr, const float low_b_lr);
    void calib(const bool is_initial);
    void copy_buffer();
    void process();
    uint32_t check_crossing();

    void printRaw();
    void printCalib();
    void printRaw_i16();
    void printLP();
    void printDot();
    void printLR();
};

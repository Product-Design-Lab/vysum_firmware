#ifndef DROP_DETECTION_H
#define DROP_DETECTION_H

#include <cstdint>
#include <MovingAverage.h>
#include <Arduino_APDS9960.h>

#define MAX_SAMPLES 32
#define MOVING_AVG_SAMPLES 5



class APDS_DataChannel
{
public:

    MovingAverage<int16_t, MOVING_AVG_SAMPLES> LP_filter;
    uint8_t count;
    uint8_t calibValue;
    uint8_t buffer[MAX_SAMPLES];// buffer for raw data
    uint8_t raw_u8[MAX_SAMPLES];// data will be copied here to prevent race condition
    int16_t raw_i16[MAX_SAMPLES];
    float lp[MAX_SAMPLES]; // lowpass
    float last_val;
    float dot[MAX_SAMPLES]; // temporal difference
    float up_b_lp, low_b_lp, up_b_dot, low_b_dot;

public:
    // constructor
    APDS_DataChannel() {}
    void calib(const bool is_initial);
    void copy_buffer();
    void zero_offset();
    void lowpass();
    void diff();
    void set_bounds_lp(const float up_b_lp, const float low_b_lp);
    void set_bounds_dot(const float up_b_dot, const float low_b_dot);
    uint8_t check_crossing();
};

class APDS_Data
{
public:
    APDS_DataChannel u;
    APDS_DataChannel d;
    APDS_DataChannel l;
    APDS_DataChannel r;
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

class APDS_DropSensor
{
public:
    APDS_Data data;

    void init();
    void update();

};

extern APDS_DropSensor drop_sensor;

#endif

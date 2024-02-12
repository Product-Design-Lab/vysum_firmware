#ifndef DROP_DETECTION_H
#define DROP_DETECTION_H

#include <cstdint>
#include <MovingAverage.h>
#include <Arduino_APDS9960.h>

#define MAX_SAMPLES 32
#define MOVING_AVG_SAMPLES 3

class APDS_DataChannel
{
public:
    MovingAverage<int16_t, MOVING_AVG_SAMPLES> LP_filter;
    uint8_t count;
    uint8_t calibValue;
    uint8_t raw_u8[MAX_SAMPLES];
    int16_t raw_i16[MAX_SAMPLES];
    float lp[MAX_SAMPLES]; // lowpass 1
    float last_val;
    float dot[MAX_SAMPLES]; // temporal difference
    float up_b_lp, low_b_lp, up_b_dot, low_b_dot;

public:
    // constructor
    APDS_DataChannel() {}
    void calib();
    void zero_offset();
    void lowpass();
    void diff();
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
    void calib();
    void process();
    void printRaw();
    void printRaw_i8();
    void printLP();
    void printDot();
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

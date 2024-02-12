// #include "DropDetection.h"

#include <MovingAverage.h>
#include <Arduino_APDS9960.h>
#include "Adafruit_TinyUSB.h"

#define MAX_SAMPLES 32
#define MOVING_AVG_SAMPLES 3

class APDS_DataChannel
{

public:
    MovingAverage<int8_t, MOVING_AVG_SAMPLES> LP_filter;
    uint8_t count;
    uint8_t calibValue;
    uint8_t raw_u8[MAX_SAMPLES];
    int8_t raw_i8[MAX_SAMPLES];
    float lp[MAX_SAMPLES]; // lowpass 1
    float last_val;
    float dot[MAX_SAMPLES]; // temporal difference
    float up_b_lp, low_b_lp, up_b_dot, low_b_dot;

public:
    // constructor
    APDS_DataChannel() {}
    void calib()
    {
        calibValue = raw_u8[0];
    }

    void zero_offset()
    {
        for (int i = 0; i < count; i++)
        {
            raw_i8[i] = (int)(raw_u8[i] - calibValue);
        }
    }

    void lowpass()
    {
        for (int i = 0; i < count; i++)
        {
            lp[i] = LP_filter.add(raw_i8[i]);
        }
    }

    void diff()
    {
        if (count == 0)
        {
            return;
        }
        dot[0] = last_val - lp[0];
        for (int i = 1; i < count; i++)
        {
            dot[i] = lp[i] - lp[i - 1];
        }
    }
};

class APDS_Data
{
public:
    APDS_DataChannel u;
    APDS_DataChannel d;
    APDS_DataChannel l;
    APDS_DataChannel r;
    int sample_count;
    float lr_diff[32];
    float up_b_lr, low_b_lr;

private:
    void updateSampleCount()
    {
        u.count = sample_count;
        d.count = sample_count;
        l.count = sample_count;
        r.count = sample_count;
    }

    void zero_offset()
    {
        u.zero_offset();
        d.zero_offset();
        l.zero_offset();
        r.zero_offset();
    }

    void lowpass()
    {
        u.lowpass();
        d.lowpass();
        l.lowpass();
        r.lowpass();
    }

    void diff()
    {
        u.diff();
        d.diff();
        l.diff();
        r.diff();
    }

public:
    // constructor
    APDS_Data()
    {
        u = APDS_DataChannel();
        d = APDS_DataChannel();
        l = APDS_DataChannel();
        r = APDS_DataChannel();
    }

    void calib()
    {
        u.calib();
        d.calib();
        l.calib();
        r.calib();
    }

    void process()
    {
        updateSampleCount();
        zero_offset();
        lowpass();
        diff();
    }

    void printRaw()
    {
        for (int i = 0; i < sample_count; i++)
        {
            Serial.printf("l=%d, r=%d, u=%d, d=%d\n", l.raw_u8[i], r.raw_u8[i], u.raw_u8[i], d.raw_u8[i]);
        }
    }

    void printRaw_i8()
    {
        for (int i = 0; i < sample_count; i++)
        {
            Serial.printf("l=%d, r=%d, u=%d, d=%d\n", l.raw_i8[i], r.raw_i8[i], u.raw_i8[i], d.raw_i8[i]);
        }
    }

    void printLP()
    {
        for (int i = 0; i < sample_count; i++)
        {
            Serial.printf("l=%.2f, r=%.2f, u=%.2f, d=%.2f\n", l.lp[i], r.lp[i], u.lp[i], d.lp[i]);
        }
    }

    void printDot()
    {
        for (int i = 0; i < sample_count; i++)
        {
            Serial.printf("dl=%.2f, dr=%.2f, du=%.2f, dd=%.2f\n", l.dot[i], r.dot[i], u.dot[i], d.dot[i]);
        }
    }
};

class APDS_DropSensor
{
public:
    APDS_Data data;

    void init()
    {
        if (!APDS.begin())
        {
            Serial.println("Error initializing APDS-9960 sensor!");
        }
        else
        {
            Serial.println("APDS-9960 initialization complete");
        }

        for (int i = 0; i < 32; i++)
        {
            data.sample_count = APDS.gestureAvailable(data.u.raw_u8, data.d.raw_u8, data.l.raw_u8, data.r.raw_u8);
            data.printRaw();
            delay(10);
        }
        data.calib();
    }

    void update()
    {
        data.sample_count = APDS.gestureAvailable(data.u.raw_u8, data.d.raw_u8, data.l.raw_u8, data.r.raw_u8);
        data.process();
        data.printLP();
    }
};

APDS_DropSensor drop_sensor;


void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    drop_sensor.init();
}

void loop()
{
    drop_sensor.update();
    delay(10);
}
#include "dropDetection.h"

void APDS_DataChannel::calib()
{
    calibValue = raw_u8[0];
}

void APDS_DataChannel::zero_offset()
{
    for (int i = 0; i < count; i++)
    {
        raw_i16[i] = raw_u8[i] - calibValue;
    }
}

void APDS_DataChannel::lowpass()
{
    for (int i = 0; i < count; i++)
    {
        lp[i] = LP_filter.add(raw_i16[i]);
    }
}

void APDS_DataChannel::diff()
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

APDS_Data::APDS_Data()
{
    sample_count = 0;
}

void APDS_Data::calib()
{
    u.calib();
    d.calib();
    l.calib();
    r.calib();
}

void APDS_Data::process()
{
    u.count = sample_count;
    d.count = sample_count;
    l.count = sample_count;
    r.count = sample_count;
    u.zero_offset();
    d.zero_offset();
    l.zero_offset();
    r.zero_offset();
    u.lowpass();
    d.lowpass();
    l.lowpass();
    r.lowpass();
    u.diff();
    d.diff();
    l.diff();
    r.diff();
    
}

void APDS_Data::printRaw()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("l=%d, r=%d, u=%d, d=%d\n", l.raw_u8[i], r.raw_u8[i], u.raw_u8[i], d.raw_u8[i]);
    }
}

void APDS_Data::printRaw_i8()
{
    Serial.printf("calibValue: cl=%d, cr=%d, cu=%d, cd=%d\n", l.calibValue, r.calibValue, u.calibValue, d.calibValue);
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("l=%d, r=%d, u=%d, d=%d\n", l.raw_i16[i], r.raw_i16[i], u.raw_i16[i], d.raw_i16[i]);
    }
}

void APDS_Data::printLP()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("l=%.2f, r=%.2f, u=%.2f, d=%.2f\n", l.lp[i], r.lp[i], u.lp[i], d.lp[i]);
    }
}

void APDS_Data::printDot()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("dl=%.2f, dr=%.2f, du=%.2f, dd=%.2f\n", l.dot[i], r.dot[i], u.dot[i], d.dot[i]);
    }
}

void APDS_DropSensor::init()
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

void APDS_DropSensor::update()
{
    data.sample_count = APDS.gestureAvailable(data.u.raw_u8, data.d.raw_u8, data.l.raw_u8, data.r.raw_u8);
    data.process();
    data.printLP();
}

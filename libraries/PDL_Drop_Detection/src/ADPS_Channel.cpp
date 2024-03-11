#include "APDS_Channel.h"
#include <cstring>

void APDS_Channel::calib(const bool is_initial = 1)
{
    if (is_initial)
        calibValue = raw_u8[0];

    if (calibValue < raw_u8[0])
        calibValue++;
    else if (calibValue > raw_u8[0])
        calibValue--;
}

void APDS_Channel::zero_offset()
{
    for (int i = 0; i < count; i++)
    {
        raw_i16[i] = raw_u8[i] - calibValue;
    }
}

#include <Adafruit_TinyUSB.h>//TODO: remove this aftern testing
void APDS_Channel::lowpass()
{
    for (int i = 0; i < count; i++)
    {
        
        lp[i] = LP_filter.add(raw_i16[i]);
        // lp[i] = raw_i16[i]*0.2 + lp[i]*0.8;
        // Serial.printf("raw_i16[%d]:%d, lp[%d]:%d\n", i, raw_i16[i], i, lp[i]);
    }
}

void APDS_Channel::diff()
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
    last_val = lp[count - 1];
}

void APDS_Channel::set_bounds_lp(const float up_b_lp, const float low_b_lp)
{
    this->up_b_lp = up_b_lp;
    this->low_b_lp = low_b_lp;
}

void APDS_Channel::set_bounds_dot(const float up_b_dot, const float low_b_dot)
{
    this->up_b_dot = up_b_dot;
    this->low_b_dot = low_b_dot;
}

APDS_Channel::channel_crossing_state_t APDS_Channel::check_crossing_state()
{
    APDS_Channel::channel_crossing_state_t crossing_state = {};
    for (int i = 0; i < count; i++)
    {
        crossing_state.LP_CROSS_UPPER_BOUND |= (bool)(lp[i] > up_b_lp);
        crossing_state.LP_CROSS_LOWER_BOUND |= (bool)(lp[i] < low_b_lp);
        crossing_state.DOT_CROSS_UPPER_BOUND |= (bool)(dot[i] > up_b_dot);
        crossing_state.DOT_CROSS_LOWER_BOUND |= (bool)(dot[i] < low_b_dot);
    }
    return crossing_state;
}

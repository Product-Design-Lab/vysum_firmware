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

void APDS_Channel::copy_buffer()
{
    memcpy(raw_u8, buffer, sizeof(uint8_t) * count);
}

void APDS_Channel::zero_offset()
{
    for (int i = 0; i < count; i++)
    {
        raw_i16[i] = raw_u8[i] - calibValue;
    }
}

void APDS_Channel::lowpass()
{
    for (int i = 0; i < count; i++)
    {
        lp[i] = LP_filter.add(raw_i16[i]);
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

uint8_t APDS_Channel::check_crossing()
{
    uint8_t state = 0;
    for (int i = 0; i < count; i++)
    {
        if (lp[i] > up_b_lp)
        {
            state |= 0x01;
        }
        else if (lp[i] < low_b_lp)
        {
            state |= 0x2;
        }

        if (dot[i] > up_b_dot)
        {
            state |= 0x4;
        }
        else if (dot[i] < low_b_dot)
        {
            state |= 0x8;
        }
    }
    return state;
}

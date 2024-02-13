#include "APDS_Data.h"
#include "Adafruit_TinyUSB.h"

void APDS_Data::set_bounds_lr(const float up_b_lr, const float low_b_lr)
{
    this->up_b_lr = up_b_lr;
    this->low_b_lr = low_b_lr;
}

void APDS_Data::calib(const bool is_initial = 1)
{
    u.calib(is_initial);
    d.calib(is_initial);
    l.calib(is_initial);
    r.calib(is_initial);
}

void APDS_Data::copy_buffer()
{
    u.count = sample_count;
    d.count = sample_count;
    l.count = sample_count;
    r.count = sample_count;
    u.copy_buffer();
    d.copy_buffer();
    l.copy_buffer();
    r.copy_buffer();
}

void APDS_Data::process()
{
    copy_buffer();

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

    for (int i = 0; i < sample_count; i++)
    {
        lr_diff[i] = l.lp[i] - r.lp[i];
    }
}

uint32_t APDS_Data::check_crossing()
{
    uint32_t state = 0;
    // bit field structure
    // | reserved | lr_state | r_state | l_state | d_state | u_state |
    // |  32 - 18 |  17 - 16 | 15 - 12 |  11 - 8 |  7 - 4  |  3 - 0  |

    state |= (u.check_crossing() & 0x0F) << 0;
    state |= (d.check_crossing() & 0x0F) << 4;
    state |= (l.check_crossing() & 0x0F) << 8;
    state |= (r.check_crossing() & 0x0F) << 12;

    for (int i = 0; i < sample_count; i++)
    {
        if (lr_diff[i] > up_b_lr)
        {
            state |= (0x01 << 16);
        }
        else if (lr_diff[i] < low_b_lr)
        {
            state |= (0x02 << 16);
        }
    }
    return state;
}

void APDS_Data::printRaw()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("ul=%d, ur=%d, uu=%d, ud=%d\n", l.raw_u8[i], r.raw_u8[i], u.raw_u8[i], d.raw_u8[i]);
    }
}

void APDS_Data::printCalib()
{
    Serial.printf("cl=%d, cr=%d, cu=%d, cd=%d\n", l.calibValue, r.calibValue, u.calibValue, d.calibValue);
}

void APDS_Data::printRaw_i16()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("il=%d, ir=%d, iu=%d, id=%d\n", l.raw_i16[i], r.raw_i16[i], u.raw_i16[i], d.raw_i16[i]);
    }
}

void APDS_Data::printLP()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("ll=%.2f, lr=%.2f, lu=%.2f, ld=%.2f\n", l.lp[i], r.lp[i], u.lp[i], d.lp[i]);
    }
}

void APDS_Data::printDot()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("dl=%.2f, dr=%.2f, du=%.2f, dd=%.2f\n", l.dot[i], r.dot[i], u.dot[i], d.dot[i]);
    }
}

void APDS_Data::printLR()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("l-r=%.2f\n", lr_diff[i]);
    }
}


APDS_Data::APDS_Data()
{
    sample_count = 0;
}

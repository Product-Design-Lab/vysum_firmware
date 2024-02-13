#include "dropDetection.h"

// Function to print a 32-bit unsigned integer in binary format
static void
printBinary(uint32_t value)
{
    // Start from the most significant bit (leftmost)
    for (int i = 31; i >= 0; i--)
    {
        // Check if the current bit is set (1) or not (0)
        printf("%d", (value >> i) & 1);

        // Add a space for better readability
        if (i % 4 == 0)
        {
            printf(" ");
        }
    }
    printf("\n");
}

void plotBinary(uint32_t value)
{
    // give each bit a name for better readability, plot each bit
    const char *names[] = {
        "U_U", "U_D", "DU_U", "DU_D",
        "D_U", "D_D", "DD_U", "DD_D",
        "L_U", "L_D", "DL_U", "DL_D",
        "R_U", "R_D", "DR_U", "DR_D",
        "LR_U", "LR_D"};
    for (int i = 0; i < 18; i++)
    {
        Serial.printf("%s:%d,", names[i], (value >> i) & 1);
    }
    Serial.printf("\n");

}

void APDS_DataChannel::calib(const bool is_initial = 1)
{
    if (is_initial)
        calibValue = raw_u8[0];

    if (calibValue < raw_u8[0])
        calibValue++;
    else if (calibValue > raw_u8[0])
        calibValue--;
}

void APDS_DataChannel::copy_buffer()
{
    memcpy(raw_u8, buffer, sizeof(uint8_t) * count);
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
    last_val = lp[count - 1];
}

void APDS_DataChannel::set_bounds_lp(const float up_b_lp, const float low_b_lp)
{
    this->up_b_lp = up_b_lp;
    this->low_b_lp = low_b_lp;
}
void APDS_DataChannel::set_bounds_dot(const float up_b_dot, const float low_b_dot)
{
    this->up_b_dot = up_b_dot;
    this->low_b_dot = low_b_dot;
}

uint8_t APDS_DataChannel::check_crossing()
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

/*
 * APDS_Data
 *
 */
APDS_Data::APDS_Data()
{
    sample_count = 0;
}

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

/*
 * APDS-9960 Gesture Sensor
 *
 */
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

    for (int i = 0; i < 128; i++)
    {
        data.sample_count = APDS.gestureAvailable(data.u.buffer, data.d.buffer, data.l.buffer, data.r.buffer);
        data.copy_buffer();
        data.printRaw();
        data.calib(false);
        delay(10);
    }

    data.set_bounds_lr(6, -4);

    data.u.set_bounds_lp(4, -4);
    data.d.set_bounds_lp(4, -4);
    data.l.set_bounds_lp(4, -4);
    data.r.set_bounds_lp(4, -4);

    data.u.set_bounds_dot(4, -4);
    data.d.set_bounds_dot(4, -4);
    data.l.set_bounds_dot(4, -4);
    data.r.set_bounds_dot(4, -4);
}

void APDS_DropSensor::update()
{
    data.sample_count = APDS.gestureAvailable(data.u.buffer, data.d.buffer, data.l.buffer, data.r.buffer);
    data.process();
    data.calib(false);

    // data.printRaw();
    // data.printCalib();
    // data.printRaw_i16();
    // data.printLP();
    // data.printDot();
    // data.printLR();

    uint32_t value = data.check_crossing();
    Serial.printf("value=%d\n", value);
    // printBinary(value);
    // plotBinary(value);
}

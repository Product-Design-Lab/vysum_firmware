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

void APDS_Data::compute_lr_diff()
{
    lr_diff[0] = lr_diff_prev;
    for (int i = 0; i <= sample_count; i++)
    {
        lr_diff[i + 1] = l.lp[i] - r.lp[i];
        // Serial.printf("lr_diff[%d]:%.2f\n", i, lr_diff[i]);
    }
    lr_diff_prev = lr_diff[sample_count];
}

APDS_Data::channel_pair_crossing_state_t APDS_Data::check_lr_crossing_state()
{
    channel_pair_crossing_state_t state = {};
    for (int i = 0; i < sample_count; i++)
    {
        state.RISE_OVER_UPPER_BOUND |= (bool)(lr_diff[i] < up_b_lr && lr_diff[i + 1] >= up_b_lr);
        state.FALL_BELOW_UPPER_BOUND |= (bool)(lr_diff[i] > up_b_lr && lr_diff[i + 1] <= up_b_lr);
        state.RISE_OVER_LOWER_BOUND |= (bool)(lr_diff[i] > low_b_lr && lr_diff[i + 1] <= low_b_lr);
        state.FALL_BELOW_LOWER_BOUND |= (bool)(lr_diff[i] < low_b_lr && lr_diff[i + 1] >= low_b_lr);
    }
    return state;
}

void APDS_Data::process()
{
    u.count = sample_count;
    d.count = sample_count;
    l.count = sample_count;
    r.count = sample_count;

    // zero the offset of the data
    u.zero_offset();
    d.zero_offset();
    l.zero_offset();
    r.zero_offset();

    // lowpass filter the data
    u.lowpass();
    d.lowpass();
    l.lowpass();
    r.lowpass();

    // find time derivative of lowpass filtered data
    u.diff();
    d.diff();
    l.diff();
    r.diff();

    // find the difference between left and right channels
    compute_lr_diff();
    channel_pair_crossing_state_t lr_state = check_lr_crossing_state();

    data_corssing_state_t temp_state = {};
    temp_state.u.state |= u.check_crossing_state().state;
    temp_state.d.state |= d.check_crossing_state().state;
    temp_state.l.state |= l.check_crossing_state().state;
    temp_state.r.state |= r.check_crossing_state().state;
    temp_state.lr.state |= lr_state.state;
    crossing_state = temp_state;

    // calibrate, bring the data back to zero
    calib(false);
}

APDS_Data::data_corssing_state_t APDS_Data::get_crossing_state()
{
    data_corssing_state_t state = crossing_state;
    crossing_state.state = 0;
    return state;
}

void APDS_Data::printRaw()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("ul:%d, ur:%d, uu:%d, ud:%d\n", l.raw_u8[i], r.raw_u8[i], u.raw_u8[i], d.raw_u8[i]);
    }
}

void APDS_Data::printCalib()
{
    Serial.printf("cl:%d, cr:%d, cu:%d, cd:%d\n", l.calibValue, r.calibValue, u.calibValue, d.calibValue);
}

void APDS_Data::printRaw_i16()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("il:%d, ir:%d, iu:%d, id:%d\n", l.raw_i16[i], r.raw_i16[i], u.raw_i16[i], d.raw_i16[i]);
    }
}

void APDS_Data::printLP()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("ll:%.2f, lr:%.2f, lu:%.2f, ld:%.2f\n", l.lp[i], r.lp[i], u.lp[i], d.lp[i]);
    }
}

void APDS_Data::printDot()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("dl:%.2f, dr:%.2f, du:%.2f, dd:%.2f\n", l.dot[i], r.dot[i], u.dot[i], d.dot[i]);
    }
}

void APDS_Data::printLR()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("l-r:%.2f\n", lr_diff[i]);
    }
}

// Function to print a 32-bit unsigned integer in binary format
void APDS_Data::printCrossingState(uint32_t val)
{
    // Start from the most significant bit (leftmost)
    for (int i = 31; i >= 0; i--)
    {
        // Check if the current bit is set (1) or not (0)
        printf("%d", (val >> i) & 1);

        // Add a space for better readability
        if (i % 4 == 0)
        {
            printf(" ");
        }
    }
    printf("\n");
}

void APDS_Data::plotCrossingState(uint32_t val)
{
    // give each bit a name for better readability, plot each bit
    const char *names[] = {
        "U_U",
        "U_D",
        "DU_U",
        "DU_D",
        "D_U",
        "D_D",
        "DD_U",
        "DD_D",
        "L_U",
        "L_D",
        "DL_U",
        "DL_D",
        "R_U",
        "R_D",
        "DR_U",
        "DR_D",
        "LR_RU",
        "LR_FU",
        "LR_RL",
        "LR_FL",
    };
    for (int i = 0; i < 20; i++)
    {
        Serial.printf("%s:%d,", names[i], (val >> i) & 1);
    }
    Serial.printf("\n");
}

APDS_Data::APDS_Data()
{
    sample_count = 0;
}

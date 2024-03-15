#include "APDS_Data.h"
#include "Adafruit_TinyUSB.h"

void APDS_Data::set_bounds_lr(const int up_b_lr, const int low_b_lr)
{
    this->up_b_lr = up_b_lr;
    this->low_b_lr = low_b_lr;
}

APDS_Data::channel_pair_crossing_state_t APDS_Data::compute_lr_diff()
{
    lr_diff[0] = lr_diff_prev;
    for (int i = 0; i <= sample_count; i++)
    {
        lr_diff[i + 1] = l.get_lp()[i] - r.get_lp()[i];
        // Serial.printf("lr_diff[%d]:%d\n", i, lr_diff[i]);
    }
    lr_diff_prev = lr_diff[sample_count];

    return check_lr_crossing_state();
}

APDS_Data::channel_pair_crossing_state_t APDS_Data::check_lr_crossing_state()
{
    channel_pair_crossing_state_t state = {};
    for (int i = 0; i < sample_count; i++)
    {
        state.RISE_OVER_UPPER_BOUND |= (bool)(lr_diff[i] < up_b_lr && lr_diff[i + 1] > up_b_lr);
        state.FALL_BELOW_UPPER_BOUND |= (bool)(lr_diff[i] > up_b_lr && lr_diff[i + 1] < up_b_lr);
        state.RISE_OVER_LOWER_BOUND |= (bool)(lr_diff[i] > low_b_lr && lr_diff[i + 1] < low_b_lr);
        state.FALL_BELOW_LOWER_BOUND |= (bool)(lr_diff[i] < low_b_lr && lr_diff[i + 1] > low_b_lr);
    }
    return state;
}

APDS_Data::data_crossing_state_t APDS_Data::process_all_channel()
{
    data_crossing_state_t temp_state = {};
    temp_state.u.state = u.process_single_channel(sample_count).state;
    temp_state.d.state = d.process_single_channel(sample_count).state;
    temp_state.l.state = l.process_single_channel(sample_count).state;
    temp_state.r.state = r.process_single_channel(sample_count).state;
    temp_state.lr.state = compute_lr_diff().state;

    crossing_state.state |= temp_state.state;
    return temp_state;
}

APDS_Data::data_crossing_state_t APDS_Data::get_crossing_state()
{
    data_crossing_state_t state = crossing_state;
    crossing_state.state = 0;
    return state;
}

void APDS_Data::printRaw()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("il:%d, ir:%d, iu:%d, id:%d\n", l.get_raw_u8()[i], r.get_raw_u8()[i], u.get_raw_u8()[i], d.get_raw_u8()[i]);
    }
}

void APDS_Data::printCalib()
{
    Serial.printf("cl:%d, cr:%d, cu:%d, cd:%d\n", l.get_calibValue(), r.get_calibValue(), u.get_calibValue(), d.get_calibValue());
}

void APDS_Data::printRaw_i32()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("il:%d, ir:%d, iu:%d, id:%d\n", l.get_raw_i32()[i], r.get_raw_i32()[i], u.get_raw_i32()[i], d.get_raw_i32()[i]);
    }
}

void APDS_Data::printLP()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("ll:%d, lr:%d, lu:%d, ld:%d\n", l.get_lp()[i], r.get_lp()[i], u.get_lp()[i], d.get_lp()[i]);
    }
}

void APDS_Data::printDot()
{
    for (int i = 0; i < sample_count; i++)
    {
        // Serial.printf("dl:%.2f, dr:%.2f, du:%.2f, dd:%.2f\n", l.dot[i], r.dot[i], u.dot[i], d.dot[i]);
        Serial.printf("dl:%d, dr:%d, du:%d, dd:%d\n", l.get_dot()[i], r.get_dot()[i], u.get_dot()[i], d.get_dot()[i]);
    }
}

void APDS_Data::printLR()
{
    for (int i = 0; i < sample_count; i++)
    {
        Serial.printf("l-r:%d\n", lr_diff[i]);
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

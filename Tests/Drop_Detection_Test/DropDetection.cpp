#include "dropDetection.h"

// Function to print a 32-bit unsigned integer in binary format
static void printBinary(uint32_t value)
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

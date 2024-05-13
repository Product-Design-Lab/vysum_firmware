#include <Arduino.h>
#include <PDL_simple_IMU.h>

void setup() 
{
    init_imu();
    Serial.begin(115200);
    set_debug_status(IMU_DEBUG_STATUS_ANGLE);
    set_vertical_threshold(10);
    set_loop_delay(100);

}
void loop() 
{
    delay(1000);
}

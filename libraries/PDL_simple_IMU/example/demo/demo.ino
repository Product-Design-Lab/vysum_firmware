#include <Arduino.h>
#include "PDL_Simple_IMU.h"

PDL_Simple_IMU imu;

void onTilted() {
    Serial.println("Device is tilted");
}

void onLevel() {
    Serial.println("Device is level");
}

void setup() {
    // Initialize Serial for debugging outputs
    Serial.begin(115200);
    while (!Serial); // Wait for serial port to connect.

    Serial.println("Initializing system...");

    // Initialize the IMU
    imu.init();

    // Set debug status to display angles
    imu.setDebugStatus(IMU_DEBUG_STATUS_ANGLE);

    // Set vertical thresholds for X and Y directions
    imu.setVerticalThresholds(-10, 10, -10, 10);

    // Set loop delay
    imu.setLoopDelay(100);

    // Set the tilt and level callbacks
    imu.setTiltedCallback(onTilted);
    imu.setLevelCallback(onLevel);

    Serial.println("IMU setup complete.");
}

void loop() {
    delay(1000);
}

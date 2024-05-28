

#include "LSM6DS3.h"
#include "Wire.h"
#include "MovingAverage.h"

#define VERTICAL_THRESHOLD_DEGREES 10

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A
MovingAverage<uint16_t, 16> filter;
uint16_t ax, ay, az;
float angle, angle_ave;
bool is_vertical;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial)
    ;
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("Device OK!");
  }
}

void loop() {
  //Accelerometer
  ax = myIMU.readRawAccelX();
  ay = myIMU.readRawAccelY();
  az = myIMU.readRawAccelZ();

  //down direction is x
  angle = atan2(sqrt(ay * ay + az * az), ax) * 180 / M_PI;
  angle_ave = filter.add(angle);

    if (angle_ave < VERTICAL_THRESHOLD_DEGREES) {
    is_vertical = true;
  }
  else {
    is_vertical = false;
  }

  // Serial.printf("ax:%6.3f, ay:%6.3f, az:%6.3f\n", ax, ay, az);
  Serial.printf("angle:%f, verticle:%d\n", angle_ave, is_vertical);
  //Thermometer
  // Serial.print("\nThermometer:\n");
  // Serial.print(" Degrees C1 = ");
  // Serial.println(myIMU.readTempC(), 4);
  // Serial.print(" Degrees F1 = ");
  // Serial.println(myIMU.readTempF(), 4);
  delay(20);
  // micros();
}

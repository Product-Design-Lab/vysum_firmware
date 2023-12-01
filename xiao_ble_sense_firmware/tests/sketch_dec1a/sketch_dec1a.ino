#include "pose_checker.hpp"
#include <Adafruit_TinyUSB.h> // for Serial
#include <Arduino.h>
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  IMU::init(0.5f);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(IMU::isVertical()){
    Serial.println("verticle");
  }else {
  {
    Serial.println("tilted");
  }
  }
}

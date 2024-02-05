/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>  // for Serial
#include "RotaryEncoder.h"

#define PIN_A D2
#define PIN_B D3
HwRotaryEncoder encoder;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // for nrf52840 with native usb

  Serial.println("Bluefruit52 HW Rotary Encoder Callback Example");
  Serial.println("----------------------------------------------\n");

  // Initialize Encoder
  encoder.begin(PIN_A, PIN_B);

  // Set callback
  encoder.setCallback(encoder_callback);

  // Start encoder
  encoder.start();
}

void loop() {
  // do nothing
}

void encoder_callback(int step) {
  // Serial.print(step);
  if (step > 0) {

    Serial.println("Right");
  } else {
    Serial.println("Left");
  }
}

#include <ArduinoBLE.h>

#define SERIAL_TIMEOUT 4000

// Timers 
unsigned long tick = 0;
unsigned long tock = 0;
unsigned long timer = 0;

// Pins
const int ledPin = 13; 

BLEService aeolusService("20B10020-E8F2-537E-4F6C-D104768A1214"); // create servic
BLEByteCharacteristic modeCharacteristic("20B10021-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // The device operating mode - e.g., tongue training, breath training exercise 1, breath training exercise 2, etc...
BLEIntCharacteristic pressureCharacteristic("20B10022-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // A normalised air pressure value between -1000 and 1000
BLEIntCharacteristic resistanceLevelCharacteristic("20B10023-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // The level of resistance support between 0-10
BLEIntCharacteristic tongueForceCharacteristic("20B10024-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // A normalised value of force applied to the tongue sensor between 0-1000
BLEByteCharacteristic button1Characteristic("20B10025-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // Primary device button
BLEByteCharacteristic button2Characteristic("20B10026-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // Secondary device button
BLEByteCharacteristic LEDCharacteristic("20B10027-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // Controls the onboard LED: 0 = off, 1 = on. May update to RGB later
BLEByteCharacteristic resetCharacteristic("20B10028-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // Resets the device and the app


const int testData[] = {0,1,2,3,4,5,6,5,4,3,2,1};   
int i = 0;
int resistanceLevel = 0;
int mode = 0;
int ledState = 0;

bool usbConnected = false;


void setup() {

  // Setup USB Serial 
  Serial.begin(115200);    // Set serial baud rate for USB 
  tick = millis();
  int tmp = 0;
  while(!Serial) { // Wait For Serial To Connect 
    // tmp = millis() - tick;
    if ((millis() - tick) > SERIAL_TIMEOUT) {
      usbConnected = false;
      break;
    }
  } 

  if (!BLE.begin()) {
    Serial.println("BLE Failed");
    if (usbConnected) {
      Serial.println("starting Bluetooth® Low Energy module failed!");
    }
    Serial.println("Entering LED Error State");
    errorLED();
  }
  BLE.setDeviceName("Aeolus_alpha_1");
  BLE.setLocalName("Aeolus_alpha_1");   // set the local name peripheral advertises
  BLE.setAdvertisedService(aeolusService); // set the UUID for the service this peripheral advertises:
  aeolusService.addCharacteristic(modeCharacteristic); 
  aeolusService.addCharacteristic(pressureCharacteristic); 
  aeolusService.addCharacteristic(resistanceLevelCharacteristic);
  aeolusService.addCharacteristic(tongueForceCharacteristic);
  aeolusService.addCharacteristic(button1Characteristic);
  aeolusService.addCharacteristic(button2Characteristic);
  aeolusService.addCharacteristic(LEDCharacteristic);
  aeolusService.addCharacteristic(resetCharacteristic);
  BLE.addService(aeolusService);  // add the service


  modeCharacteristic.setEventHandler(BLEWritten, modeCharacteristicWritten);
  resetCharacteristic.setEventHandler(BLEWritten, resetCharacteristicWritten);
  resistanceLevelCharacteristic.setEventHandler(BLEWritten, resistanceLevelCharacteristicWritten);
  LEDCharacteristic.setEventHandler(BLEWritten, LEDCharacteristicWritten);
  BLE.advertise(); // start advertising

  i = 0;
}

void loop() {

  BLE.poll();
  modeCharacteristic.writeValue(mode);  // Broadcast number of detected drops
  resistanceLevelCharacteristic.writeValue(resistanceLevel);
  pressureCharacteristic.writeValue((int) (1000*sin(i*0.01*PI)));
  tongueForceCharacteristic.writeValue((int) (500*sin(i*0.01*PI))+500);
  button1Characteristic.writeValue(0);
  button2Characteristic.writeValue(1);

  delay(100);
  i++;
}


void modeCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {

  Serial.print("Mode: ");

  if (characteristic.value()) {
    if (usbConnected) {
      Serial.println("Reset");
    }
    i = 0;
  }

  mode = (int) characteristic.value();
  Serial.println(mode);

  for (int j=0; j < mode; j++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }

}

void resetCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  if (usbConnected) {
    Serial.print("Characteristic event, written: ");
  }
  if (!characteristic.value()) {
    if (usbConnected) {
      Serial.println("Reset");
    }
    i = 0;
  }

  for (int j=0; j < 10; j++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }

}

void resistanceLevelCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  if (usbConnected) {
    Serial.print("Resistance Level: ");
  }
  resistanceLevel = (int) characteristic.value();
  Serial.println(resistanceLevel);

  for (int j=0; j < 3; j++) {
    digitalWrite(ledPin, HIGH);
    delay(200);
    digitalWrite(ledPin, LOW);
    delay(200);
  }

}

void LEDCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {

  Serial.println("LED Characteristic Written");

  ledState = (int) characteristic.value();

  if (ledState == 0) {
    digitalWrite(ledPin, LOW);
  }
  else if (ledState == 1) {
    digitalWrite(ledPin, HIGH);
  }

}


void errorLED() {

  if (usbConnected) {
    Serial.println("ERROR");
  }

  while(1) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

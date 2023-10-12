#include <ArduinoBLE.h>

#define SERIAL_TIMEOUT 4000

// Timers
unsigned long tick = 0;
unsigned long tock = 0;
unsigned long timer = 0;

// Pins
const int ledPin = 13;

BLEService ocumateService("20B10020-E8F2-537E-4F6C-D104768A1214");                                            // create service
BLEByteCharacteristic medicationCharacteristic("20B10021-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);   // The selected medication
BLEIntCharacteristic dropCharacteristic("20B10022-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);          // The number of drops recorded by the device
BLEIntCharacteristic batteryLevelCharacteristic("20B10023-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);  // The battery level of the device from 0-100
BLEIntCharacteristic dropsLeftCharacteristic("20B10024-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);     // The theoretical number of drops left in the bottle
BLEByteCharacteristic resetCharacteristic("20B10028-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);        // Resets the device and the app


const int testData[] = { 0, 1, 2, 3, 4, 5, 6, 5, 4, 3, 2, 1 };
int i = 0;
int batteryLevel = 100;
int medication = 0;
int ledState = 0;
int drops = 0;
int totalDrops = 100;

bool usbConnected = false;


void setup() {

  // Setup USB Serial
  Serial.begin(115200);  // Set serial baud rate for USB
  tick = millis();
  int tmp = 0;
  while (!Serial) {  // Wait For Serial To Connect
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
  BLE.setDeviceName("Ocumate_POC_1");
  BLE.setLocalName("Ocumate_POC_1");         // set the local name peripheral advertises
  BLE.setAdvertisedService(ocumateService);  // set the UUID for the service this peripheral advertises:
  ocumateService.addCharacteristic(medicationCharacteristic);
  ocumateService.addCharacteristic(dropCharacteristic);
  ocumateService.addCharacteristic(batteryLevelCharacteristic);
  ocumateService.addCharacteristic(dropsLeftCharacteristic);
  ocumateService.addCharacteristic(resetCharacteristic);
  BLE.addService(ocumateService);  // add the service


  medicationCharacteristic.setEventHandler(BLEWritten, medicationCharacteristicWritten);
  resetCharacteristic.setEventHandler(BLEWritten, resetCharacteristicWritten);


  BLE.advertise();  // start advertising

  i = 0;

  Serial.println("Ocumate Ready.");
}

void loop() {

  if (i % 10 == 0) {
    drops++;
  }

  if (drops >= 99) {
    drops = 0;
    i = 0;
  }

  if (i % 15 == 0) {
    batteryLevel--;
  }
  if (batteryLevel < 0) {
    batteryLevel = 100;
  }

  BLE.poll();
  medicationCharacteristic.writeValue(medication);  // Broadcast number of detected drops
  batteryLevelCharacteristic.writeValue(batteryLevel);
  dropCharacteristic.writeValue(drops);
  dropsLeftCharacteristic.writeValue(totalDrops - drops);


  delay(100);
  i++;
}


void medicationCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {

  Serial.print("medication: ");


  // medication = characteristic.value();
  // Serial.println(medication);
  Serial.println(characteristic.value()[0]);

  // for (int j=0; j < medication; j++) {
  //   digitalWrite(ledPin, HIGH);
  //   delay(100);
  //   digitalWrite(ledPin, LOW);
  //   delay(100);
  // }
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

  for (int j = 0; j < 10; j++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
}



void errorLED() {

  if (usbConnected) {
    Serial.println("ERROR");
  }

  while (1) {
    digitalWrite(ledPin, HIGH);
    delay(500);
    digitalWrite(ledPin, LOW);
    delay(500);
  }
}

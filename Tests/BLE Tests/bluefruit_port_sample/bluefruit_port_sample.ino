#include <bluefruit.h>

#define SERIAL_TIMEOUT 4000

// Timers
unsigned long tick = 0;
unsigned long tock = 0;
unsigned long timer = 0;

// Pins
const static int ledPin = 13;

// BLE Services and Characteristics UUIDs
const char SERVICE_UUID[] = "20B10020-E8F2-537E-4F6C-D104768A1214";
const char MEDICATION_UUID[] = "20B10021-E8F2-537E-4F6C-D104768A1214";
const char DROP_UUID[] = "20B10022-E8F2-537E-4F6C-D104768A1214";
const char BATTERY_UUID[] = "20B10023-E8F2-537E-4F6C-D104768A1214";
const char DROPS_LEFT_UUID[] = "20B10024-E8F2-537E-4F6C-D104768A1214";
const char RESET_UUID[] = "20B10028-E8F2-537E-4F6C-D104768A1214";

BLEService ocumateService(SERVICE_UUID);
BLECharacteristic medicationCharacteristic(MEDICATION_UUID, BLENotify | BLEWrite);
BLECharacteristic dropCharacteristic(DROP_UUID, BLENotify | BLEWrite);
BLECharacteristic batteryLevelCharacteristic(BATTERY_UUID, BLENotify | BLEWrite);
BLECharacteristic dropsLeftCharacteristic(DROPS_LEFT_UUID, BLENotify | BLEWrite);
BLECharacteristic resetCharacteristic(RESET_UUID, BLENotify | BLEWrite);

int batteryLevel = 100;
int medication = 0;
int ledState = 0;
int drops = 0;
int totalDrops = 100;
int i = 0;

void setup()
{
  // Setup USB Serial
  Serial.begin(115200);
  while (!Serial && millis() < SERIAL_TIMEOUT)
    ;

  // Setup Bluefruit
  Bluefruit.begin();
  Bluefruit.setTxPower(4); // Check the max power available
  Bluefruit.setName("Ocumate_POC_1");

  // Set up the BLE service
  ocumateService.begin();

  // Set up BLE characteristics with user-friendly names
  medicationCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  medicationCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  medicationCharacteristic.setFixedLen(1);
  medicationCharacteristic.setWriteCallback(medicationCharacteristicWritten);
  medicationCharacteristic.setUserDescriptor("Medication");
  medicationCharacteristic.begin();

  dropCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  dropCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  dropCharacteristic.setFixedLen(2);
  dropCharacteristic.setUserDescriptor("Drops Recorded");
  dropCharacteristic.begin();

  batteryLevelCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  batteryLevelCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  batteryLevelCharacteristic.setFixedLen(2);
  batteryLevelCharacteristic.setUserDescriptor("Battery Level");
  batteryLevelCharacteristic.begin();

  dropsLeftCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  dropsLeftCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  dropsLeftCharacteristic.setFixedLen(2);
  dropsLeftCharacteristic.setUserDescriptor("Drops Left");
  dropsLeftCharacteristic.begin();

  resetCharacteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  resetCharacteristic.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  resetCharacteristic.setFixedLen(1);
  resetCharacteristic.setWriteCallback(resetCharacteristicWritten);
  resetCharacteristic.setUserDescriptor("Reset");
  resetCharacteristic.begin();

  // Start advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(ocumateService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.start();

  Serial.println("Ocumate Ready.");
}

void loop()
{

  // Update drop count and battery level for demonstration
  if (i % 10 == 0)
  {
    drops++;
  }

  if (drops >= 99)
  {
    drops = 0;
    i = 0;
  }

  if (i % 15 == 0)
  {
    batteryLevel--;
  }

  if (batteryLevel < 0)
  {
    batteryLevel = 100;
  }

  // Write values to characteristics
  medicationCharacteristic.write(&medication, sizeof(medication));
  dropCharacteristic.write(&drops, sizeof(drops));
  batteryLevelCharacteristic.write(&batteryLevel, sizeof(batteryLevel));
  dropsLeftCharacteristic.write(&totalDrops - drops, sizeof(totalDrops - drops));

  delay(100);
  i++;

  Serial.println(i);
}

void medicationCharacteristicWritten(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
  medication = data[0];
  Serial.printf("Medication: %d\n", medication);
}

void resetCharacteristicWritten(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
  int value = data[0];
  Serial.printf("Reset: %d\n", value);
  i = 0;
}

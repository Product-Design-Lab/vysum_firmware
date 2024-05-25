#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "MyClass.h"

void setup() {
    // Initialize USB serial communication
    Serial.begin(115200);

    // Wait for the USB serial connection to be established
    while (!Serial) {
        delay(10);
    }

    Serial.println("Starting...");

    MyClass* instance1 = MyClass::createInstance();
    MyClass* instance2 = MyClass::createInstance();
    MyClass* instance3 = MyClass::createInstance();
    MyClass* instance4 = MyClass::createInstance(); // This will be nullptr because MAX_INSTANCES is 3

    if (instance4 == nullptr) {
        Serial.println("Cannot create more than 3 instances.");
    }

    // Initialize instances to start tasks
    if (instance1) instance1->init();
    if (instance2) instance2->init();
    if (instance3) instance3->init();
}

void loop() {
    // Main loop code
    Serial.println("MAIN");
    delay(1000);
}

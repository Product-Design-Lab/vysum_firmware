#include <Arduino.h>
#include <PDL_Async_Button_Group.h>
#include <Adafruit_TinyUSB.h>

// Create an instance of AsyncButtonGroup
AsyncButtonGroup button;

// Callback function for short press
void onShortPress() {
    Serial.println("Event: Short Press detected");
}

// Callback function for long press
void onLongPress() {
    Serial.println("Event: Long Press detected");
}

void setup() {
    // Initialize Serial Communication
    Serial.begin(115200);
    while (!Serial);

    // Configure the button
    button.setPin(0, HIGH);
    button.setDebounceTime(50);
    button.setLongPressTime(1000);
    button.setShortPressCallback(onShortPress);
    button.setLongPressCallback(onLongPress);
    button.init();

    Serial.println("Button initialized.");
}

void loop() {
    // The button press events are handled by the callbacks, nothing to do here
    delay(1000);
    Serial.println(".");
}

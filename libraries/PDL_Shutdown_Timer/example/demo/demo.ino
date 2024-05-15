#include "PDL_Shutdown_Timer.h"
#include "Adafruit_TinyUSB.h"

#define LED_PIN LED_GREEN // Define the pin where the LED is connected

// Create an instance of PDL_Shutdown_Timer
PDL_Shutdown_Timer shutdownTimer;

void setup() {
    // Initialize Serial for debugging outputs
    Serial.begin(115200);
    while (!Serial) ; // Wait for serial port to connect.

    Serial.println("Initializing system...");

    // Set up the LED pin as an output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Set debug level
    shutdownTimer.setDebug(PDL_Shutdown_Timer::DEBUG_ON);

    // Initialize the shutdown timer module
    shutdownTimer.init();

    // Set the timer's enable pin to the LED pin
    shutdownTimer.setEnPin(LED_PIN);

    // Set the shutdown duration (e.g., 5 seconds)
    shutdownTimer.setShutdownTimeSec(5);

    // Start the timer
    if (shutdownTimer.start() != pdPASS) {
        Serial.println("Failed to start the shutdown timer");
    }

    // Indicate the system is alive
    digitalWrite(LED_PIN, LOW); // Turn on the LED
}

void loop() {
    // Toggle the LED to show the system is running
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
}

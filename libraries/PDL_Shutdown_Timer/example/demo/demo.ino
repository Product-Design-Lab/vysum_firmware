#include "PDL_Shutdown_Timer.h"
#include "Adafruit_TinyUSB.h"

#define LED_PIN LED_GREEN // Define the pin where the LED is connected

void setup()
{
    // Initialize Serial for debugging outputs
    Serial.begin(115200);
    while (!Serial)
        ; // Wait for serial port to connect.

    Serial.println("Initializing system...");

    // Set up the LED pin as an output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    PDL_Shutdown_Timer_set_debug(PDL_Shutdown_Timer_Debug_ON);

    // Initialize the shutdown timer module
    PDL_Shutdown_Timer_init();

    // Set the timer's enable pin to the LED pin
    // PDL_Shutdown_Timer_set_en_pin(2);

    // Set the shutdown duration (e.g., 5 seconds)
    PDL_Shutdown_Timer_set_shutdown_time_sec(5);

    // Start the timer
    PDL_Shutdown_Timer_start();

    // Indicate the system is alive
    digitalWrite(LED_PIN, LOW); // Turn on the LED
}

void loop()
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
}

#include <Arduino.h>
#include <PDL_Async_Button_Group.h>
#include <Adafruit_TinyUSB.h>

// Create an instance of AsyncButtonGroup
AsyncButtonGroup button;

// Variables to keep track of the previous state
uint8_t previousState = AsyncButtonGroup::BUTTON_IDLE;
int previousShortPressCount = 0;
int previousLongPressCount = 0;

void setup()
{
    // Initialize Serial Communication
    Serial.begin(115200);
    while (!Serial)
        ;

    // Configure the button
    button.setPin(0, HIGH);
    button.setDebounceTime(50);
    button.setLongPressTime(1000);
    button.init();

    Serial.println("Button initialized.");
}

void loop()
{
    int short_press_count;
    int long_press_count;
    uint8_t state = button.getState(&short_press_count, &long_press_count);

    // Print state and counts only if they change
    if (state != previousState || short_press_count != previousShortPressCount || long_press_count != previousLongPressCount)
    {
        switch (state)
        {
        case AsyncButtonGroup::BUTTON_IDLE:
            Serial.print("State: IDLE");
            break;
        case AsyncButtonGroup::BUTTON_DEBOUNCE:
            Serial.print("State: DEBOUNCE");
            break;
        case AsyncButtonGroup::BUTTON_SHORT_PRESS:
            Serial.print("State: SHORT PRESS");
            break;
        case AsyncButtonGroup::BUTTON_LONG_PRESS:
            Serial.print("State: LONG PRESS");
            break;
        default:
            Serial.print("State: UNKNOWN");
            break;
        }

        Serial.printf(", Short: %d, Long: %d\n", short_press_count, long_press_count);

        // Update the previous state and counts
        previousState = state;
        previousShortPressCount = short_press_count;
        previousLongPressCount = long_press_count;
    }

    // Small delay to avoid flooding the Serial output
    delay(100);
}

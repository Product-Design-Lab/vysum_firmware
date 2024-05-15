#include <Arduino.h>
#include <PDL_Async_Button_Group.h>
#include <Adafruit_TinyUSB.h>

// Create instances of AsyncButtonGroup
AsyncButtonGroup button1;
AsyncButtonGroup button2;

// Variables to keep track of the previous states
uint8_t previousState1 = AsyncButtonGroup::BUTTON_IDLE;
int previousShortPressCount1 = 0;
int previousLongPressCount1 = 0;

uint8_t previousState2 = AsyncButtonGroup::BUTTON_IDLE;
int previousShortPressCount2 = 0;
int previousLongPressCount2 = 0;

void initializeButton(AsyncButtonGroup &button, uint8_t pin, uint32_t debounceTime, uint32_t longPressTime)
{
    button.setPin(pin, HIGH);
    button.setDebounceTime(debounceTime);
    button.setLongPressTime(longPressTime);
    button.init();
}

void setup()
{
    // Initialize Serial Communication
    Serial.begin(115200);
    while (!Serial)
        ;

    // Initialize buttons
    initializeButton(button1, 0, 50, 1000);
    initializeButton(button2, 1, 50, 1000);

    Serial.println("Buttons initialized.");
}

void printState(AsyncButtonGroup &button, uint8_t &previousState, int &previousShortPressCount, int &previousLongPressCount)
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
}

void loop()
{
    printState(button1, previousState1, previousShortPressCount1, previousLongPressCount1);
    printState(button2, previousState2, previousShortPressCount2, previousLongPressCount2);
    delay(100);
}

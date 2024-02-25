#include <Arduino.h>
#include <PDL_Async_Button_Group.h>
#include <Adafruit_TinyUSB.h>

AsyncButtonGroup button;
AsyncButtonGroup button2;

void setup() 
{
    Serial.begin(115200);
    while(!Serial);

    button.setPin(0, HIGH);
    button.setDebounceTime(50);
    button.setLongPressTime(1000);
    button.init();

    button2.setPin(1, HIGH);
    button2.setDebounceTime(50);
    button2.setLongPressTime(1000);
    button2.init();
}

void printState(AsyncButtonGroup &button)
{
    int short_press_count;
    int long_press_count;
    uint8_t state = button.getState(&short_press_count, &long_press_count);

    switch(state)
    {
        case AsyncButtonGroup::BUTTON_IDLE:
            Serial.print("state: IDLE");
            break;
        case AsyncButtonGroup::BUTTON_DEBOUCE:
            Serial.print("state: DEBOUNCE");
            break;
        case AsyncButtonGroup::BUTTON_SHORT_PRESS:
            Serial.print("state: SHORT PRESS");
            break;
        case AsyncButtonGroup::BUTTON_LONG_PRESS:
            Serial.print("state: LONG PRESS");
            break;
        default:
            Serial.print("state UNKNOWN");
            break;
    }
    
    Serial.printf(", Short: %d, Long: %d\n", short_press_count, long_press_count);
}
void loop() 
{
    printState(button);
    printState(button2);
    delay(250);
}

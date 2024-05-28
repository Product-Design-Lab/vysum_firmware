#include <Adafruit_TinyUSB.h>
#include "state_machine.h"

// Custom action functions
void CustomInitAction()
{
    Serial.println("Custom Init Action");
}

void CustomGrippingAction()
{
    Serial.println("Custom Gripping Action");
}

void CustomIdleAction()
{
    Serial.println("Custom Idle Action");
}

void CustomDispensingAction()
{
    Serial.println("Custom Dispensing Action");
}

void CustomRetractingAction()
{
    Serial.println("Custom Retracting Action");
}

void CustomReleasingAction()
{
    Serial.println("Custom Releasing Action");
}

void CustomPauseAction()
{
    Serial.println("Custom Pause Action");
}

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
    }

    // Assign custom action functions
    SetInitAction(CustomInitAction);
    SetGrippingAction(CustomGrippingAction);
    SetIdleAction(CustomIdleAction);
    SetDispensingAction(CustomDispensingAction);
    SetRetractingAction(CustomRetractingAction);
    SetReleasingAction(CustomReleasingAction);
    SetPauseAction(CustomPauseAction);

    // Initialize the state machine
    StateMachine_Init();
    Serial.println("State machine initialized. Use 'sp', 'lp', 'dr', 'dd', 'ms', 'dt', 'dv', 'to' to trigger events.");
}

State_t currentState = STATE_INIT;

void loop()
{
    delay(200);


    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input == "sp")
        {
            HandleEvent(EVENT_SHORT_PRESS);
        }
        else if (input == "lp")
        {
            HandleEvent(EVENT_LONG_PRESS);
        }
        else if (input == "dr")
        {
            HandleEvent(EVENT_DISTANCE_REACHED);
        }
        else if (input == "dd")
        {
            HandleEvent(EVENT_DROP_DETECTED);
        }
        else if (input == "ms")
        {
            HandleEvent(EVENT_MOTOR_STALL);
        }
        else if (input == "dt")
        {
            HandleEvent(EVENT_DEVICE_TILTED);
        }
        else if (input == "dv")
        {
            HandleEvent(EVENT_DEVICE_VERTICAL);
        }
        else if (input == "to")
        {
            HandleEvent(EVENT_TIMEOUT);
        }
        else
        {
            Serial.println("Unknown command. Use 'sp', 'lp', 'dr', 'dd', 'ms', 'dt', 'dv', 'to' to trigger events.");
        }
    }
}

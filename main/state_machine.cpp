#include "state_machine.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>

// Current state of the state machine
static State_t currentState = STATE_INIT;

// State transition table
static const StateTransition_t stateTransitions[] = {
    {STATE_INIT, EVENT_SHORT_PRESS, STATE_GRIPPING},
    {STATE_INIT, EVENT_LONG_PRESS, STATE_RELEASING},
    {STATE_GRIPPING, EVENT_MOTOR_STALL, STATE_IDLE},
    {STATE_IDLE, EVENT_LONG_PRESS, STATE_RELEASING},
    {STATE_IDLE, EVENT_SHORT_PRESS, STATE_DISPENSING},
    {STATE_DISPENSING, EVENT_DISTANCE_REACHED, STATE_RETRACTING},
    {STATE_DISPENSING, EVENT_DROP_DETECTED, STATE_RETRACTING},
    {STATE_DISPENSING, EVENT_MOTOR_STALL, STATE_RETRACTING},
    {STATE_DISPENSING, EVENT_DEVICE_TILTED, STATE_PAUSE},
    {STATE_RETRACTING, EVENT_DISTANCE_REACHED, STATE_IDLE},
    // {STATE_RELEASING, EVENT_DISTANCE_REACHED, STATE_INIT},
    {STATE_RELEASING, EVENT_MOTOR_STALL, STATE_INIT},
    {STATE_PAUSE, EVENT_DEVICE_VERTICAL, STATE_DISPENSING},
    {STATE_PAUSE, EVENT_TIMEOUT, STATE_RETRACTING}};

// Number of transitions
#define NUM_TRANSITIONS (sizeof(stateTransitions) / sizeof(stateTransitions[0]))

// Function pointers for state-specific actions
static StateAction_t stateActions[STATE_MAX_STATES] = {0};

#define QUEUE_LENGTH 10

QueueHandle_t xQueue = NULL;
TaskHandle_t stateMachineTaskHandle = NULL;

static void printState(State_t state)
{
    switch (state)
    {
    case STATE_INIT:
        printf("Current state: INIT\n");
        break;
    case STATE_GRIPPING:
        printf("Current state: GRIPPING\n");
        break;
    case STATE_IDLE:
        printf("Current state: IDLE\n");
        break;
    case STATE_DISPENSING:
        printf("Current state: DISPENSING\n");
        break;
    case STATE_RETRACTING:
        printf("Current state: RETRACTING\n");
        break;
    case STATE_RELEASING:
        printf("Current state: RELEASING\n");
        break;
    case STATE_PAUSE:
        printf("Current state: PAUSE\n");
        break;
    }
}

static void printEvent(Event_t event)
{
    switch (event)
    {
    case EVENT_SHORT_PRESS:
        printf("Event: SHORT_PRESS\n");
        break;
    case EVENT_LONG_PRESS:
        printf("Event: LONG_PRESS\n");
        break;
    case EVENT_DISTANCE_REACHED:
        printf("Event: DISTANCE_REACHED\n");
        break;
    case EVENT_DROP_DETECTED:
        printf("Event: DROP_DETECTED\n");
        break;
    case EVENT_MOTOR_STALL:
        printf("Event: MOTOR_STALL\n");
        break;
    case EVENT_DEVICE_TILTED:
        printf("Event: DEVICE_TILTED\n");
        break;
    case EVENT_DEVICE_VERTICAL:
        printf("Event: DEVICE_VERTICAL\n");
        break;
    case EVENT_TIMEOUT:
        printf("Event: TIMEOUT\n");
        break;
    }
}

// Function to find the next state based on the current state and event
static State_t GetNextState(State_t currentState, Event_t event)
{
    for (int i = 0; i < NUM_TRANSITIONS; i++)
    {
        if (stateTransitions[i].currentState == currentState && stateTransitions[i].event == event)
        {
            // print current state, event and next state
            printState(currentState);
            printEvent(event);
            printState(stateTransitions[i].nextState);

            return stateTransitions[i].nextState;
        }
    }
    return currentState; // No transition found, remain in the current state
}

// Function to handle state transitions
static void _HandleEvent(Event_t event)
{
    State_t nextState = GetNextState(currentState, event);
    if (nextState != currentState)
    {
        currentState = nextState;
        if (stateActions[currentState])
        {
            stateActions[currentState]();
        }
    }
}

void stateMachineTask(void *pvParameters)
{
    Event_t event;
    while (1)
    {
        if (xQueueReceive(xQueue, &event, portMAX_DELAY))
        {
            _HandleEvent(event);
        }
    }
}

/**
 * public functions
 */

// Function to assign action functions
void SetInitAction(StateAction_t action)
{
    stateActions[STATE_INIT] = action;
}

void SetGrippingAction(StateAction_t action)
{
    stateActions[STATE_GRIPPING] = action;
}

void SetIdleAction(StateAction_t action)
{
    stateActions[STATE_IDLE] = action;
}

void SetDispensingAction(StateAction_t action)
{
    stateActions[STATE_DISPENSING] = action;
}

void SetRetractingAction(StateAction_t action)
{
    stateActions[STATE_RETRACTING] = action;
}

void SetReleasingAction(StateAction_t action)
{
    stateActions[STATE_RELEASING] = action;
}

void SetPauseAction(StateAction_t action)
{
    stateActions[STATE_PAUSE] = action;
}

bool HandleEvent(Event_t event)
{
    if(xQueue == NULL)
    {
        return pdFALSE;
    }

    if(__get_IPSR())//check if we are in an ISR. This MACRO is ARM specific, doesn't work on other architectures such as RISC-V
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(xQueue, &event, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return pdTRUE;
    }
    else
    {
       return xQueueSend(xQueue, &event, 0);
    }
}

// Function to get the current state
State_t getCurrentState(void)
{
    return currentState;
}

// Function to initialize the state machine
void StateMachine_Init(void)
{
    xQueue = xQueueCreate(QUEUE_LENGTH, sizeof(Event_t));
    if (xQueue == NULL)
    {
        printf("Queue creation failed\n");
    }

    xTaskCreate(stateMachineTask, "stateMachineTask", 1024, NULL, 1, &stateMachineTaskHandle);
    if (stateMachineTaskHandle == NULL)
    {
        printf("Task creation failed\n");
    }

}

#pragma once

#include <stdint.h>

// Enumeration of possible states
typedef enum
{
    STATE_INIT,
    STATE_GRIPPING,
    STATE_IDLE,
    STATE_DISPENSING,
    STATE_RETRACTING,
    STATE_RELEASING,
    STATE_PAUSE,
    STATE_MAX_STATES // For boundary checking
} State_t;

// Enumeration of possible events
typedef enum
{
    EVENT_SHORT_PRESS,
    EVENT_LONG_PRESS,
    EVENT_DISTANCE_REACHED,
    EVENT_DROP_DETECTED,
    EVENT_MOTOR_STALL,
    EVENT_DEVICE_TILTED,
    EVENT_DEVICE_VERTICAL,
    EVENT_TIMEOUT,
    EVENT_MAX_EVENTS // For boundary checking
} Event_t;

// Definition of a state transition
typedef struct
{
    State_t currentState;
    Event_t event;
    State_t nextState;
} StateTransition_t;

// Function pointer type for state-specific actions
typedef void (*StateAction_t)(void);

// Function prototypes for assigning actions
void SetInitAction(StateAction_t action);
void SetGrippingAction(StateAction_t action);
void SetIdleAction(StateAction_t action);
void SetDispensingAction(StateAction_t action);
void SetRetractingAction(StateAction_t action);
void SetReleasingAction(StateAction_t action);
void SetPauseAction(StateAction_t action);

// Function prototype for handling state transitions

bool HandleEvent(Event_t event);
State_t getCurrentState(void);

// Function prototype for initializing the state machine
void StateMachine_Init(void);


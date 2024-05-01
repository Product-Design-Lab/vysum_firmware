// PDL_Shutdown_Timer.h

#pragma once

#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

enum PDL_Shutdown_Timer_Debug
{
    PDL_Shutdown_Timer_Debug_OFF = 0,
    PDL_Shutdown_Timer_Debug_ON = 1,
    PDL_Shutdown_Timer_Debug_MAX,
};

// Initializes the timer module
void PDL_Shutdown_Timer_init();
// Starts the shutdown timer
int PDL_Shutdown_Timer_start();
// Stops the shutdown timer
int PDL_Shutdown_Timer_stop();
// Resets the shutdown timer
void PDL_Shutdown_Timer_reset();
// De-initializes the timer module, freeing resources
void PDL_Shutdown_Timer_deinit();

// Sets the enable pin for the timer
void PDL_Shutdown_Timer_set_en_pin(uint8_t en_pin);
// Sets the shutdown duration in seconds
void PDL_Shutdown_Timer_set_shutdown_time_sec(uint32_t shutdown_time_sec);
// Sets the GPIO state to use for shutdown
void PDL_Shutdown_Timer_set_enable_gpio_state(bool enable_gpio_state);

// Sets the debug level for the timer module
void PDL_Shutdown_Timer_set_debug(uint8_t debug);

// Shutdown the system
void PDL_Shutdown_Timer_system_shutdown();

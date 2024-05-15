#pragma once

#include <Arduino.h>
#include <stdint.h>

class PDL_Shutdown_Timer {
public:
    enum DebugLevel {
        DEBUG_OFF = 0,
        DEBUG_ON = 1,
        DEBUG_MAX,
    };

    PDL_Shutdown_Timer();

    // Initializes the timer module
    void init();
    // Starts the shutdown timer
    int start();
    // Stops the shutdown timer
    int stop();
    // Resets the shutdown timer
    void reset();
    // De-initializes the timer module, freeing resources
    void deinit();

    // Sets the enable pin for the timer
    void setEnPin(uint8_t en_pin);
    // Sets the shutdown duration in seconds
    void setShutdownTimeSec(uint32_t shutdown_time_sec);
    // Sets the GPIO state to use for shutdown
    void setEnableGpioState(bool enable_gpio_state);

    // Sets the debug level for the timer module
    void setDebug(DebugLevel debug);

    // Shuts down the system
    void systemShutdown();

    // Puts the system to sleep
    void systemSleep();

private:
    // Private Variables
    TimerHandle_t xTimer;
    uint8_t en_pin;
    uint32_t shutdown_time_sec;
    bool enable_gpio_state;
    DebugLevel debug;

    // Private Function Prototypes
    static void vTimerCallback(TimerHandle_t xTimer);
    void initializeTimer();
    bool timerIsInitialized();
    void shutdown();
    void keepSystemAlive();

    // Debugging
    void debugPrintf(const char *format, ...);
    void debugInit();
};

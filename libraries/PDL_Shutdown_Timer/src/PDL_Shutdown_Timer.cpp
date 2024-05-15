#include "PDL_Shutdown_Timer.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <bluefruit.h>
#include <Arduino.h>

#define ENABLE_DEBUG

// Default Configuration Constants
#define TIMER_ID 0
#define DEFAULT_EN_PIN 6
#define DEFAULT_SHUTDOWN_TIME_SEC 10
#define DEFAULT_ENABLE_GPIO_STATE 1

PDL_Shutdown_Timer::PDL_Shutdown_Timer()
    : xTimer(NULL), en_pin(DEFAULT_EN_PIN), shutdown_time_sec(DEFAULT_SHUTDOWN_TIME_SEC), enable_gpio_state(DEFAULT_ENABLE_GPIO_STATE), debug(DEBUG_OFF) {}

void PDL_Shutdown_Timer::init() {
    debugInit();
    Bluefruit.begin();
    pinMode(en_pin, OUTPUT);
    keepSystemAlive();
    if (xTimer == NULL) {
        initializeTimer();
    }
}

int PDL_Shutdown_Timer::start() {
    if (!timerIsInitialized()) return pdFAIL;
    int ret = xTimerStart(xTimer, 0);
    debugPrintf("Timer %s\n", ret == pdPASS ? "started" : "failed to start");
    return ret;
}

int PDL_Shutdown_Timer::stop() {
    if (!timerIsInitialized()) return pdFAIL;
    int ret = xTimerStop(xTimer, 0);
    debugPrintf("Timer %s\n", ret == pdPASS ? "stopped" : "failed to stop");
    return ret;
}

void PDL_Shutdown_Timer::reset() {
    if (!timerIsInitialized()) return;
    int ret = xTimerReset(xTimer, 0);
    debugPrintf("Timer %s\n", ret == pdPASS ? "reset" : "failed to reset");
}

void PDL_Shutdown_Timer::deinit() {
    if (!timerIsInitialized()) return;
    if (xTimerDelete(xTimer, 0) == pdPASS) {
        xTimer = NULL;
        debugPrintf("Timer deleted\n");
    } else {
        debugPrintf("Failed to delete timer\n");
    }
}

void PDL_Shutdown_Timer::setEnPin(uint8_t en_pin_param) {
    en_pin = en_pin_param;
}

void PDL_Shutdown_Timer::setShutdownTimeSec(uint32_t shutdown_time_sec_param) {
    if (shutdown_time_sec_param == 0) {
        debugPrintf("Invalid shutdown time (0 seconds) - Timer period not updated\n");
        return;
    }
    shutdown_time_sec = shutdown_time_sec_param;
    if (!timerIsInitialized()) return;
    if (xTimerChangePeriod(xTimer, pdMS_TO_TICKS(shutdown_time_sec * 1000), 0) == pdPASS) {
        debugPrintf("Timer period changed to %d seconds\n", shutdown_time_sec);
    } else {
        debugPrintf("Failed to change timer period\n");
    }
}

void PDL_Shutdown_Timer::setEnableGpioState(bool enable_gpio_state_param) {
    enable_gpio_state = enable_gpio_state_param;
}

void PDL_Shutdown_Timer::setDebug(DebugLevel debug_param) {
    if (debug_param >= DEBUG_MAX) {
        debugPrintf("Invalid debug level\n");
        return;
    }
    debug = debug_param;
}

void PDL_Shutdown_Timer::systemShutdown() {
    shutdown();
}

void PDL_Shutdown_Timer::systemSleep() {
    debugPrintf("System sleep\n");
    delay(1000);
    sd_power_system_off();
}

// Private Function Implementations

#ifdef ENABLE_DEBUG
void PDL_Shutdown_Timer::debugInit() {
    Serial.begin(115200);
    while (!Serial) delay(10);
}

void PDL_Shutdown_Timer::debugPrintf(const char *format, ...) {
    if (debug != DEBUG_ON) return;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}
#endif

void PDL_Shutdown_Timer::initializeTimer() {
    if (shutdown_time_sec == 0) {
        debugPrintf("Invalid shutdown time (0 seconds) - Timer not created\n");
        return;
    }
    xTimer = xTimerCreate("Shutdown Timer", pdMS_TO_TICKS(shutdown_time_sec * 1000), pdFALSE, this, vTimerCallback);
    debugPrintf("Timer %s\n", xTimer != NULL ? "created" : "failed to create");
}

bool PDL_Shutdown_Timer::timerIsInitialized() {
    return xTimer != NULL;
}

void PDL_Shutdown_Timer::shutdown() {
    debugPrintf("System shutdown\n");
    digitalWrite(en_pin, !enable_gpio_state);
}

void PDL_Shutdown_Timer::keepSystemAlive() {
    digitalWrite(en_pin, enable_gpio_state);
}

void PDL_Shutdown_Timer::vTimerCallback(TimerHandle_t xTimer) {
    PDL_Shutdown_Timer *timer = static_cast<PDL_Shutdown_Timer*>(pvTimerGetTimerID(xTimer));
    if (timer) {
        timer->shutdown();
        // Power enable pin will not cut off power for seeed if USB is connected. In this case, shutdown seeed via software API
        timer->systemSleep();
        xTimerDelete(xTimer, 0);
        timer->xTimer = NULL;
    }
}

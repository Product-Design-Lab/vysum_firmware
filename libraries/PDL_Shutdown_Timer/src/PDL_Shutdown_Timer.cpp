#include "PDL_Shutdown_Timer.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <Arduino.h>

#define ENABLE_DEBUG

// Debugging Macros and Functions
#ifdef ENABLE_DEBUG
#include "Adafruit_TinyUSB.h"
static void debugPrintf(const char *format, ...);
static void debug_init();
#define DEBUG_INIT() debug_init()
#else
#define DEBUG_INIT() ((void)0)
#define debugPrintf(...) ((void)0)
#endif

// Default Configuration Constants
#define TIMER_ID 0
#define DEFAULT_EN_PIN 2
#define DEFAULT_SHUTDOWN_TIME_SEC 10
#define DEFAULT_ENABLE_GPIO_STATE 1

// Private Variables
static TimerHandle_t xTimer = NULL;
static int _en_pin = DEFAULT_EN_PIN;
static int _shutdown_time_sec = DEFAULT_SHUTDOWN_TIME_SEC;
static bool _enable_gpio_state = DEFAULT_ENABLE_GPIO_STATE;
static uint8_t _debug = 0;

// Private Function Prototypes
static void vTimerCallback(TimerHandle_t xTimer);
static void initialize_timer();
static bool timer_is_initialized();
static void shutdown();
static void keep_system_alive();

// Public Function Implementations
void PDL_Shutdown_Timer_init() {
    DEBUG_INIT();
    pinMode(_en_pin, OUTPUT);
    keep_system_alive();
    if (xTimer == NULL) {
        initialize_timer();
    }
}

int PDL_Shutdown_Timer_start() {
    if (!timer_is_initialized()) return pdFAIL;
    int ret = xTimerStart(xTimer, 0);
    debugPrintf("Timer %s\n", ret == pdPASS ? "started" : "failed to start");
    return ret;
}

int PDL_Shutdown_Timer_stop() {
    if (!timer_is_initialized()) return pdFAIL;
    int ret = xTimerStop(xTimer, 0);
    debugPrintf("Timer %s\n", ret == pdPASS ? "stopped" : "failed to stop");
    return ret;
}

void PDL_Shutdown_Timer_reset() {
    if (!timer_is_initialized()) return;
    int ret = xTimerReset(xTimer, 0);
    debugPrintf("Timer %s\n", ret == pdPASS ? "reset" : "failed to reset");
}

void PDL_Shutdown_Timer_deinit() {
    if (!timer_is_initialized()) return;
    if (xTimerDelete(xTimer, 0) == pdPASS) {
        xTimer = NULL;
        debugPrintf("Timer deleted\n");
    } else {
        debugPrintf("Failed to delete timer\n");
    }
}

void PDL_Shutdown_Timer_set_en_pin(uint8_t en_pin) {
    _en_pin = en_pin;
}

void PDL_Shutdown_Timer_set_shutdown_time_sec(uint32_t shutdown_time_sec) {
    _shutdown_time_sec = shutdown_time_sec;
    if (!timer_is_initialized()) return;
    if (xTimerChangePeriod(xTimer, pdMS_TO_TICKS(_shutdown_time_sec * 1000), 0) == pdPASS) {
        debugPrintf("Timer period changed to %d seconds\n", _shutdown_time_sec);
    } else {
        debugPrintf("Failed to change timer period\n");
    }
}

void PDL_Shutdown_Timer_set_enable_gpio_state(bool enable_gpio_state) {
    _enable_gpio_state = enable_gpio_state;
}

void PDL_Shutdown_Timer_set_debug(uint8_t debug) {
    if (debug >= PDL_Shutdown_Timer_Debug_MAX) {
        debugPrintf("Invalid debug level\n");
        return;
    }
    _debug = debug;
}

void PDL_Shutdown_Timer_system_shutdown() {
    shutdown();
}

// Private Function Implementations
#ifdef ENABLE_DEBUG
void debug_init() {
    Serial.begin(115200);
    while (!Serial) delay(10);
}

static void debugPrintf(const char *format, ...) {
    if (_debug != PDL_Shutdown_Timer_Debug_ON) return;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}
#endif

static void initialize_timer() {
    xTimer = xTimerCreate("Shutdown Timer", pdMS_TO_TICKS(_shutdown_time_sec * 1000), pdFALSE, TIMER_ID, vTimerCallback);
    debugPrintf("Timer %s\n", xTimer != NULL ? "created" : "failed to create");
}

static bool timer_is_initialized() {
    return xTimer != NULL;
}

static void shutdown() {
    digitalWrite(_en_pin, !_enable_gpio_state);
    debugPrintf("System shutdown\n");
}

static void keep_system_alive() {
    digitalWrite(_en_pin, _enable_gpio_state);
}

static void vTimerCallback(TimerHandle_t xTimer)
{
    shutdown();
    xTimerDelete(xTimer, 0);
    xTimer = NULL;
}
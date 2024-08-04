#include "PDL_Tilt_Sensor.h"
#include "PDL_Addressable_LED.h"
#include <Arduino.h>

#define NUM_LED 12
#define PIN_NEOPIXEL_LED 10
#define LED_INDEX_X_AXIS_OFFSET 6 // index of the led that is on the x axis
#define TILE_ANGLE_THRESHOLD 10

PDL_Tilt_Sensor tilt_sensor;

Adafruit_NeoPixel ring(NUM_LED, PIN_NEOPIXEL_LED, NEO_GRB + NEO_KHZ800);
PDL_Addressable_LED led_ring(ring);

PDL_Addressable_LED::single_color_pattern_t pat = PDL_Addressable_LED::PATTERN_ORANGE_DIM_DEFAULT_INDEX;

void onAzumithUpdate(PDL_Tilt_Sensor::AzumithCbsContext_t *context)
{
    // set led center index based on azumith
    // set led color based on azumith magnitude transition from blue to red

    uint16_t azumith_u16 = (uint16_t)(context->azumith + 360) % 360;
    uint8_t led_index = uint8_t(azumith_u16 * NUM_LED / 360) + LED_INDEX_X_AXIS_OFFSET;
    pat.r = (uint8_t)(context->azumith_magnitude * 255);
    pat.g = 0;
    pat.b = 255 - pat.r;
    pat.index = led_index;
    led_ring.setPatternSingleColor(pat);

    Serial.printf("Azumith: %d, led index: %d, color: %d %d %d\n", azumith_u16, led_index, pat.r, pat.g, pat.b);
}

void onTileted()
{
    Serial.println("Device is tilted, enable liquid effect");
    led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_ORANGE_DIM_DEFAULT_INDEX);
    tilt_sensor.setAzumithUpdateCallback(onAzumithUpdate);
}

void onLevel()
{
    Serial.println("Device is level, turn on all leds");
    tilt_sensor.setAzumithUpdateCallback(nullptr);
    led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_GREEN_FADE_ALL);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    tilt_sensor.init();
    tilt_sensor.setDebugStatus(PDL_Tilt_Sensor::DEBUG_NONE);
    tilt_sensor.setVerticalThresholds(-TILE_ANGLE_THRESHOLD, TILE_ANGLE_THRESHOLD, -TILE_ANGLE_THRESHOLD, TILE_ANGLE_THRESHOLD);
    tilt_sensor.setLoopDelay(100);
    tilt_sensor.setTiltedCallback(onTileted);
    tilt_sensor.setLevelCallback(onLevel);
    tilt_sensor.setAzumithUpdateCallback(nullptr);

    led_ring.init();
    led_ring.setDebug(0);
    led_ring.setPatternSingleColor(PDL_Addressable_LED::PATTERN_RED_CONST_ALL);
}

void loop()
{
    yield();
}
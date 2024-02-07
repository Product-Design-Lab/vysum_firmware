#include <MovingAverage.h>

#include "PDL_N20_Motor_Control.h"
#include "PDL_async_button.h"
#include <Arduino_APDS9960.h>

// #include "ComTool_Neutree.h"

#define MOVING_AVG_SAMPLES 3
#define TIMESTEP_MILLISECONDS 3

typedef enum
{
    DROP_DETECT_IDLE = 0,
    DROP_DETECT_TRIGGERED = 1,
    DROP_DETECT_FALLING = 2,
    DROP_DETECT_EXIT = 3,
    DROP_DETECT_MAX_STATES = 4
} Drop_detect_state_e;

Drop_detect_state_e drop_detect_state = DROP_DETECT_IDLE;

uint8_t u[32] = {0};
uint8_t d[32] = {0};
uint8_t l[32] = {0};
uint8_t r[32] = {0};
float lr, lr_last, lr_ave, lr_upb, lr_lowb;
bool lr_upb_out, lr_upb_in, lr_lowb_out, lr_lowb_in;
const uint8_t lr_falling_persistance = 20;
uint8_t lr_falling_count = 0;
int drop_count = 0;

float alpha = 0.005;
uint8_t count = 0;
uint8_t trigger_threshold = 5; // 5

int drop_count_previous = 0;
const unsigned long LED_ON_TIME = 200;
unsigned long led_time = 0;
boolean led_state = false;

// Timers
unsigned long tick = 0, tock = 0, timer = 0, curr_millis, prev_millis, lUTimer = 0, rUTimer = 0, lLTimer = 0,
              rLTimer = 0, dropCountTimer = 0, dropCountTimer2 = 0;

// Moving Averages
double uMeanPrev = 0, uMean = 0, dMeanPrev = 0, dMean = 0, lMeanPrev = 0, lMean = 0, rMeanPrev = 0, rMean = 0;
double uMeanDotMeanPrev = 0, uMeanDotMean = 0, dMeanDotMeanPrev = 0, dMeanDotMean = 0, lMeanDotMeanPrev = 0,
       lMeanDotMean = 0, rMeanDotMeanPrev = 0, rMeanDotMean = 0;

int testDropCount = 0, dropCountCombined = 0;
bool lUpperFlag = 0, lLowerFlag = 0, rUpperFlag = 0, rLowerFlag = 0;
#define DropFlagResetTime 0 * TIMESTEP_MILLISECONDS
#define sampleDelay 100
#define sampleDelay2 500
bool dropFlagA = false, dropFlagB = false;

// Moving Averages
MovingAverage<uint8_t, MOVING_AVG_SAMPLES> uMovingAvg;
MovingAverage<uint8_t, MOVING_AVG_SAMPLES> dMovingAvg;
MovingAverage<uint8_t, MOVING_AVG_SAMPLES> lMovingAvg;
MovingAverage<uint8_t, MOVING_AVG_SAMPLES> rMovingAvg;

void drop_detect_init()
{
    if (!APDS.begin())
    {
        Serial.println("Error initializing APDS-9960 sensor!");
    }

    drop_detect_state = DROP_DETECT_IDLE;
    for (int i = 0; i < 5; i++)
    {
        count = APDS.gestureAvailable(u, d, l, r);
        for (int j = 1; j < count; j++)
        {
            lr = l[i] - r[i];
            lr_ave = 0.5 * lr_ave + 0.5 * lr;
        }
        delay(20);
    }
}

void drop_detect_update()
{

    count = APDS.gestureAvailable(u, d, l, r); // Original
    // count = APDS.gestureAvailable(d, u, r, l);

    // --- WIP ---- //
    uMeanPrev = uMean;
    dMeanPrev = dMean;
    lMeanPrev = lMean;
    rMeanPrev = rMean;
    uMean = uMovingAvg.add(u[0]);
    dMean = dMovingAvg.add(d[0]);
    lMean = lMovingAvg.add(l[0]);
    rMean = rMovingAvg.add(r[0]);

    // Serial.print(u[0]); Serial.print(", "); Serial.print(d[0]); Serial.print(", "); Serial.print(l[0]);
    // Serial.print(", "); Serial.println(r[0]);

    prev_millis = curr_millis;
    curr_millis = millis();

    double uMeanDot = (uMean - uMeanPrev) / (float)(curr_millis - prev_millis + 0.01); // +1 avoids db0 error
    double dMeanDot = (dMean - dMeanPrev) / (float)(curr_millis - prev_millis + 0.01); // +1 avoids db0 error
    double lMeanDot = (lMean - lMeanPrev) / (float)(curr_millis - prev_millis + 0.01); // +1 avoids db0 error
    double rMeanDot = (rMean - rMeanPrev) / (float)(curr_millis - prev_millis + 0.01); // +1 avoids db0 error

    // Serial.print(uMeanDot); Serial.print(", "); Serial.print(dMeanDot); Serial.print(", "); Serial.print(lMeanDot);
    // Serial.print(", "); Serial.println(rMeanDot);

    double commonThresh = 1.95;
    double lUpperBound = commonThresh, lLowerBound = -commonThresh, rUpperBound = commonThresh,
           rLowerBound = -commonThresh;

    if (lMeanDot > lUpperBound && lUpperFlag == false && (curr_millis - dropCountTimer > sampleDelay))
    {
        lUpperFlag = true;
        lUTimer = curr_millis;
        // Serial.println("Flag lU");
        // Serial.println(lMeanDot);
    }
    if (lMeanDot < lLowerBound && lLowerFlag == false && (curr_millis - dropCountTimer > sampleDelay))
    {
        lLowerFlag = true;
        lLTimer = curr_millis;
        // Serial.println("Flag lL");
    }
    if (rMeanDot > rUpperBound && rUpperFlag == false && (curr_millis - dropCountTimer > sampleDelay))
    {
        rUpperFlag = true;
        rUTimer = curr_millis;
        // Serial.println("Flag rU");
    }
    if (rMeanDot > rLowerBound && rLowerFlag == false && (curr_millis - dropCountTimer > sampleDelay))
    {
        rLowerFlag = true;
        rLTimer = curr_millis;
        // Serial.println("Flag rL");
    }
    // Serial.print(lMeanDot); Serial.print(", "); Serial.print(rMeanDot); Serial.print(", ");
    // Serial.print(lMean); Serial.print(", "); Serial.print(rMean); Serial.print(", ");
    // Serial.print(lMeanDotMean); Serial.print(", "); Serial.print(rMeanDotMean); Serial.print(", ");
    // Serial.print(commonThresh); Serial.print(", "); Serial.print(-commonThresh); Serial.print(", ");
    // Serial.print(lUpperFlag); Serial.print(", "); Serial.print(lLowerFlag); Serial.print(", ");
    // Serial.print(rUpperFlag); Serial.print(", "); Serial.println(rLowerFlag);

    // if (lUpperFlag && lLowerFlag && rUpperFlag && rLowerFlag) {
    if ((lUpperFlag && rUpperFlag) || (lLowerFlag && rLowerFlag) || (rUpperFlag && rLowerFlag) ||
        (lUpperFlag && lLowerFlag))
    {
        testDropCount++;
        dropFlagA = true;
        // Serial.println("NB Count: " + String(testDropCount));
        lUpperFlag = 0;
        lLowerFlag = 0;
        rUpperFlag = 0;
        rLowerFlag = 0;
        dropCountTimer = curr_millis;
    }
    // Serial.println(testDropCount);
    if ((curr_millis - lUTimer) > DropFlagResetTime)
    {
        lUpperFlag = 0;
        lLowerFlag = 0;
    }
    if ((curr_millis - rUTimer) > DropFlagResetTime)
    {
        rUpperFlag = 0;
        rLowerFlag = 0;
    }

    // ------------//

    // Serial.print(l[0]); Serial.print(", "); Serial.print(r[0]); Serial.print(", "); Serial.print(u[0]);
    // Serial.print(", "); Serial.println(d[0]);

    for (int i = 0; i < count; i++)
    {
        // update left and right sensor value difference
        lr_last = lr;
        // l[i] = l[i] - (160-80);
        lr = l[i] - r[i];

        // detect threshold crossing
        lr_upb_out = (lr > lr_upb && lr_last <= lr_upb);
        lr_upb_in = (lr < lr_upb && lr_last >= lr_upb);
        lr_lowb_out = (lr < lr_lowb && lr_last >= lr_lowb);
        lr_lowb_in = (lr > lr_lowb && lr_last <= lr_lowb);

        // update threshold
        lr_ave = alpha * lr + (1.0 - alpha) * lr_ave;
        lr_upb = lr_ave + trigger_threshold;
        lr_lowb = lr_ave - trigger_threshold;

        // update state
        switch (drop_detect_state)
        {
        case DROP_DETECT_IDLE:
            if (lr_upb_out)
            {
                drop_detect_state = DROP_DETECT_TRIGGERED;
            }
            break;
        case DROP_DETECT_TRIGGERED:
            if (lr_upb_in)
            {
                drop_detect_state = DROP_DETECT_FALLING;
            }

            if (lr_lowb_out) // very fast drops
            {
                drop_detect_state = DROP_DETECT_EXIT;
            }
            break;
        case DROP_DETECT_FALLING:
            if (lr_lowb_out || lr_upb_out || lr_upb_in || lr_lowb_out)
            {
                lr_falling_count = 0;
            }
            else
            {
                lr_falling_count++;
            }

            // check if the water drop exit the FOV in the right direction
            if (lr_falling_count > lr_falling_persistance)
            {
                lr_falling_count = 0;
                drop_detect_state = DROP_DETECT_IDLE;
            }

            if (lr_lowb_out)
            {
                drop_detect_state = DROP_DETECT_EXIT;
                drop_count++;
                // Serial.println("PL Count: " + String(drop_count));
                dropFlagB = true;
            }
            break;
        case DROP_DETECT_EXIT:
            if (lr_lowb_in)
            {
                drop_detect_state = DROP_DETECT_IDLE;
            }
            break;
        default:
            drop_detect_state = DROP_DETECT_IDLE;
            break;
        }

        // ComToolPlot("lr", lr);
        // ComToolPlot("lr_ave", lr_ave);
        // ComToolPlot("lr_upb", lr_upb);
        // ComToolPlot("lr_lowb", lr_lowb);
        // ComToolPlot("state", drop_detect_state);
        // Serial.print(String(l[i])+","+String(r[i])+",");
        // Serial.println(count);
    }

    double currentTime = millis();

    if (currentTime - dropCountTimer2 < sampleDelay)
    {
        dropFlagA = false;
        dropFlagB = false;
    }
    else if (dropFlagA || dropFlagB)
    {
        dropCountCombined++;
        Serial.println("Combined Drop Count: " + String(dropCountCombined) + ", PL Count: " + String(drop_count) +
                       ", NB Count: " + String(testDropCount));
        dropFlagA = false;
        dropFlagB = false;
        dropCountTimer2 = currentTime;
    }
    // delay(20);

    // Serial.println(0);
    // ComToolPlot("drop_count", drop_count);
    // Serial.printf("drop_detect_state:%d, drop_count:%d\n", drop_detect_state, drop_count);
    // Serial.print("drop_detect_state:"); Serial.print(drop_detect_state); Serial.print(" | ");
    // Serial.print("drop_count:"); Serial.println(drop_count);
    // Serial.println(String(l)+","+String(r)+","+String(lr)+"," + String(lr_ave)+"," + String(lr_upb)+"," +
    // String(lr_lowb)+"," + String(drop_detect_state)+"," + String(drop_count));
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    drop_detect_init();

    pinMode(LED_BUILTIN, OUTPUT);

    curr_millis = millis();

    PDL_N20_Motor_Control::setPin(2, 3, 9, 8, 10);
    PDL_N20_Motor_Control::setMaxPwm(128);
    PDL_N20_Motor_Control::setPositionLimits(20000, -20000);
    PDL_N20_Motor_Control::setGain(-0.01);
    PDL_N20_Motor_Control::enable();
    // PDL_N20_Motor_Control::enableDebug();
    PDL_N20_Motor_Control::init();

    PDL_Async_Button::setPin(D0);
    PDL_Async_Button::setDebounceTime(5);
    PDL_Async_Button::setLongPressTime(1000);
    PDL_Async_Button::init();
}

void loop()
{
    if (PDL_Async_Button::getState() == PDL_Async_Button::SHORT_PRESS)
    {
        PDL_N20_Motor_Control::setPwmPercent(10);
    }
    else if (PDL_Async_Button::getState() == PDL_Async_Button::LONG_PRESS)
    {
        PDL_N20_Motor_Control::setPwmPercent(-10);
    }

    tick = millis();
    drop_detect_update();

    if (drop_count != drop_count_previous)
    {
        led_state = true;
        drop_count_previous = drop_count;
        led_time = millis();
    }

    if (led_state)
    {
        digitalWrite(LED_BUILTIN, LOW);
        PDL_N20_Motor_Control::setCurrentPosition(10000);
        PDL_N20_Motor_Control::setTargetPosition(0);

        if (millis() - led_time > LED_ON_TIME)
        {
            led_state = false;
        }
    }
    else
    {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    // delay(20);

    tock = millis();
    timer = tock - tick;

    if (timer > TIMESTEP_MILLISECONDS)
    {
        Serial.println("Warning: Running late by " + String(timer - TIMESTEP_MILLISECONDS) + " milliseconds!");
        return; // Exit loop early.
    }

    delay(TIMESTEP_MILLISECONDS - timer);
}

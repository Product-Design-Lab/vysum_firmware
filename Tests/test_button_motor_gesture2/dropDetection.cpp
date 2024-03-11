#include "dropDetection.h"

#include <Arduino_APDS9960.h>
#include <movingAvg.h>

#define MOVING_AVG_SAMPLES (3)
#define DEBOUNCE_DELAY (25)
#define THRESHHOLD (5.0)
#define LOOP_DELAY (pdMS_TO_TICKS(20)) // 20ms

uint8_t u[32] = {0};
uint8_t d[32] = {0};
uint8_t l[32] = {0};
uint8_t r[32] = {0};

uint8_t sampleCount = 0;
uint8_t debounceCount = 0;
int dropCount = 0;

double uMeanPrev = 0, uMean = 0, uMeanDot = 0;
double dMeanPrev = 0, dMean = 0, dMeanDot = 0;
double lMeanPrev = 0, lMean = 0, lMeanDot = 0;
double rMeanPrev = 0, rMean = 0, rMeanDot = 0;

bool lUpperFlag = 0, lLowerFlag = 0, rUpperFlag = 0, rLowerFlag = 0;

// Moving Averages
movingAvg uMovingAvg(MOVING_AVG_SAMPLES);
movingAvg dMovingAvg(MOVING_AVG_SAMPLES);
movingAvg lMovingAvg(MOVING_AVG_SAMPLES);
movingAvg rMovingAvg(MOVING_AVG_SAMPLES);

TickType_t xLastWakeTime;
TaskHandle_t dropDetectionTaskHandle = NULL;

void drop_detect_update();

static void dropDetectTask(void *pvParameters)
{
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        drop_detect_update();
        // APDS9960 has 32 buffers and takes 5-6 samples every 20ms,
        // the maximum delay time to avoid buffer overflow is 120ms
        // when buffer overflow, the sensor will stop sampling and stall the system
        vTaskDelayUntil(&xLastWakeTime, LOOP_DELAY);
    }
}

void dropDetectInit()
{
    if (!Serial)
        Serial.begin(115200);
    while (!Serial)
        ;

    if (!APDS.begin())
    {
        Serial.println("Error initializing APDS-9960 sensor!");
    }

    uMovingAvg.begin();
    dMovingAvg.begin();
    lMovingAvg.begin();
    rMovingAvg.begin();
 
    xTaskCreate(dropDetectTask, "dropDetectTask", 1024*2, NULL, 1, &dropDetectionTaskHandle);
}

void drop_detect_update()
{

    /*
      sampling rate is fixed by the sensor's internal state machine. use the IC time to avoid aliasing
    */
    sampleCount = APDS.gestureAvailable(u, d, l, r);
    // Serial.println(sampleCount);

    for (int i = 0; i < sampleCount; i++)
    {
        debounceCount++;
        uMeanPrev = uMean;
        dMeanPrev = dMean;
        lMeanPrev = lMean;
        rMeanPrev = rMean;

        uMean = uMovingAvg.reading(u[i]);
        dMean = dMovingAvg.reading(d[i]);
        lMean = lMovingAvg.reading(l[i]);
        rMean = rMovingAvg.reading(r[i]);

        double uMeanDot = (uMean - uMeanPrev); // there is always a fixed delay between the samples
        double dMeanDot = (dMean - dMeanPrev);
        double lMeanDot = (lMean - lMeanPrev);
        double rMeanDot = (rMean - rMeanPrev);

        // Serial.printf("u:%d, d:%d, l:%d, r:%d\n", u[i], d[i], l[i], r[i]);
        // Serial.printf("uMean:%f, dMean:%f, lMean:%f, rMean:%f\n", uMean, dMean, lMean, rMean);
        // Serial.printf("uMeanDot:%.2f, dMeanDot:%.2f, lMeanDot:%.2f, rMeanDot:%.2f\n", uMeanDot, dMeanDot, lMeanDot, rMeanDot);

        if (debounceCount > DEBOUNCE_DELAY)
        {
            lUpperFlag |= (bool)(lMeanDot > THRESHHOLD);
            lLowerFlag |= (bool)(lMeanDot < -THRESHHOLD);
            rUpperFlag |= (bool)(rMeanDot > THRESHHOLD);
            rLowerFlag |= (bool)(rMeanDot < -THRESHHOLD);

            if (lUpperFlag + lLowerFlag + rUpperFlag + rLowerFlag > 2)
            {
                dropCount++;

                lUpperFlag = 0;
                lLowerFlag = 0;
                rUpperFlag = 0;
                rLowerFlag = 0;
                debounceCount = 0;

                Serial.println("NB Count: " + String(dropCount));
            }
        }
    }
}

int getDropCount()
{
    return dropCount;
}

void setDropCount(const int count)
{
    dropCount = count;
}
#include <Arduino_APDS9960.h>
#include <MovingAverage.h>

#define MOVING_AVG_SAMPLES (4)
#define DEBOUNCE_DELAY (25)
#define THRESHHOLD (5.0)
#define THRESHHOLD_2 (2)
#define LOOP_DELAY (pdMS_TO_TICKS(50))  // 20ms

uint8_t u[32] = { 0 };
uint8_t d[32] = { 0 };
uint8_t l[32] = { 0 };
uint8_t r[32] = { 0 };

uint8_t sampleCount = 0;
uint8_t debounceCount = 0;
int dropCount = 0;

int uMeanPrev = 0, uMean = 0, uMeanDot = 0;
int dMeanPrev = 0, dMean = 0, dMeanDot = 0;
int lMeanPrev = 0, lMean = 0, lMeanDot = 0;
int rMeanPrev = 0, rMean = 0, rMeanDot = 0;

bool lUpperFlag = 0, lLowerFlag = 0;
bool rUpperFlag = 0, rLowerFlag = 0;
bool uUpperFlag = 0, uLowerFlag = 0;
bool dUpperFlag = 0, dLowerFlag = 0;


MovingAverage<int, MOVING_AVG_SAMPLES> uMovingAvg;
MovingAverage<int, MOVING_AVG_SAMPLES> dMovingAvg;
MovingAverage<int, MOVING_AVG_SAMPLES> lMovingAvg;
MovingAverage<int, MOVING_AVG_SAMPLES> rMovingAvg;

TickType_t xLastWakeTime;

void drop_detect_update();

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  if (!APDS.begin()) {
    Serial.println("Error initializing APDS-9960 sensor!");
  }
}

void loop() {
  drop_detect_update();
  // APDS9960 has 32 buffers and takes 5-6 samples every 20ms,
  // the maximum delay time to avoid buffer overflow is 120ms
  // when buffer overflow, the sensor will stop sampling and stall the system
  vTaskDelayUntil(&xLastWakeTime, LOOP_DELAY);
}

void drop_detect_update() {

  /*
    sampling rate is fixed by the sensor's internal state machine. use the IC time to avoid aliasing
  */
  sampleCount = APDS.gestureAvailable(u, d, l, r);
  // Serial.println(sampleCount);

  for (int i = 0; i < sampleCount; i++) {
    debounceCount++;
    uMeanPrev = uMean;
    dMeanPrev = dMean;
    lMeanPrev = lMean;
    rMeanPrev = rMean;

    uMean = uMovingAvg.add(u[i]);
    dMean = dMovingAvg.add(d[i]);
    lMean = lMovingAvg.add(l[i]);
    rMean = rMovingAvg.add(r[i]);

    uMeanDot = (uMean - uMeanPrev);  // there is always a fixed delay between the samples
    dMeanDot = (dMean - dMeanPrev);
    lMeanDot = (lMean - lMeanPrev);
    rMeanDot = (rMean - rMeanPrev);

    // Serial.printf("u:%d, d:%d, l:%d, r:%d\n", u[i], d[i], l[i], r[i]);
    // Serial.printf("uMean:%d, dMean:%d, lMean:%d, rMean:%d\n", uMean, dMean, lMean, rMean);
    Serial.printf("cnt:%d, uMeanDot:%d, dMeanDot:%d, lMeanDot:%d, rMeanDot:%d\n", sampleCount, uMeanDot, dMeanDot, lMeanDot, rMeanDot);

    if (debounceCount > DEBOUNCE_DELAY) {
      lUpperFlag |= (bool)(lMeanDot > THRESHHOLD);
      lLowerFlag |= (bool)(lMeanDot < -THRESHHOLD);
      rUpperFlag |= (bool)(rMeanDot > THRESHHOLD);
      rLowerFlag |= (bool)(rMeanDot < -THRESHHOLD);
      uUpperFlag |= (bool)(uMeanDot > THRESHHOLD);
      uLowerFlag |= (bool)(uMeanDot < -THRESHHOLD);
      dUpperFlag |= (bool)(dMeanDot > THRESHHOLD);
      dLowerFlag |= (bool)(dMeanDot < -THRESHHOLD);

      // Serial.printf("db_cnt:%d, u_up:%d, u_low:%d, d_up:%d, d_low:%d, l_up:%d, l_low:%d, r_up:%d, r_low:%d\n", debounceCount, uUpperFlag, uLowerFlag, dUpperFlag, dLowerFlag, lUpperFlag, lLowerFlag, rUpperFlag, rLowerFlag);

      if (lUpperFlag + lLowerFlag + rUpperFlag + rLowerFlag + uUpperFlag + uLowerFlag + dUpperFlag + dLowerFlag > THRESHHOLD_2) {
        dropCount++;
        Serial.printf("db_cnt:%d, u_up:%d, u_low:%d, d_up:%d, d_low:%d, l_up:%d, l_low:%d, r_up:%d, r_low:%d\n", debounceCount, uUpperFlag, uLowerFlag, dUpperFlag, dLowerFlag, lUpperFlag, lLowerFlag, rUpperFlag, rLowerFlag);

        lUpperFlag = 0;
        lLowerFlag = 0;
        rUpperFlag = 0;
        rLowerFlag = 0;
        dUpperFlag = 0;
        dLowerFlag = 0;
        uUpperFlag = 0;
        uLowerFlag = 0;

        debounceCount = 0;

        Serial.printf("NB_Count:%d\n", dropCount);
      }
    } 
  }
}

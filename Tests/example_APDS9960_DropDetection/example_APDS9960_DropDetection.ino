#include <Arduino_APDS9960.h>
// #include "ComTool_Neutree.h"

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
uint8_t trigger_threshold = 5;


int drop_count_previous = 0;
const unsigned long LED_ON_TIME = 200;
unsigned long led_time = 0;
boolean led_state = false;


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
    count = APDS.gestureAvailable(u, d, l, r);
    for (int i = 0; i < count; i++)
    {
        // update left and right sensor value difference
        lr_last = lr;
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
    }
    // ComToolPlot("drop_count", drop_count);
    // Serial.printf("drop_detect_state:%d, drop_count:%d\n", drop_detect_state, drop_count);
    Serial.print("drop_detect_state: "); Serial.print(drop_detect_state); Serial.print(" | ");
    Serial.print("drop_count: "); Serial.println(drop_count);
}

void setup()
{
    Serial.begin(9600);
    while (!Serial)
        ;

    drop_detect_init();

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    drop_detect_update();

    if (drop_count != drop_count_previous) {
      led_state = true;
      drop_count_previous = drop_count;
      led_time = millis();
    }

    if (led_state) {
      digitalWrite(LED_BUILTIN, LOW);

      if (millis()-led_time > LED_ON_TIME) {
        led_state = false;
      }
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    // delay(10);
}

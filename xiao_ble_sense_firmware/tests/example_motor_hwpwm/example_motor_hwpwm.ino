#include <Arduino.h>

#define PIN_MOTOR_DIR D10
#define PIN_MOTOR_PWM D9

void setup()
{  
  // Add LED RED to PWM0
  HwPWM0.addPin( PIN_MOTOR_PWM );
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  // pinMode(PIN_MOTOR_PWM, OUTPUT);
  digitalWrite(PIN_MOTOR_DIR, HIGH);
  analogWrite(PIN_MOTOR_PWM, 0);

  // Add LED BLUE to PWM1
  // HwPWM1.addPin( LED_BLUE );

  // Enable PWM modules with 15-bit resolutions(max) but different clock div
  HwPWM0.begin();
  HwPWM0.setResolution(11);
  HwPWM0.setClockDiv(PWM_PRESCALER_PRESCALER_DIV_1); // freq = 1Mhz
}


void loop()
{
  HwPWM0.writePin(PIN_MOTOR_PWM, bit(11) - 1, false);
  // analogWrite(PIN_MOTOR_PWM, 128);
  digitalWrite(PIN_MOTOR_DIR, HIGH);
  delay(1000);


  HwPWM0.writePin(PIN_MOTOR_PWM, bit(9) - 1, false);
  // analogWrite(PIN_MOTOR_PWM, 64);
  digitalWrite(PIN_MOTOR_DIR, LOW);
  delay(1000);
}

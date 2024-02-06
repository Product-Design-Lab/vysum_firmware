/*
Author  : Andrea Lombardo
Site    : https://www.lombardoandrea.com
Source  : https://github.com/AndreaLombardo/L298N/

Here you can see how to work in a common configuration. 

Speed range go from 0 to 255, default is 100.
Use setSpeed(speed) to change.

Sometimes at lower speed motors seems not running.
It's normal, may depends by motor and power supply.

Wiring schema in file "L298N - Schema_with_EN_pin.png"
*/

// Include the library
#include <L298N.h>

// Pin definition
const unsigned int IN1 = D8;
const unsigned int IN2 = D10;
const unsigned int EN = D9;

// Create one motor instance
L298N motor(EN, IN1, IN2);

void setup()
{

}

void loop()
{

  motor.setSpeed(70);
  motor.forward();
  delay(1000);

  motor.stop();
  delay(1000);

  motor.setSpeed(70);
  motor.backward();
  delay(1000);

  motor.stop();
  delay(1000);

}
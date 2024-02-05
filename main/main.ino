#include "diagnostics.h"
#include "pose_checker.hpp"
#include "MotorN20.hpp"
#include "drop_detection.hpp"
#include "Button.hpp"


void setup()
{
  DIAG::init();
  IMU::init();
  MotorN20::init();
  VCNL::init();
  Button::init();

}

void loop()
{
  // MotorN20::set_mode(MotorN20::CONTROL_PWM);
  // MotorN20::set_target_pwm(0.5);
  // delay(1000);

  // MotorN20::set_target_pwm(0);
  // delay(1000);

  // MotorN20::set_target_pwm(-0.5);
  // delay(1000);

  // MotorN20::set_target_pwm(0);
  // delay(1000);

  MotorN20::set_mode(MotorN20::CONTROL_POSITION);
  MotorN20::set_current_position(0);
  
  MotorN20::set_target_position(1000);
  delay(5000);

  MotorN20::set_target_position(0);
  delay(5000);
}

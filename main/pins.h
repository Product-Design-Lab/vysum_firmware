// ## Default (cannot change)
// D4 - I2C SDA
// D5 - I2C SCL
// D6 - Serial Tx Note that Seeed xiao uses USB CDC to emulate serial. Serial Tx is not used
// D7 - Serial Rx
// D11 - LED_RED
// D12 - LED_BLUE
// D13 - LED_GREEN

#define PIN_LED_RED LED_RED
#define PIN_LED_GREEN LED_GREEN
#define PIN_LED_BLUE LED_BLUE

#define PIN_BUTTON          D0  // need GPIO pull-up
#define PIN_VISEN           D1  // analog input
#define PIN_MOTOR_ENCODER_A D2  //digital input
#define PIN_MOTOR_ENCODER_B D3  //digital input
#define PIN_I2C_SDA         D4 
#define PIN_I2C_SCL         D5
#define PIN_POWER_EN        D6  // digital output
#define PIN_MOTOR_PWM_1     D8  // PWM output
#define PIN_MOTOR_PWM_2     D9  // PWM output
#define PIN_LED_DATA        D10 // digital output
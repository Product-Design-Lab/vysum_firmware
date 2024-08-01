// rtos task priority

#define IMU_TASK_PRIORITY 2
#define DAIG_TASK_PRIORITY 3
#define MOTOR_TASK_PRIORITY 2
#define DROP_TASK_PRIORITY 2
#define BUTTON_TASK_PRIORITY 2

// rtos task stack size
#define IMU_TASK_STACK_SIZE 1024
#define DAIG_TASK_STACK_SIZE 1024
#define MOTOR_TASK_STACK_SIZE 1024
#define DROP_TASK_STACK_SIZE 1024
#define BUTTON_TASK_STACK_SIZE 1024

// rtos task delay
#define IMU_LOOP_DELAY_MS 100
#define DIAG_LOOP_DELAY_MS 100
#define MOTOR_LOOP_DELAY_MS 10 // TODO: Tune this
#define DROP_LOOP_DELAY_MS 100
#define BUTTON_NOTIFY_WAIT_MS 1000

// project configuration
#define GRIP_PWM -0.5
#define RELEASE_PWM 0.6
#define DISPENSE_MOTOR_ADVANCE -7000
#define STALL_THRESHOLD_SPEED 5
#define RETRACT_TIMEOUT 5000 //3000
#define DISPENSE_FAST_TIMEOUT 5000 //3000
#define DISPENSE_SLOW_TIMEOUT 5000 //3000
#define DISPENSE_HIGH_TIMEOUT 2000
#define DISPENSE_FAST_SPEED 15 // TODO: tune this speed
#define DISPENSE_SLOW_SPEED 10 // TODO: tune this speed
#define RETRACT_SPEED 50 // TODO: tune this speed
#define HOLD_TIMEOUT 300

// LED ring
#define LED_COUNT 12
#define LED_BRIGHTNESS 80
#define LED_RATE 5

// misc
#define SERIAL_TIMEOUT 1000   //ms
#define SHUTDOWN__TIMEOUT 60  //sec

// Control Flags
bool tilt_limit_flag = false;
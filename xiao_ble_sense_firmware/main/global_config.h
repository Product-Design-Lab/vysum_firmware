// rtos task priority

#define IMU_TASK_PRIORITY 2
#define DAIG_TASK_PRIORITY 3
#define MOTOR_TASK_PRIORITY 2


// rtos task stack size
#define IMU_TASK_STACK_SIZE 1024
#define DAIG_TASK_STACK_SIZE 1024
#define MOTOR_TASK_STACK_SIZE 1024

//rtos task delay
#define IMU_LOOP_DELAY_MS 100
#define DIAG_LOOP_DELAY_MS 100
#define MOTOR_LOOP_DELAY_MS 50
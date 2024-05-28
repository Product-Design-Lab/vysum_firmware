
## locate FreeRTOSConfig.h

For ubuntu
~/.arduino15/packages/Seeeduino/hardware/nrf52/1.1.7/cores/nRF5/freertos/config/FreeRTOSConfig.h
note that 1.1.7 is the version number, it may change in the future

For windows
%USERPROFILE%\AppData\Local\Arduino15\packages\Seeeduino\hardware\nrf52\1.1.8\cores\nRF5\freertos\config\FreeRTOSConfig.h
note that 1.1.8 is the version number, it may change in the future

## set the following in FreeRTOSConfig.h

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS                            1
#define configUSE_TRACE_FACILITY                                 1
#define configUSE_STATS_FORMATTING_FUNCTIONS                     1

#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()                 // configured in delay.c
#define portGET_RUN_TIME_COUNTER_VALUE()                         (DWT->CYCCNT)   
#ifndef MYCLASS_H
#define MYCLASS_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <Adafruit_TinyUSB.h>

class MyClass {
public:
    static MyClass* createInstance();
    static void deleteInstance(MyClass* instance);
    void init();

    void function1();

private:
    static const int MAX_INSTANCES = 3;
    static int instanceCount;
    static MyClass* instances[MAX_INSTANCES];

    // Static memory allocation for tasks
    static StackType_t taskStacks[MAX_INSTANCES][configMINIMAL_STACK_SIZE];
    static StaticTask_t taskTCBs[MAX_INSTANCES];

    TaskHandle_t taskHandle;
    char taskName[16];

    MyClass(const char* name);
    ~MyClass();

    static void taskFunction(void* pvParameters);
};

#endif // MYCLASS_H

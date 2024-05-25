#include "MyClass.h"

int MyClass::instanceCount = 0;
MyClass* MyClass::instances[MAX_INSTANCES] = {nullptr, nullptr, nullptr};

// Define static memory for FreeRTOS tasks
StackType_t MyClass::taskStacks[MAX_INSTANCES][configMINIMAL_STACK_SIZE];
StaticTask_t MyClass::taskTCBs[MAX_INSTANCES];

MyClass::MyClass(const char* name) : taskHandle(nullptr) {
    strncpy(taskName, name, sizeof(taskName) - 1);
    taskName[sizeof(taskName) - 1] = '\0'; // Ensure null-termination

    for (int i = 0; i < MAX_INSTANCES; ++i) {
        if (instances[i] == nullptr) {
            instances[i] = this;
            break;
        }
    }
}

MyClass::~MyClass() {
    if (taskHandle != nullptr) {
        vTaskDelete(taskHandle);
    }
}

MyClass* MyClass::createInstance() {
    if (instanceCount < MAX_INSTANCES) {
        for (int i = 0; i < MAX_INSTANCES; ++i) {
            if (instances[i] == nullptr) {
                char taskName[16];
                snprintf(taskName, sizeof(taskName), "Task_%d", i + 1);
                MyClass* instance = new MyClass(taskName);
                ++instanceCount;
                return instance;
            }
        }
    }
    return nullptr; // No available slot for a new instance
}

void MyClass::deleteInstance(MyClass* instance) {
    for (int i = 0; i < MAX_INSTANCES; ++i) {
        if (instances[i] == instance) {
            delete instances[i];
            instances[i] = nullptr;
            --instanceCount;
            return;
        }
    }
}

void MyClass::init() {
    for (int i = 0; i < MAX_INSTANCES; ++i) {
        if (instances[i] == this && taskHandle == nullptr) {
            taskHandle = xTaskCreateStatic(taskFunction, taskName, configMINIMAL_STACK_SIZE, this, 1, taskStacks[i], &taskTCBs[i]);
        }
    }
}

void MyClass::taskFunction(void* pvParameters) {
    MyClass* instance = static_cast<MyClass*>(pvParameters);
    while (true) {
        Serial.println(instance->taskName);
        instance->function1();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Example delay
    }
}

void MyClass::function1() {
    // Some functionality
    Serial.println("Function 1 executed.");
}

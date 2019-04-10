#ifndef __TASK_INITCONFIG_H
#define __TASK_INITCONFIG_H
#include "FreeRTOS.h"
#include "task.h"



extern void vTaskStart(void *pvParameters);


extern TaskHandle_t xHandleTaskLED;
extern TaskHandle_t xHandleTaskStart;	
extern TaskHandle_t xHandleTaskCANParse;
extern TaskHandle_t xHandleTaskRCParse;
extern TaskHandle_t xHandleTaskIMUData;
extern TaskHandle_t xHandleTaskPCParse;

#endif

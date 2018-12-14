#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include "FreeRTOS.h"
#include "task.h"

extern char g_2006_angle_flag;
extern char g_2006_angle_reset;

extern void vTaskControl(void *pvParameters);


#endif


#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include "FreeRTOS.h"
#include "task.h"
#include <stm32f4xx.h>



typedef struct
{
    u8 shoot_cycle;
    u8 gimble_cycle;
    
} control_cycle_t;

extern void vTaskControl(void *pvParameters);


#endif


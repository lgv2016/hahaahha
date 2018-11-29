#include <task_config.h>
#include <stm32f4xx.h>


TaskHandle_t xHandleTaskLED 		= NULL;
TaskHandle_t xHandleTaskStart 	    = NULL;

void vTaskLED(void *pvParameters)
{
    while(1)
    {
        GPIO_ResetBits(GPIOG, GPIO_Pin_1);
        vTaskDelay(200);
        GPIO_SetBits(GPIOG, GPIO_Pin_1);
        vTaskDelay(200);
    }
}


void vTaskStart(void *pvParameters)
{
	taskENTER_CRITICAL();
    xTaskCreate(vTaskLED,             
                "vTaskLED",           
                128,        
                NULL,                  
                2,        
                &xHandleTaskLED);   
	vTaskDelete(xHandleTaskStart);
    taskEXIT_CRITICAL();  
}










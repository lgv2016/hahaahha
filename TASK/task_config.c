#include <stm32f4xx.h>
#include <motor_cradle_head.h>

#include <drive_control.h>

#include <task_config.h>
#include <task_can_parse.h>
#include <task_control.h>
#include <task_usart_parse.h>


TaskHandle_t xHandleTaskLED 		= NULL;
TaskHandle_t xHandleTaskStart 	    = NULL;
TaskHandle_t xHandleTaskCANParse 	= NULL;
TaskHandle_t xHandleTaskControl 	= NULL;
TaskHandle_t xHandleTaskUSARTParse 	= NULL;


void vTaskLED(void *pvParameters)
{
	vTaskDelay(5000);
    while(1)
    {
        GPIO_ResetBits(GPIOG, GPIO_Pin_1);
        vTaskDelay(500);
        GPIO_SetBits(GPIOG, GPIO_Pin_1);
        vTaskDelay(500);
		
    }
}


void vTaskStart(void *pvParameters)
{
	taskENTER_CRITICAL();
	
	xTaskCreate(vTaskCANParse,             
                "vTaskCANParse",           
                128,        
                NULL,                  
                0,        
                &xHandleTaskCANParse);
	
	xTaskCreate(vTaskUSARTParse,             
                "vTaskUSARTParse",           
                256,        
                NULL,                  
                1,        
                &xHandleTaskUSARTParse);
	
	xTaskCreate(vTaskControl,             
                "vTaskControl",           
                128,        
                NULL,                  
                2,        
                &xHandleTaskControl);
				
				
	xTaskCreate(vTaskLED,             
                "vTaskLED",           
                128,        
                NULL,                  
                3,        
                &xHandleTaskLED);			
				
				
				
	vTaskDelete(xHandleTaskStart);
	
    taskEXIT_CRITICAL();  
}










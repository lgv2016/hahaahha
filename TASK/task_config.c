#include <stm32f4xx.h>
#include <motor_cradle_head.h>

#include <drive_imu.h>
#include <drive_control.h>
#include <drive_delay.h>

#include <task_config.h>
#include <task_can_parse.h>
#include <task_control.h>
#include <task_usart_parse.h>
#include <task_rc_parse.h>
#include <task_imudata.h>

#include <task_pc_parse.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

#include <bsp_nvic.h>
#include <robotstatus.h>

TaskHandle_t xHandleTaskLED 		= NULL;
TaskHandle_t xHandleTaskStart 	    = NULL;
TaskHandle_t xHandleTaskCANParse 	= NULL;
TaskHandle_t xHandleTaskControl 	= NULL;
TaskHandle_t xHandleTaskUSARTParse 	= NULL;
TaskHandle_t xHandleTaskRCParse 	= NULL;
TaskHandle_t xHandleTaskIMUData 	= NULL;
TaskHandle_t xHandleTaskPCParse 	= NULL;

void vTaskLED(void *pvParameters)
{	
	
	vTaskDelay(200);
    while(1)
    {
		if(robot_status.mpu6500_status==MPU6500_INIT) 
		{
			taskENTER_CRITICAL();
			while(mpu_dmp_init());
			robot_status.mpu6500_status=MPU6500_SUCCESS;
			taskEXIT_CRITICAL();  
		}
		
		vTaskDelay(8);
    }
}


void vTaskStart(void *pvParameters)
{
	taskENTER_CRITICAL();
	
	xTaskCreate(vTaskRCParse,             
                "vTaskRCParse",           
                512,        
                NULL,                  
                5,        
                &xHandleTaskRCParse);
	
	xTaskCreate(vTaskCANParse,             
                "vTaskCANParse",           
                256,        
                NULL,                  
                6,        
                &xHandleTaskCANParse);
	
	xTaskCreate(vTaskIMUData,             
                "vTaskIMUData",           
                384,        
                NULL,                  
                4,        
                &xHandleTaskIMUData);	
				
	xTaskCreate(vTaskUSARTParse,             
                "vTaskUSARTParse",           
                256,        
                NULL,                  
                3,        
                &xHandleTaskUSARTParse);
	
	
	xTaskCreate(vTaskPCParse,             
                "vTaskPCParse",           
                128,        
                NULL,                  
                2,        
                &xHandleTaskPCParse);
				
	xTaskCreate(vTaskControl,             
                "vTaskControl",           
                256,        
                NULL,                  
                1,        
                &xHandleTaskControl);
				
	xTaskCreate(vTaskLED,             
                "vTaskLED",           
                512,        
                NULL,                  
                0,        
                &xHandleTaskLED);			
							
	vTaskDelete(xHandleTaskStart);
	
    taskEXIT_CRITICAL();  
}










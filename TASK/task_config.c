#include <stm32f4xx.h>
#include <motor_cradle_head.h>

#include <drive_imu.h>
#include <drive_control.h>

#include <task_config.h>
#include <task_can_parse.h>
#include <task_control.h>
#include <task_usart_parse.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 


#include <robotstatus.h>

TaskHandle_t xHandleTaskLED 		= NULL;
TaskHandle_t xHandleTaskStart 	    = NULL;
TaskHandle_t xHandleTaskCANParse 	= NULL;
TaskHandle_t xHandleTaskControl 	= NULL;
TaskHandle_t xHandleTaskUSARTParse 	= NULL;


void vTaskLED(void *pvParameters)
{	
    while(1)
    {
		if(robot_status.imu_status==CORRECT_START) 
		{
			taskENTER_CRITICAL();
			while(mpu_dmp_init());
			robot_status.imu_status=CORRECT_FINISH;
			taskEXIT_CRITICAL();  
		}
		
		if(robot_status.imu_status==CORRECT_FINISH)
		{
			if(mpu_mpl_get_data()==0)
		    {
				MPU_Get_Gyroscope();
				robot_status.imu_data=DATA_TRUE;
		    }
		}
		vTaskDelay(5);
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
                512,        
                NULL,                  
                3,        
                &xHandleTaskLED);			
							
	vTaskDelete(xHandleTaskStart);
	
    taskEXIT_CRITICAL();  
}










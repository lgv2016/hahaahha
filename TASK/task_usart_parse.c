#include <task_usart_parse.h>
#include <drive_delay.h>
#include <motor_cradle_head.h>
#include <drive_control.h>

#include <robotstatus.h>

float s=5;
//裁判系统数据解算任务
void vTaskUSARTParse(void *pvParameters)
{
	//Snail_Calibration();

    while(1)
    {
		if(robot_status.gimbal_data==GIMBAL_MOTOR_GYRO)
		{
//		g_speed_target.pitch=s;
//		vTaskDelay(1500);
//		g_speed_target.pitch=-s;
//	 vTaskDelay(1500);
		}
	 vTaskDelay(10);
		
		
    }
}

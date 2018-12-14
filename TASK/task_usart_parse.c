#include <task_usart_parse.h>
#include <drive_delay.h>
#include <motor_cradle_head.h>
#include <drive_control.h>
void vTaskUSARTParse(void *pvParameters)
{
	Snail_Calibration();
    while(1)
    {
	 //	g_angle_target.pitch=260;
		vTaskDelay(2000);
		//g_angle_target.pitch=270;
		vTaskDelay(2000);
    }
}

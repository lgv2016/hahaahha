#include <task_usart_parse.h>
#include <drive_delay.h>
#include <motor_cradle_head.h>
void vTaskUSARTParse(void *pvParameters)
{
	Snail_Calibration();
    while(1)
    {
		vTaskDelay(100);
    }
}

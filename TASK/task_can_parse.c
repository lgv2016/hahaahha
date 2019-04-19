#include <task_can_parse.h>
#include <drive_delay.h>
#include <motor_cradle_head.h>
#include <motor_chassis.h>

#include <drive_shoot.h>



void vTaskCANParse(void *pvParameters)
{
    while(1)
    {
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		Get_6623_data(s_rx_message);
        Get_2006_data(s_rx_message);
        Get_3510_data(s_rx_message); 
		Get_GIMBLE_data(s_rx_message);
		
		GET_209_Data(s_rx_message);
    }
}


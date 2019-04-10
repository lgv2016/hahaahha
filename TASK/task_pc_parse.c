#include <task_pc_parse.h>

#include <drive_usart.h>

#include <robotstatus.h>

void vTaskPCParse(void *pvParameters)
{
	const TickType_t xBlockTime = pdMS_TO_TICKS(100); // 设置最大等待时间为 500ms
	while(1)
	{
		ulTaskNotifyTake(pdTRUE,xBlockTime);
		
		if(robot_status.gimbal_mode==AUTO&&robot_status.gimbal_status!=NO_INIT)
	    {
			MiniPC_Rece_Resolver();
	    }
		
		MiniPC_Send_Data(0x03);
	}
}


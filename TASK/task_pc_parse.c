#include <task_pc_parse.h>

#include <drive_usart.h>

#include <robotstatus.h>

void vTaskPCParse(void *pvParameters)
{
	const TickType_t xBlockTime = pdMS_TO_TICKS(100); // 设置最大等待时间为 100ms
	while(1)
	{
		ulTaskNotifyTake(pdTRUE,xBlockTime);
		
		MiniPC_Rece_Resolver();
	
	}
}


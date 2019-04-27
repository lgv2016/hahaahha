#include <task_rc_parse.h>
#include <robotstatus.h>
#include <task_control.h>
#include <drive_rc.h>





void vTaskRCParse(void *pvParameters)
{	
	while(1)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	    RC_Data_Parse();
		
		
		
	}
	
}


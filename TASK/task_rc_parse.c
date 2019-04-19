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
		if(g_rc_control.rc.s1==1)
		{
			robot_status.control_mode=USE_RC;          //ң�ؿ���ģʽ
			robot_status.gimbal_mode=MANUAL;
		}
		else if(g_rc_control.rc.s1==3)
		{
			robot_status.control_mode=USE_PC;         //PC����ģʽ
			
		}
		else if(g_rc_control.rc.s1==2)
		{
			robot_status.control_mode=USE_RC;
			robot_status.gimbal_mode=AUTO;
		}
		if(robot_status.control_mode==USE_RC)
		{
			
		}
	}
	
}


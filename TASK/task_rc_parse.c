#include <task_rc_parse.h>
#include <robotstatus.h>
#include <task_control.h>
#include <drive_rc.h>





void vTaskRCParse(void *pvParameters)
{
	
	u16 e_lastval=0;
	u8 s2_lastval=0;        //单连发控制
	
	while(1)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	    RC_Data_Parse();
		if(g_rc_control.rc.s1==1)
		{
			robot_status.control_mode=USE_RC;          //遥控控制模式
		}
		else if(g_rc_control.rc.s1==3)
		{
			robot_status.control_mode=USE_PC;         //PC控制模式
			
		}
		else if(g_rc_control.rc.s1==2)
		{
			
		}
		if(robot_status.control_mode==USE_RC)
		{
		
			if(g_rc_control.rc.s2==1)              //单发控制
			{
				if(s2_lastval==3)
				{
					robot_status.shoot_mode=AWM;
					g_2006_angle_reset=1;         //角度置零
					s2_lastval=0;
				}
			}
			else if(g_rc_control.rc.s2==3)        //单发
			{
				s2_lastval=3;
				robot_status.chassis_mode=CH_SPEED;
				robot_status.shoot_mode=RELOAD;
			}
			else if(g_rc_control.rc.s2==2)        //连发
			{
				robot_status.chassis_mode=CH_ROTATE;
				robot_status.shoot_mode=AK47;
			}
		}
		else if(robot_status.control_mode==USE_PC)
		{
			if(g_rc_control.key.k[E]==0X01)
			{
				e_lastval++;
			}
			if(e_lastval%2==1)
			{
				if(g_rc_control.mouse.press_l)
				{
					robot_status.shoot_mode=AWM;   //单发
				}
					
			}
			else if(e_lastval%2==0)
			{
				if(g_rc_control.mouse.press_l)
				{
					robot_status.shoot_mode=AK47;  //连发
				}
				else
					robot_status.shoot_mode=RELOAD;
			}
		}
		
	}
	
}


#include <task_rc_parse.h>
#include <robotstatus.h>
#include <task_control.h>
#include <drive_rc.h>





void vTaskRCParse(void *pvParameters)
{
	
	u16 e_lastval=0;
	u8 s2_lastval=0;        //����������
	
	while(1)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
	    RC_Data_Parse();
		if(g_rc_control.rc.s1==1)
		{
			robot_status.control_mode=USE_RC;          //ң�ؿ���ģʽ
		}
		else if(g_rc_control.rc.s1==3)
		{
			robot_status.control_mode=USE_PC;         //PC����ģʽ
			
		}
		else if(g_rc_control.rc.s1==2)
		{
			
		}
		if(robot_status.control_mode==USE_RC)
		{
		
			if(g_rc_control.rc.s2==1)              //��������
			{
				if(s2_lastval==3)
				{
					robot_status.shoot_mode=AWM;
					g_2006_angle_reset=1;         //�Ƕ�����
					s2_lastval=0;
				}
			}
			else if(g_rc_control.rc.s2==3)        //����
			{
				s2_lastval=3;
				robot_status.chassis_mode=CH_SPEED;
				robot_status.shoot_mode=RELOAD;
			}
			else if(g_rc_control.rc.s2==2)        //����
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
					robot_status.shoot_mode=AWM;   //����
				}
					
			}
			else if(e_lastval%2==0)
			{
				if(g_rc_control.mouse.press_l)
				{
					robot_status.shoot_mode=AK47;  //����
				}
				else
					robot_status.shoot_mode=RELOAD;
			}
		}
		
	}
	
}


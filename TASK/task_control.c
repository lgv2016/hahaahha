#include <task_control.h>
#include <drive_control.h>
#include <drive_delay.h>
#include <robotstatus.h>
#include <motor_cradle_head.h>
#include <drive_rc.h>
void Chassis_Move(u16 xmaxval,u16 ymaxval)
{
	float xspeed,yspeed;
	if(g_rc_control.rc.ch2==1684)
	{
		xspeed=xmaxval;
	}
	else if(g_rc_control.rc.ch2==364)
	{
		xspeed=-xmaxval;
	}
	else
	{
		xspeed=0;
	}
	
	if(g_rc_control.rc.ch3==1684)
	{
		yspeed=ymaxval;
	}
	else if(g_rc_control.rc.ch3==364)
	{
		yspeed=-ymaxval;
	}
	else
	{
		yspeed=0;
	}
	Speed_Chassis_Control(xspeed,yspeed,0);
}
char g_2006_angle_reset=0;
char g_2006_angle_flag=1;
void vTaskControl(void *pvParameters)
{
	robot_status.chassis_mode=CH_SPEED;
	u8 last_shoot_mode;
	vTaskDelay(1500);  //6623开始时数据不正确，需要延时等待
    while(1)
    {
		if(robot_status.shoot_mode==AWM)
		{
			last_shoot_mode=AWM;
			if(g_2006_angle_reset==1)
			{
				g_2006_angle_flag=1;
				g_2006_angle_reset=0;
				g_data_2006.count=0;
			}
			g_angle_target.shoot=60;
			Angle_2006_Control(g_angle_target);
		}
		else if(robot_status.shoot_mode==AK47)
		{
			last_shoot_mode=AK47;
			g_speed_target.shoot=2000;
			Speed_2006_Control(g_speed_target);
		}
		else if(robot_status.shoot_mode==RELOAD)
		{
			if(last_shoot_mode==AWM)
			{
				g_angle_target.shoot=60;
				Angle_2006_Control(g_angle_target);
			}
			else if(last_shoot_mode==AK47)
			{
				g_speed_target.shoot=0;
				Speed_2006_Control(g_speed_target);
			}
		}
		if(robot_status.chassis_mode==CH_SPEED)
		{
			if(robot_status.control_mode==USE_RC)
			{
				Chassis_Move(1500,1500);
			}

		
		}
		vTaskDelay(10);
	}
}

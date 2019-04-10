#include <drive_shoot.h>
#include <drive_control.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>


u8 AWM_Shoot_Control(float angle)
{	
	if(robot_status.shoot_mode!=AWM)
	{
		return 1;
	}
	
	if(g_rc_control.mouse.press_l)	
	{
		g_angle_target.shoot=angle;              //Ŀ��Ƕȿ���
		Angle_2006_Control(g_angle_target);
		if(abs(g_infc.angle_outer_error.shoot)<0.2)
		{
			return 1;
		}	
	}
}


void AK47_Shoot_Control(int16_t  fire_rate)
{  
	if(g_rc_control.mouse.press_l)
	{
		g_speed_target.shoot=fire_rate;            //Ŀ���ٶȿ���
		Speed_2006_Control(g_speed_target);
	}
}


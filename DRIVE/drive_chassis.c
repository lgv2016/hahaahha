#include <drive_chassis.h>


#include <drive_control.h>

#include <drive_delay.h> 
#include <drive_imu.h>
#include <motor_chassis.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>



//µ×ÅÌ¸úËæÔÆÌ¨
u8 CHASSIS_Follow_Gimble_Control()
{
	if(robot_status.chassis_mode==CH_ROTATE)
		return 1;
	
	robot_status.chassis_mode=CH_FOLLOW_GIMBAL;
	
	if(g_rc_control.key.k[SHIFT])
	{
		return 1;
	}
	
	g_angle_target.ch_rotate= g_data_6623.angle[YAW];
	Angle_Rotate_Control(g_angle_target);
	
	if(abs(g_infc.angle_outer_error.ch_rotate)<2.0f)
	{
		return 1;
	}
	return 0;
}

//µ×ÅÌÒÆ¶¯
void CHASSIS_Move_Control(int16_t vx,int16_t vy)
{
	if(g_rc_control.key.k[W])
	{
		if(CHASSIS_Follow_Gimble_Control())
		{
			Speed_Chassis_Control(0,vy,0);
		}
	}
	else if(g_rc_control.key.k[A])
	{
		if(CHASSIS_Follow_Gimble_Control())
		{
			Speed_Chassis_Control(-vx,0,0);
		}
	}
	
	else if(g_rc_control.key.k[D])
	{
		if(CHASSIS_Follow_Gimble_Control())
		{
			Speed_Chassis_Control(vx,0,0);
		}
	}
	else if(g_rc_control.key.k[S])
	{
		if(CHASSIS_Follow_Gimble_Control())
		{
			Speed_Chassis_Control(0,-vy,0);
		}
	}
}


//µ×ÅÌ×ÔÐý
void CHASSIS_Rotate_Control(int16_t rotate_speed)
{
	if(g_rc_control.key.k[Q])
	{
		robot_status.chassis_mode=CH_ROTATE;
		g_speed_target.ch_rotate=rotate_speed;
		Speed_Rotate_Control(g_speed_target);
	}
	
}


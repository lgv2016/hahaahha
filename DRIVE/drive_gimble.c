#include <drive_gimble.h>
#include <drive_control.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>

float YAW_INIT_ANGLE=347.0f;
float PIT_INIT_ANGLE=296.5f;

void GIMBLE_Control()
{
	if(robot_status.gimbal_mode==MANUAL)
	{
		if(robot_status.gimbal_status==NO_INIT)
		{
			g_angle_target.pitch = PIT_INIT_ANGLE;
			g_angle_target.yaw   = YAW_INIT_ANGLE;
		}
		if(robot_status.gimbal_status==INIT_GOOD)
		{
			YAW_INIT_ANGLE=0;
			

			
			g_angle_target.pitch = PIT_INIT_ANGLE+g_pit_target;
			g_angle_target.yaw   = YAW_INIT_ANGLE+g_yaw_target;
			
			
			
//			g_angle_target.ch_rotate=YAW_INIT_ANGLE+g_yaw_target;
//			
//	        Angle_Rotate_Control(g_angle_target);
		
			
		}
    }
   Angle_6623_Control(g_angle_target);
}



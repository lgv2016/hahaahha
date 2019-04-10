#include <drive_gimble.h>
#include <drive_control.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>


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
			g_angle_target.pitch = PIT_INIT_ANGLE+g_pit_target;
			g_angle_target.yaw   = g_yaw_target;
		}
    }
   Angle_6623_Control(g_angle_target);
}


#include <drive_gimble.h>



#include <drive_control.h>

#include <drive_delay.h> 
#include <drive_imu.h>
#include <motor_chassis.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>


#define YAW_INIT_ANGLE    281.0f
#define PIT_INIT_ANGLE    110.0f




void GIMBLE_Control()
{
	if( robot_status.gimbal_status==INIT_GOOD)
	{
		g_angle_target.pitch =PIT_INIT_ANGLE+g_pit_target;
		g_angle_target.yaw   =YAW_INIT_ANGLE+g_yaw_target;
	}
    Angle_6623_Control(g_angle_target);
}
#include <robotstatus.h>

robot_status_t robot_status;

void BSP_ROBOT_Init(void)
{
	robot_status.shoot_mode      =  SHOOT_NO;
	robot_status.fric_status     =  FRIC_OFF;
	
	
    robot_status.gimbal_mode     =  GIMBAL_NO;
	robot_status.motor_yaw       =  MOTOR_GIMBAL_NO;
	robot_status.motor_pit       =  MOTOR_GIMBAL_NO;
	robot_status.gimbal_data     =  GIMBAL_MOTOR_NO;
	
	robot_status.depot_status    =  DEPOT_OFF;

	robot_status.posture_status  =  POSTURE_NORMAL;
}


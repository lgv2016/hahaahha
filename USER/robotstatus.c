#include <robotstatus.h>

robot_status_t robot_status;

void BSP_ROBOT_Init(void)
{
	robot_status.chassis_mode  = CH_SPEED;
	robot_status.control_mode  = USE_RC;
	robot_status.imu_data      = DATA_FALSE;
	robot_status.imu_status    = NO_CORRECT;
	robot_status.shoot_mode    = NO_SHOOT;
	robot_status.gimbal_data   = NO_DATA;
	robot_status.gimbal_status = NO_INIT;
	
}


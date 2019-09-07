#include <task_imudata.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include <drive_imu.h>
#include <robotstatus.h>
#include <drive_usart.h>



extern void yaw_angle_cal(void);

void vTaskIMUData(void *pvParameters)
{
	while(1)
	{
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		if(robot_status.mpu6500_status==MPU6500_SUCCESS)
		{
			if(mpu_mpl_get_data()==0)
		    {
				MPU_Get_Gyroscope();

				yaw_angle_cal();
				MiniPC_Send_Data(0x03);
				robot_status.gimbal_data=GIMBAL_MOTOR_GYRO;
		    }
		}
	}
}


extern void yaw_angle_cal()
{
	
	if(g_imu_data.yaw-g_imu_data.last_yaw<-180)
	{
		g_imu_data.count++;
	}
	
	else if(g_imu_data.yaw-g_imu_data.last_yaw>180)
	{
		g_imu_data.count--;
	}
	
	g_imu_data.absolute_yaw=g_imu_data.count*360.0f+g_imu_data.yaw-180.0f;
	
}


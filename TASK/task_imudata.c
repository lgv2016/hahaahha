#include <task_imudata.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include <drive_imu.h>
#include <robotstatus.h>
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
		    }
		}
	}
}

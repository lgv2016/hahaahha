#ifndef __ROBOTSTATUS_H
#define __ROBOTSTATUS_H

#include <stm32f4xx.h>
//云台控制状态

//mpu6500状态
enum
{
	CORRECT,         //未开始
	CORRECT_START,  //开始校准
	CORRECT_FINISH //校准完成
};
//mpu6500数据状态
enum
{
	DATA_FALSE,
	DATA_TRUE
};
typedef struct
{
	u8  imu_status;
    u8  imu_data;    
	
	
    u8  control_mode;   
    u8  move_mode;     
    u8  shoot_mode;
	
}robot_status_t;

extern robot_status_t robot_status;

#endif

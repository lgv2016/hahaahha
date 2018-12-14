#ifndef __ROBOTSTATUS_H
#define __ROBOTSTATUS_H

#include <stm32f4xx.h>

//遥控模式  control_mode
enum
{
	USE_RC,      //遥控
	USE_PC       //电脑
};
//mpu6500状态   imu_status
enum
{
	NO_CORRECT,         //未开始
	CORRECT_START,  //开始校准
	CORRECT_FINISH //校准完成
};
//mpu6500数据状态   imu_data
enum
{
	DATA_FALSE,
	DATA_TRUE
};
//单发连发模式    shoot_mode
enum
{
	NO_SHOOT,
	RELOAD,        //填弹
	AWM,
	AK47
};
//底盘模式    chassis_mode
enum
{
	CH_SPEED,   //速度
	CH_ANGLE,    //角度
	CH_ROTATE    //自旋
};
typedef struct
{
	u8  imu_status;  
    u8  imu_data;    
    u8  control_mode;   
    u8  move_mode;     
    u8  shoot_mode;
	u8  chassis_mode;
}robot_status_t;

extern robot_status_t robot_status;

#endif

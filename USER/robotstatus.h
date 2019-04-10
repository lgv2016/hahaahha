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

	CH_ROTATE,    //自旋
	
	CH_FOLLOW_GIMBAL,  //底盘跟随云台
	
	CH_SPURT,      //冲刺模式
};

//云台初始化状态   初始化成功后会发送数据给云台控制板
enum
{
	NO_INIT,       //未初始化
	INIT_FINISH,    //云台回中初始化成功
	
	INIT_GOOD      //云台imu初始化完成
};

//云台控制板数据
enum
{
	NO_DATA,
	RECEIVED_DATA
	
};

//云台控制模式
enum
{
	MANUAL,     //手动
	AUTO        //自动
};

typedef struct
{
	u8  imu_status;  
    u8  imu_data;    
    u8  control_mode;       
    u8  shoot_mode;
	u8  chassis_mode;
	u8  gimbal_status;
	u8  gimbal_data;
	u8  gimbal_mode;
}robot_status_t;    



extern void BSP_ROBOT_Init(void);
extern robot_status_t robot_status;

#endif

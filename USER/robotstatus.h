#ifndef __ROBOTSTATUS_H
#define __ROBOTSTATUS_H

#include <stm32f4xx.h>


//遥控模式      control_mode
enum
{
	KEY_NO,
	KEY_RC,      //遥控
	KEY_PC       //电脑
};


//射击模式      shoot_mode
enum
{
	SHOOT_NO,
	SHOOT_AWM,     //单发
	SHOOT_AK47,    //连发
	SHOOT_STOP,    //发射停止
	SHOOT_READY,   //发射准备
};

//摩擦轮状态   firc_status
enum
{
	FRIC_ON,        //摩擦轮打开
	FRIC_OFF,       //摩擦轮关闭
	FRIC_ON_START,  //
	FRIC_OFF_START  //
};

//云台模式     gimbal_mode
enum
{
	GIMBAL_NO,   
    GIMBLE_AUTO,       //视觉控制	
	GIMBLE_INIT,       //云台初始化
	GIMBLE_CALI,       //云台校准
	GIMBLE_ERROR,      //云台出错
	GIMBLE_MANUAL      //手动控制
};

//mpu6500状态  mpu6500_status
enum
{
	MPU6500_INIT,     //MPU6500初始化
	MPU6500_SUCCESS   //初始化成功
};


//云台数据     gimbal_data
enum
{
	GIMBAL_MOTOR_NO,
	GIMBAL_MOTOR_GYRO,
	GIMBAL_MOTOR_ENCONDE
};

//云台电机     motor_yaw/motor_pit
enum
{
	MOTOR_GIMBAL_NO,
	MOTOR_GIMBAL_GYRO,
	MOTOR_GIMBAL_ENCODE
	
};


//


//底盘模式    chassis_mode
enum
{
	CHASSIS_NO,
	CHASSIS_ROTATE,
	CHASSIS_STOP,
	CHASSIS_SPEED,
	CHASSIS_FOLLOW_GIMBLE,
	
	
};




typedef struct
{
	u8  mpu6500_status;  
 
    u8  control_mode;    
	
    u8  shoot_mode;
	
	u8  chassis_mode;
	u8  gimbal_mode;
	u8  gimbal_data;
	
	u8  motor_yaw;
	u8  motor_pit;
	u8 firc_status;
	
	
}robot_status_t;    



extern void BSP_ROBOT_Init(void);
extern robot_status_t robot_status;

#endif




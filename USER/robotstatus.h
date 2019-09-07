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
	FRIC_OFF,       //摩擦轮关闭
	FRIC_ON,        //摩擦轮打开
	
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
	GIMBLE_RC,          //手动控制
	GIMBLE_PC, 
	
};

//mpu6500状态  mpu6500_status
enum
{
	MPU6500_NO,
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


//己方装甲颜色状态
enum
{
	COLOR_NO,
	COLOR_RED,
	COLOR_BLUE,
};

//敌方装甲大小信息
enum
{
	ARMOR_BIG,
	ARMOR_SMALL
};


//底盘模式    chassis_mode
enum
{
	CHASSIS_NO,
	CHASSIS_ROTATE,
	CHASSIS_STOP,
	
	CHASSIS_ROTATE_RUN,
	CHASSIS_FOLLOW_GIMBLE,
	CHASSIS_NO_FOLLOW_GIMBLE
	
};


//弹仓状态：

enum
{
	DEPOT_OFF,
	DEPOT_ON
};

//姿态
enum
{
	POSTURE_NORMAL,
	POSTURE_ABNORMAL
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
	u8 fric_status;
	
	u8 enemy_color;
	u8 enemy_armor;
	
	u8 depot_status;
	u8 posture_status;
	
	
}robot_status_t;    



extern void BSP_ROBOT_Init(void);
extern robot_status_t robot_status;

#endif




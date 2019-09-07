#ifndef __ROBOTSTATUS_H
#define __ROBOTSTATUS_H

#include <stm32f4xx.h>


//ң��ģʽ      control_mode
enum
{
	KEY_NO,
	KEY_RC,      //ң��
	KEY_PC       //����
};


//���ģʽ      shoot_mode
enum
{
	SHOOT_NO,
	SHOOT_AWM,     //����
	SHOOT_AK47,    //����
	SHOOT_STOP,    //����ֹͣ
	SHOOT_READY,   //����׼��
};

//Ħ����״̬   firc_status
enum
{
	FRIC_OFF,       //Ħ���ֹر�
	FRIC_ON,        //Ħ���ִ�
	
	FRIC_ON_START,  //
	FRIC_OFF_START  //
};

//��̨ģʽ     gimbal_mode
enum
{
	GIMBAL_NO,   
    GIMBLE_AUTO,       //�Ӿ�����	
	GIMBLE_INIT,       //��̨��ʼ��
	GIMBLE_CALI,       //��̨У׼
	GIMBLE_ERROR,      //��̨����
	GIMBLE_RC,          //�ֶ�����
	GIMBLE_PC, 
	
};

//mpu6500״̬  mpu6500_status
enum
{
	MPU6500_NO,
	MPU6500_INIT,     //MPU6500��ʼ��
	MPU6500_SUCCESS   //��ʼ���ɹ�
};


//��̨����     gimbal_data
enum
{
	GIMBAL_MOTOR_NO,
	GIMBAL_MOTOR_GYRO,
	GIMBAL_MOTOR_ENCONDE
};

//��̨���     motor_yaw/motor_pit
enum
{
	MOTOR_GIMBAL_NO,
	MOTOR_GIMBAL_GYRO,
	MOTOR_GIMBAL_ENCODE
	
};


//����װ����ɫ״̬
enum
{
	COLOR_NO,
	COLOR_RED,
	COLOR_BLUE,
};

//�з�װ�״�С��Ϣ
enum
{
	ARMOR_BIG,
	ARMOR_SMALL
};


//����ģʽ    chassis_mode
enum
{
	CHASSIS_NO,
	CHASSIS_ROTATE,
	CHASSIS_STOP,
	
	CHASSIS_ROTATE_RUN,
	CHASSIS_FOLLOW_GIMBLE,
	CHASSIS_NO_FOLLOW_GIMBLE
	
};


//����״̬��

enum
{
	DEPOT_OFF,
	DEPOT_ON
};

//��̬
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




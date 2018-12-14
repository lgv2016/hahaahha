#ifndef __ROBOTSTATUS_H
#define __ROBOTSTATUS_H

#include <stm32f4xx.h>

//ң��ģʽ  control_mode
enum
{
	USE_RC,      //ң��
	USE_PC       //����
};
//mpu6500״̬   imu_status
enum
{
	NO_CORRECT,         //δ��ʼ
	CORRECT_START,  //��ʼУ׼
	CORRECT_FINISH //У׼���
};
//mpu6500����״̬   imu_data
enum
{
	DATA_FALSE,
	DATA_TRUE
};
//��������ģʽ    shoot_mode
enum
{
	NO_SHOOT,
	RELOAD,        //�
	AWM,
	AK47
};
//����ģʽ    chassis_mode
enum
{
	CH_SPEED,   //�ٶ�
	CH_ANGLE,    //�Ƕ�
	CH_ROTATE    //����
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

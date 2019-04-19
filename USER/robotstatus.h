#ifndef __ROBOTSTATUS_H
#define __ROBOTSTATUS_H

#include <stm32f4xx.h>

//ң��ģʽ  control_mode
enum
{
	NO_CMD,
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
//���ģʽ  shoot_mode
enum
{
	SHOOT_NO,
	SHOOT_AWM,     //����
	SHOOT_AK47,    //����
	SHOOT_STOP,    //����ֹͣ
	SHOOT_READY,   //����׼��
	
	
};
//����ģʽ    chassis_mode
enum
{
	CH_SPEED,   //�ٶ�

	CH_ROTATE,    //����
	
	CH_FOLLOW_GIMBAL,  //���̸�����̨
	
	CH_SPURT,      //���ģʽ
};

//��̨��ʼ��״̬   ��ʼ���ɹ���ᷢ�����ݸ���̨���ư�
enum
{
	NO_INIT,       //δ��ʼ��
	INIT_FINISH,    //��̨���г�ʼ���ɹ�
	
	INIT_GOOD      //��̨imu��ʼ�����
};

//��̨���ư�����
enum
{
	NO_DATA,
	RECEIVED_DATA
	
};

//��̨����ģʽ
enum
{
	MANUAL,     //�ֶ�
	AUTO        //�Զ�
};

//Ħ����״̬
enum
{
	FRIC_OFF,
	
	FRIC_OFF_START,
	FRIC_ON,
	FRIC_ON_START
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
	
	u8 firc_status;
}robot_status_t;    



extern void BSP_ROBOT_Init(void);
extern robot_status_t robot_status;

#endif

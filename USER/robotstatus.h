#ifndef __ROBOTSTATUS_H
#define __ROBOTSTATUS_H

#include <stm32f4xx.h>
//��̨����״̬

//mpu6500״̬
enum
{
	CORRECT,         //δ��ʼ
	CORRECT_START,  //��ʼУ׼
	CORRECT_FINISH //У׼���
};
//mpu6500����״̬
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

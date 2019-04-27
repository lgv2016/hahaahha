#ifndef __DRIVE_USART_H
#define __DRIVE_USART_H

#include <stm32f4xx.h>


extern void MiniPC_Rece_Resolver(void);

extern void MiniPC_Send_Data(u8 cmd);

typedef struct
{
	float get_target_angle_yaw;
	float get_target_angle_pit;
	
	float send_motor_angle_yaw;
	float send_motor_angle_pit;
	
} minipc_data_t;

extern minipc_data_t minipc_data;

#endif


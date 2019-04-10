#ifndef __DRIVE_CHASSIS_H
#define __DRIVE_CHASSIS_H


#include <stm32f4xx.h>

typedef struct
{
	float vx_set;
	float vy_set;
	float wz_set;
	
	float chassis_angle;
	float chassis_angle_set;
	
} chassis_move_t;



extern chassis_move_t chassis_move;
extern u8 CHASSIS_Follow_Gimble_Control(void);
extern void CHASSIS_Move_Control(int16_t vx,int16_t vy);

extern void CHASSIS_Rotate_Control(int16_t rotate_speed);

#endif



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

#endif



#ifndef __DRIVE_CHASSIS_H
#define __DRIVE_CHASSIS_H


#include <stm32f4xx.h>

#define SHOOT_CONTROL_CYCLE 2

typedef struct
{
	float vx_set;
	float vy_set;
	float wz_set;
	
	
	u16 key_q_time;
	u16 key_shift_time;
	u16 key_ctrl_time;
	
	float chassis_angle;
	float chassis_angle_set;
	u8    inc_cal_flag;
	
	u8    chassis_rotate_flag;
	
	float rc_control_speedx;
	float rc_control_speedy;
	
} chassis_move_t;



extern chassis_move_t chassis_move;

extern void CHASSIS_Loop_Control(void);

extern void CHASSIS_Init(void);

extern void CHASSIS_RC_Control(int16_t maxspeedx,int16_t maxspeedy);


#endif



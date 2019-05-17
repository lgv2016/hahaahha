#ifndef __DRIVE_CHASSIS_H
#define __DRIVE_CHASSIS_H


#include <stm32f4xx.h>


typedef struct
{

	
	
	u16 key_q_time;
	u16 key_shift_time;
	u16 key_ctrl_time;

	u8    inc_cal_flag;
	
	u8    chassis_rotate_flag;
	u8    chassis_follow_flag;
	
	int16_t x;
	int16_t y;
	int16_t w;
	
	int16_t add;
	
	float rc_control_speedx;
	float rc_control_speedy;
	
	float rotate_speed;
	
	float rotate_speed_set;
	float rotate_angle;
	
	u8 chassis_last_mode;
	
	u8 last_w;
	
} chassis_move_t;



extern chassis_move_t chassis_move;

extern void CHASSIS_Loop_Control(void);

extern void CHASSIS_Init(void);

extern void CHASSIS_RC_Control(int16_t maxspeedx,int16_t maxspeedy);

extern void Get_CHASSIS_data(CanRxMsg rx_message);


#endif



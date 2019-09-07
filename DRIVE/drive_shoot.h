#ifndef __DRIVE_SHOOT_H
#define __DRIVE_SHOOT_H


#include <stm32f4xx.h>
#include <stdbool.h>

#define SHOOT_CONTROL_CYCLE 5      //5ms


typedef struct
{
    bool press_l;
    bool last_press_l;
	
	bool press_r;
	bool last_press_r;
	
//	bool last_key_v;
//	bool last_key_b;
	
    uint16_t press_l_time;
    uint16_t no_press_l_time;
	uint16_t key_time;
	
	uint16_t key_q_time;
	
	uint16_t no_key_q_time;
	
	bool last_key_q;
	bool rest_flag;
	
	float speed;
	
	u8 shoot_flag;
	
} shoot_motor_t;


typedef struct
{
	float input;
	float out;
	float max;
	float min;
	float period;
} fric_motor_t;




extern void SHOOT_Loop_Control(void);
extern void SHOOT_Init(void);

extern void GET_Fric_Status(CanRxMsg rx_message);



extern void Fric_Motor_Speed_Init(void);
extern void Fric_Motor_Speed_Control(void);


#endif


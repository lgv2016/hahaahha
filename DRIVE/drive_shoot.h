#ifndef __DRIVE_SHOOT_H
#define __DRIVE_SHOOT_H


#include <stm32f4xx.h>
#include <stdbool.h>

#define SHOOT_CONTROL_CYCLE 1      //5ms

typedef struct
{
    bool press_l;
    bool last_press_l;
    uint16_t press_l_time;
    uint16_t no_press_l_time;
	uint16_t key_time;
	
	bool rest_flag;
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


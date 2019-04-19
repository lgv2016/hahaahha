#ifndef __DRIVE_SHOOT_H
#define __DRIVE_SHOOT_H


#include <stm32f4xx.h>

#include <stdbool.h>

#define SHOOT_CONTROL_CYCLE 5       //5ms
extern void SHOOT_Loop_Control(void);
extern void SHOOT_Init(void);

extern void GET_209_Data(CanRxMsg rx_message);
typedef struct
{
    bool press_l;
    bool last_press_l;
    uint16_t press_l_time;
    uint16_t no_press_l_time;
	uint16_t key_time;
	
	bool rest_flag;
} shoot_motor_t;



#endif


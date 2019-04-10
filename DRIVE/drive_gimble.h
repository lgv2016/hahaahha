#ifndef __DRIVE_GIMBLE_H

#define __DRIVE_GIMBLE_H


#include <stm32f4xx.h>

typedef struct
{
	float input;
	float out;
	float max;
	float min;
	float period;
} fric_t;

extern fric_t fric1;
extern fric_t fric2;

extern u8 auto_flag;

extern void GIMBLE_Control(void);

#endif

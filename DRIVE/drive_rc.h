#ifndef __DRIVE_RC_H
#define __DRIVE_RC_H

#include <stm32f4xx.h>

enum
{
    W,
    S,
    A,
    D,
    Q,
    E,
    SHIFT,
    CTRL,
    KEYNUM
};



typedef struct
{
	struct
	{
		u16 ch0;
		u16 ch1;
		u16 ch2;
		u16 ch3;
		u8  s1;
		u8  s2;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		u8      press_l;
		u8      press_r;
	}mouse;
	struct
	{
	    u16 v;
        u8  k[KEYNUM];  
	}key;
}rc_control_t;

extern rc_control_t g_rc_control;
extern void RC_Data_Parse(void);

#endif

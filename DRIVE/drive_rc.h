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
		
		float   x_distance;
		float   y_distance;
		
	}mouse;
	struct
	{
	    u16 v;
        u8  k[KEYNUM];  
	}key;
}rc_control_t;


typedef struct
{
	u8 lastkey;
	u8 key;
	
	int16_t time;
	int16_t rate;
	
	u8 input;
	u8 output;
	
	
}key_t;


extern rc_control_t g_rc_control;
extern void RC_Data_Parse(void);

extern void CH_Speed_Control(u16 xspeedmax,u16 yspeedmax);
extern void CH_Angle_Control(float yawmax,float pitchmax);

extern void PC_Angle_Control(void);

extern float g_pit_target;
extern float g_yaw_target;

#endif

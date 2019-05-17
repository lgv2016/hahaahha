#ifndef __DRIVE_GIMBLE_H
#define __DRIVE_GIMBLE_H
#include <stm32f4xx.h>




#define GIMBLE_CONTROL_CYCLE 1

#define YAW_INIT_ANGLE 180.0f
#define PIT_INIT_ANGLE 51.0f


typedef struct
{
	u8 angle_init_time;

	float rc_control_angle;
	float pc_control_angle;
	float vision_control_angle;
	
	float last_rc_control_angle;
	float last_pc_control_angle;
	float last_vision_control_angle;
	
	float angle;
	
	u16 no_control_time;
	
	u16   press_r_time;
} Gimbal_Motor_t;

extern void GIMBLE_Init(void);
extern void Get_GIMBLE_data(CanRxMsg rx_message);
extern void GIMBLE_Loop_Control(void);

extern void GET_GIMBLE_Status(CanRxMsg rx_message);

extern void GIMBLE_Set_Mode(void);




#endif

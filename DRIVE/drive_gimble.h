#ifndef __DRIVE_GIMBLE_H
#define __DRIVE_GIMBLE_H
#include <stm32f4xx.h>



#define GIMBLE_CONTROL_CYCLE 1



typedef struct
{

	
	u8 angle_init_time;
	
	float rc_control_angle;
	float pc_control_angle;
	float vision_control_angle;
	
	u16   press_r_time;

} Gimbal_Motor_t;

extern void GIMBLE_Init(void);
extern void Get_GIMBLE_data(CanRxMsg rx_message);
extern void GIMBLE_Loop_Control(void);




#endif

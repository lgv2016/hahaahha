#ifndef __DRIVE_JUDGE_H
#define __DRIVE_JUDGE_H

#include <stm32f4xx.h>

typedef struct
{
  float power;
	u16  power_buffer;
  u16   shoot_rate;
  u8    robot_level;
  u8 robot_id;
  
  u16   shooter_heat0;
	
	float max_power;
	


} judge_data_t;

extern judge_data_t judge_data;


extern void Get_Judge_data(CanRxMsg rx_message);

extern void Cmd_Judge_ESC(void);
#endif








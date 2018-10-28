#ifndef __BSP_MOTOR_CRADLE_HEAD_H
#define __BSP_MOTOR_CRADLE_HEAD_H

#include <stm32f4xx.h>

typedef struct
{
    float    speed[2];
    
    u16      angle[2];
    
    int16_t  set_current[2];
    
    int16_t  actual_current[2];
    
}data_6623_t;



extern data_6623_t g_data_6623;


extern void Cmd_6623_ESC(int16_t  current_205,int16_t current_206);
extern void Get_6623_data(CanRxMsg rx_message);
extern void Get_6623_Speed(data_6623_t g_data_6623);//бу/s


#endif  

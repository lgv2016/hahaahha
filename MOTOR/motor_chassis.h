#ifndef __MOTOR_CHASSIS_H
#define __MOTOR_CHASSIS_H

#include <stm32f4xx.h>

enum
{
    LF,    //×óÇ°
    LA,
    RF,
    RA,    //ÓÒºó
    NUM_3510
};



typedef struct
{
    float    speed[NUM_3510];
    u16      angle[NUM_3510];
}data_3510_t;



extern data_3510_t g_data_3510;


void Get_3510_data(CanRxMsg rx_message);
extern void Cmd_3510_ESC(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204);




#endif  

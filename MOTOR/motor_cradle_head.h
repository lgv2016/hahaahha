#ifndef __BSP_MOTOR_CRADLE_HEAD_H
#define __BSP_MOTOR_CRADLE_HEAD_H

#include <stm32f4xx.h>
enum
{
    YAW,
    PITCH,
    NUM_6623
};

typedef struct
{
    
    float    angle[NUM_6623];
    u16      pre_angle[NUM_6623];
	
    int16_t  speed[NUM_6623];
    int16_t  set_current[NUM_6623];
    int16_t  actual_current[NUM_6623];
    
}data_6623_t;


typedef struct
{
    u16        pre_angle;
    u16        last_angle;
    u16        offset_angle;
    int32_t    total_angle;
    int32_t    count;
    
    float      angle;
    
    int16_t    speed;
    int16_t    torque;
    
}data_2006_t;



extern data_6623_t g_data_6623;
extern data_2006_t g_data_2006;



extern void Cmd_6623_ESC(int16_t  current_205,int16_t current_206);
extern void Get_6623_data(CanRxMsg rx_message);




extern void Cmd_2006_ESC(int16_t  current_207);
extern void Get_2006_data(CanRxMsg rx_message);
extern void Get_2006_Offset_angle(CanRxMsg rx_message);

extern void Cmd_YUNTAI_ESC(u8 current_208);
extern void Get_YUNTAI_Data(CanRxMsg rx_message);

extern void Snail_Calibration(void);
extern void Snail_Stop(void);
extern void Snail_Stat(void);




#endif  

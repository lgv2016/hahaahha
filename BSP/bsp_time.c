#include <bsp_time.h>

void BSP_TIME_Init()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    
    
    //TIM6
    TIM_TimeBase_InitStructure.TIM_Period           =   10-1;
    TIM_TimeBase_InitStructure.TIM_Prescaler        =   9000-1;
    TIM_TimeBaseInit(TIM6,&TIM_TimeBase_InitStructure);
    TIM_Cmd(TIM6, ENABLE);
    
    
    //TIM5
    TIM_TimeBase_InitStructure.TIM_ClockDivision    =   TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_CounterMode      =   TIM_CounterMode_Up;
    TIM_TimeBase_InitStructure.TIM_Period           =   20000-1;
    TIM_TimeBase_InitStructure.TIM_Prescaler        =   90-1;
    TIM_TimeBaseInit(TIM5,&TIM_TimeBase_InitStructure);
    
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM5,ENABLE);
    TIM_Cmd(TIM5, DISABLE);	
  
}
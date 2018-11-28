#include <bsp_time.h>

void BSP_TIME_Init()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
  
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
    
    //TIM6  10ms
    TIM_TimeBase_InitStructure.TIM_Period           =   10000-1;
    TIM_TimeBase_InitStructure.TIM_Prescaler        =   90-1;
    TIM_TimeBaseInit(TIM6,&TIM_TimeBase_InitStructure);
    TIM_Cmd(TIM6, ENABLE);
    

    //TIM5 OC4  OC3
    TIM_TimeBase_InitStructure.TIM_ClockDivision    =   TIM_CKD_DIV1;
    TIM_TimeBase_InitStructure.TIM_CounterMode      =   TIM_CounterMode_Up;
    TIM_TimeBase_InitStructure.TIM_Period           =   20000-1;
    TIM_TimeBase_InitStructure.TIM_Prescaler        =   90-1;
    TIM_TimeBaseInit(TIM5,&TIM_TimeBase_InitStructure);
    
    TIM_OCInitStructure.TIM_OCMode                  =   TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity              =   TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse                   =   0;
    TIM_OCInitStructure.TIM_OutputState             =   TIM_OutputState_Enable;
    
    TIM_OC4Init(TIM5,&TIM_OCInitStructure);
    TIM_OC3Init(TIM5,&TIM_OCInitStructure);
    
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM5,ENABLE);	
    TIM_Cmd(TIM5, DISABLE);
    
}

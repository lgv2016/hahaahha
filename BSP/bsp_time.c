#include <bsp_time.h>

void BSP_TIME_Init()
{
    TIM_TimeBaseInitTypeDef TIM_TimeBase_InitStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    TIM_ICInitTypeDef       TIM_ICInitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
    
    //TIM6  10ms
    TIM_TimeBase_InitStructure.TIM_Period           =   10000-1;
    TIM_TimeBase_InitStructure.TIM_Prescaler        =   90-1;
    TIM_TimeBaseInit(TIM6,&TIM_TimeBase_InitStructure);
    TIM_Cmd(TIM6, ENABLE);
    

    
    //TIM5 OC4
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
    
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM5,ENABLE);	
    
    //TIM5 IC3
	TIM_ICInitStructure.TIM_Channel                 =   TIM_Channel_3; 
    TIM_ICInitStructure.TIM_ICPolarity              =   TIM_ICPolarity_Rising;	
    TIM_ICInitStructure.TIM_ICSelection             =   TIM_ICSelection_DirectTI; 
    TIM_ICInitStructure.TIM_ICPrescaler             =   TIM_ICPSC_DIV1;	 
    TIM_ICInitStructure.TIM_ICFilter                =   0x00;
    TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
    TIM_Cmd(TIM5,ENABLE );
    
}
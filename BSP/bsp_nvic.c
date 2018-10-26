#include <bsp_nvic.h>

void BSP_NVIC_Init()
{
    
    NVIC_InitTypeDef  NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    //USART1 DBUS DMA接收
    NVIC_InitStructure.NVIC_IRQChannel                    = DMA2_Stream2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  =2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         =2;		
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
    DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);
    
    //UASRT6 JUDGE USART空闲中断
    NVIC_InitStructure.NVIC_IRQChannel                    = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority  = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority         = 2;		
	NVIC_InitStructure.NVIC_IRQChannelCmd                 = ENABLE;			
	NVIC_Init(&NVIC_InitStructure);	
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
    
    //USART6 JUDGE DMA接收
    NVIC_InitStructure.NVIC_IRQChannel                     = DMA2_Stream7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                  = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority   = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority          = 2;
    NVIC_Init(&NVIC_InitStructure);
    DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
    DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
    
}
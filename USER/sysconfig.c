#include <sysconfig.h>

void SYS_Config_Init()
{
	BSP_DWT_Init();
	
    BSP_GPIO_Init();
    BSP_USART_Init();
    
    BSP_CAN_Init();
    BSP_DMA_Init();
    
    BSP_TIME_Init();
    BSP_NVIC_Init();
	
	BSP_ROBOT_Init();		        
    Infan_Control_Init();
	

}

int fputc(int ch, FILE *f)
{
    USART_SendData(USART6, (uint8_t) ch);
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);		
	return (ch);
}

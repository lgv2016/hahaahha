#include <bsp_usart.h>

void BSP_USART_Init()
{
    
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE);
	
	
    //USART1  遥控
    USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate            = 100000;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_Even;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx;	
    USART_Init(USART1, &USART_InitStructure);
    
    USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    USART_Cmd(USART1, ENABLE); 
    
    //USART6 裁判系统
    USART_DeInit(USART6);
    USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;	
    USART_Init(USART6, &USART_InitStructure); 
	 
    USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);
    USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
    USART_Cmd(USART6, ENABLE);
	
	
	 //USART8 妙算通信接口
    USART_DeInit(UART8);
    USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;	
    USART_Init(UART8, &USART_InitStructure); 
	 
    USART_DMACmd(UART8,USART_DMAReq_Rx,ENABLE);
    USART_DMACmd(UART8,USART_DMAReq_Tx,ENABLE);
    USART_Cmd(UART8, ENABLE);
}

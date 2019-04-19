#include <bsp_gpio.h>
#include <bsp_iic.h>
#include <drive_delay.h>
void  BSP_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使用外部中断
    
    //IMU 模拟IIC    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOF, GPIO_Pin_6); //CS=1
	GPIO_SetBits(GPIOF, GPIO_Pin_9); //IIC_SDA=1
	GPIO_SetBits(GPIOF, GPIO_Pin_7);//IIC_SCL=1
	GPIO_ResetBits(GPIOF, GPIO_Pin_8); //AD=0
	
	

	//MPU6500 INT
	//PB8
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB,EXTI_PinSource8);
	
	
	
    
    //LED   
    //PG1
    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOG, GPIO_Pin_1);     //关闭
	
	//激光
	GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_SetBits(GPIOG, GPIO_Pin_13);     //关闭
	
    
    //TIM5 OC4  OC3
    //PI0  PH12
    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_12|GPIO_Pin_10|GPIO_Pin_11;
    GPIO_Init(GPIOH, &GPIO_InitStructure);
    
    
    GPIO_PinAFConfig(GPIOI,GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5);
    
    //USART1 
    //RX:PB7
    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource7, GPIO_AF_USART1); 
    
    //USART6
    //TX:PG14 RX:PG9
    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_9 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOG,GPIO_PinSource9, GPIO_AF_USART6); 
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
	
	 //USART8
    //TX:PE1 RX:PE0
    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource0, GPIO_AF_UART8); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource1, GPIO_AF_UART8);
    
    //CAN1
    //H:PD1 L:PD0
    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_0 | GPIO_Pin_1; 
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1);
    
    //CAN2
    //H:PB13 L:PB12
    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2);
	
	
	//TIM3 OC2  
    //PB5
    GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource5, GPIO_AF_TIM3);
	//激光
	GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
    GPIO_SetBits(GPIOG, GPIO_Pin_13);    
	
	//4个24V电源控制口
	
    GPIO_InitStructure.GPIO_Pin    = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode   = GPIO_Mode_OUT;      
    GPIO_InitStructure.GPIO_OType  = GPIO_OType_PP;    
    GPIO_InitStructure.GPIO_Speed  = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_PuPd   = GPIO_PuPd_UP;       
    GPIO_Init(GPIOH, &GPIO_InitStructure); 
	
	GPIO_ResetBits(GPIOH,GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
	Delay_US(1000);
	GPIO_SetBits(GPIOH,GPIO_Pin_2);
    Delay_US(1000);
	GPIO_SetBits(GPIOH,GPIO_Pin_3);
	Delay_US(1000);
	GPIO_SetBits(GPIOH,GPIO_Pin_4);
	Delay_US(1000);
	GPIO_SetBits(GPIOH,GPIO_Pin_5);
}

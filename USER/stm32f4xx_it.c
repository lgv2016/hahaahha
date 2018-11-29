/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"


#include <sysconfig.h>


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
// 
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/************************************以下为添加的静态变量**************************************************/


/************************************以下为添加的全局变量**************************************************/



/************************************ 以下为添加的中断函数 ***********************************************/

void ENABLE_DMA2_Stream7_Tx(u16 DMA_Send_Buff_Size)
{
	
	DMA_Cmd(DMA2_Stream7, DISABLE);
	while (DMA_GetCmdStatus(DMA2_Stream7)); 
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7|DMA_FLAG_HTIF7|DMA_FLAG_TEIF7|DMA_FLAG_DMEIF7|DMA_FLAG_FEIF7);                                                                                                 
	DMA_ClearITPendingBit(DMA2_Stream7,DMA_IT_TEIF7|DMA_IT_HTIF7|DMA_IT_TCIF7|DMA_IT_DMEIF7|DMA_IT_FEIF7);
	DMA_SetCurrDataCounter(DMA2_Stream7,DMA_Send_Buff_Size);
	DMA_Cmd(DMA2_Stream7, ENABLE);
	
}
//发送
void DMA2_Stream7_IRQHandler(void)  
{  
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7)!=RESET)
    {   
        DMA_ClearITPendingBit(DMA2_Stream7, DMA_IT_TCIF7);
        DMA_Cmd(DMA2_Stream7, DISABLE); 
    }  

} 
//接收
void USART6_IRQHandler()
{
	u16 RxCounter;
	u8 clear;
	u16 i=0;
	if(USART_GetITStatus(USART6,USART_IT_IDLE)!=RESET)
	{
		DMA_Cmd(DMA2_Stream1, DISABLE); 
		clear=USART6->SR;
		clear=USART6->DR;
		RxCounter=DMA_Judge_Reve_Buff_Size-DMA_GetCurrDataCounter(DMA2_Stream1);
		
     while(RxCounter--)
	 {
         g_DMA_Judge_Send_Buff[i]=g_DMA_Judge_Reve_Buff[i];
         i++;
     }
     ENABLE_DMA2_Stream7_Tx(i);

    while (DMA_GetCmdStatus(DMA2_Stream1));   

    DMA_SetCurrDataCounter(DMA2_Stream1,DMA_Judge_Reve_Buff_Size);
    DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1|DMA_FLAG_HTIF1|DMA_FLAG_TEIF1|DMA_FLAG_DMEIF1|DMA_FLAG_FEIF1); 
    DMA_Cmd(DMA2_Stream1, ENABLE); 
  }
}



void TIM6_DAC_IRQHandler()
{

    if(TIM_GetITStatus( TIM6,TIM_IT_Update)!= RESET )
    {

		Speed_2006_Control(g_speed_target);  
	    Speed_3510_Control(g_speed_target);
		
        TIM_ClearITPendingBit(TIM6 , TIM_IT_Update);
    }
}


void TIM5_IRQHandler()
{
    
    if(TIM_GetITStatus(TIM5,TIM_IT_CC3) != RESET)
    {
        
        
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
    }
}

void CAN1_RX0_IRQHandler(void)
{
    static u8 flag=1;
    CanRxMsg rx_message;
  if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=RESET)
  {
    
      CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
      if(flag)
      {
          Get_2006_Offset_angle(rx_message);
          flag=0;
      }
      
      Get_6623_data(rx_message);
      Get_2006_data(rx_message);
      Get_3510_data(rx_message);
      
      CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
  }
}

void DMA2_Stream2_IRQHandler(void)
{
    
  if(DMA_GetITStatus(DMA2_Stream2, DMA_IT_TCIF2))
   { 
      DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
      DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
     //遥控协议解析
      RC_Data_Parse();

    }
 
}




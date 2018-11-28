#include <bsp_can.h>

void BSP_CAN_Init()
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_Filter_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
    
    CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
  
    CAN_InitStructure.CAN_ABOM           = DISABLE;                    //自动离线管理
    CAN_InitStructure.CAN_AWUM           = ENABLE;                    //关闭自动唤醒模式
    CAN_InitStructure.CAN_BS1            = CAN_BS1_5tq;	               //配置CAN Baudrate = 1 MBps   45/(1+5+3)/5=1 Mbps
    CAN_InitStructure.CAN_BS2            = CAN_BS2_3tq;	
    CAN_InitStructure.CAN_Mode           = CAN_Mode_Normal;           //正常工作模式
    CAN_InitStructure.CAN_NART           = DISABLE;                    //禁止报文自动重传
    CAN_InitStructure.CAN_Prescaler      = 5;
    CAN_InitStructure.CAN_RFLM           = DISABLE;                    //禁止接收FIFO锁定模式 溢出时新报文会覆盖原有报文 
    CAN_InitStructure.CAN_SJW            = CAN_SJW_2tq;		             //BTR-SJW 重新同步跳跃宽度 2个时间单元
    CAN_InitStructure.CAN_TTCM           = DISABLE;                    //关闭时间触发通信模式使能
    CAN_InitStructure.CAN_TXFP           = DISABLE;                    //发送F报文优先级取决于顺序
    CAN_Init(CAN1,&CAN_InitStructure);
  
    CAN_Filter_InitStructure.CAN_FilterFIFOAssignment       = CAN_Filter_FIFO0;  //报文会被存储到接收FIF0 0
    CAN_Filter_InitStructure.CAN_FilterIdHigh               = 0X00;            //接收任何ID的报文，不过滤
    CAN_Filter_InitStructure.CAN_FilterIdLow                = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMaskIdHigh           = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMaskIdLow            = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMode                 = CAN_FilterMode_IdMask;  //掩码模式
    CAN_Filter_InitStructure.CAN_FilterNumber               = 0;                      //过滤器0
    CAN_Filter_InitStructure.CAN_FilterScale                = CAN_FilterScale_16bit;  
    CAN_Filter_InitStructure.CAN_FilterActivation           = ENABLE;
    CAN_FilterInit(&CAN_Filter_InitStructure);
    
    
    CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);
  
    CAN_InitStructure.CAN_ABOM           = DISABLE;                    //自动离线管理
    CAN_InitStructure.CAN_AWUM           = ENABLE;                    //自动唤醒模式//是的
    CAN_InitStructure.CAN_BS1            = CAN_BS1_5tq;	               //配置CAN Baudrate = 1 MBps   45/(1+5+3)/5=1 Mbps
    CAN_InitStructure.CAN_BS2            = CAN_BS2_3tq;	
    CAN_InitStructure.CAN_Mode           = CAN_Mode_Normal;           //正常工作模式
    CAN_InitStructure.CAN_NART           = DISABLE;                    //禁止报文自动重传
    CAN_InitStructure.CAN_Prescaler      = 5;
    CAN_InitStructure.CAN_RFLM           = DISABLE;                    //禁止接收FIFO锁定模式 溢出时新报文会覆盖原有报文 
    CAN_InitStructure.CAN_SJW            = CAN_SJW_2tq;		             //BTR-SJW 重新同步跳跃宽度 2个时间单元
    CAN_InitStructure.CAN_TTCM           = DISABLE;                    //关闭时间触发通信模式使能
    CAN_InitStructure.CAN_TXFP           = DISABLE;                    //发送F报文优先级取决于顺序
    CAN_Init(CAN2,&CAN_InitStructure);
  
     
    CAN_Filter_InitStructure.CAN_FilterFIFOAssignment       = CAN_Filter_FIFO1;  //报文会被存储到接收FIF0 1
    CAN_Filter_InitStructure.CAN_FilterIdHigh               = 0X00;            //接收任何ID的报文，不过滤
    CAN_Filter_InitStructure.CAN_FilterIdLow                = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMaskIdHigh           = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMaskIdLow            = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMode                 = CAN_FilterMode_IdMask;  //掩码模式
    CAN_Filter_InitStructure.CAN_FilterNumber               = 14;                      //过滤器14
    CAN_Filter_InitStructure.CAN_FilterScale                = CAN_FilterScale_16bit;  //32位
    CAN_Filter_InitStructure.CAN_FilterActivation           = ENABLE;
    CAN_FilterInit(&CAN_Filter_InitStructure);
    
}
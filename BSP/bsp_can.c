#include <bsp_can.h>

void BSP_CAN_Init()
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_Filter_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
    
    CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
  
    CAN_InitStructure.CAN_ABOM           = DISABLE;                    //�Զ����߹���
    CAN_InitStructure.CAN_AWUM           = ENABLE;                    //�ر��Զ�����ģʽ
    CAN_InitStructure.CAN_BS1            = CAN_BS1_5tq;	               //����CAN Baudrate = 1 MBps   45/(1+5+3)/5=1 Mbps
    CAN_InitStructure.CAN_BS2            = CAN_BS2_3tq;	
    CAN_InitStructure.CAN_Mode           = CAN_Mode_Normal;           //��������ģʽ
    CAN_InitStructure.CAN_NART           = DISABLE;                    //��ֹ�����Զ��ش�
    CAN_InitStructure.CAN_Prescaler      = 5;
    CAN_InitStructure.CAN_RFLM           = DISABLE;                    //��ֹ����FIFO����ģʽ ���ʱ�±��ĻḲ��ԭ�б��� 
    CAN_InitStructure.CAN_SJW            = CAN_SJW_2tq;		             //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_TTCM           = DISABLE;                    //�ر�ʱ�䴥��ͨ��ģʽʹ��
    CAN_InitStructure.CAN_TXFP           = DISABLE;                    //����F�������ȼ�ȡ����˳��
    CAN_Init(CAN1,&CAN_InitStructure);
  
    CAN_Filter_InitStructure.CAN_FilterFIFOAssignment       = CAN_Filter_FIFO0;  //���Ļᱻ�洢������FIF0 0
    CAN_Filter_InitStructure.CAN_FilterIdHigh               = 0X00;            //�����κ�ID�ı��ģ�������
    CAN_Filter_InitStructure.CAN_FilterIdLow                = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMaskIdHigh           = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMaskIdLow            = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMode                 = CAN_FilterMode_IdMask;  //����ģʽ
    CAN_Filter_InitStructure.CAN_FilterNumber               = 0;                      //������0
    CAN_Filter_InitStructure.CAN_FilterScale                = CAN_FilterScale_16bit;  
    CAN_Filter_InitStructure.CAN_FilterActivation           = ENABLE;
    CAN_FilterInit(&CAN_Filter_InitStructure);
    
    
    CAN_DeInit(CAN2);
	CAN_StructInit(&CAN_InitStructure);
  
    CAN_InitStructure.CAN_ABOM           = DISABLE;                    //�Զ����߹���
    CAN_InitStructure.CAN_AWUM           = ENABLE;                    //�Զ�����ģʽ//�ǵ�
    CAN_InitStructure.CAN_BS1            = CAN_BS1_5tq;	               //����CAN Baudrate = 1 MBps   45/(1+5+3)/5=1 Mbps
    CAN_InitStructure.CAN_BS2            = CAN_BS2_3tq;	
    CAN_InitStructure.CAN_Mode           = CAN_Mode_Normal;           //��������ģʽ
    CAN_InitStructure.CAN_NART           = DISABLE;                    //��ֹ�����Զ��ش�
    CAN_InitStructure.CAN_Prescaler      = 5;
    CAN_InitStructure.CAN_RFLM           = DISABLE;                    //��ֹ����FIFO����ģʽ ���ʱ�±��ĻḲ��ԭ�б��� 
    CAN_InitStructure.CAN_SJW            = CAN_SJW_2tq;		             //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
    CAN_InitStructure.CAN_TTCM           = DISABLE;                    //�ر�ʱ�䴥��ͨ��ģʽʹ��
    CAN_InitStructure.CAN_TXFP           = DISABLE;                    //����F�������ȼ�ȡ����˳��
    CAN_Init(CAN2,&CAN_InitStructure);
  
     
    CAN_Filter_InitStructure.CAN_FilterFIFOAssignment       = CAN_Filter_FIFO1;  //���Ļᱻ�洢������FIF0 1
    CAN_Filter_InitStructure.CAN_FilterIdHigh               = 0X00;            //�����κ�ID�ı��ģ�������
    CAN_Filter_InitStructure.CAN_FilterIdLow                = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMaskIdHigh           = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMaskIdLow            = 0X00;
    CAN_Filter_InitStructure.CAN_FilterMode                 = CAN_FilterMode_IdMask;  //����ģʽ
    CAN_Filter_InitStructure.CAN_FilterNumber               = 14;                      //������14
    CAN_Filter_InitStructure.CAN_FilterScale                = CAN_FilterScale_16bit;  //32λ
    CAN_Filter_InitStructure.CAN_FilterActivation           = ENABLE;
    CAN_FilterInit(&CAN_Filter_InitStructure);
    
}
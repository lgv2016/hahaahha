#include <bsp_dma.h>

u8  g_DMA_Dbus_Buff[DMA_Dbus_Buff_Size];
u8  g_DMA_Judge_Send_Buff[DMA_Judge_Send_Buff_Size];
u8  g_DMA_Judge_Reve_Buff[DMA_Judge_Reve_Buff_Size];

void BSP_DMA_Init()
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
    
    //USART1_RX  DMA2 数据流2通道4
    DMA_DeInit(DMA2_Stream2);      
    while (DMA_GetCmdStatus(DMA2_Stream2));
    DMA_InitStructure.DMA_BufferSize              = DMA_Dbus_Buff_Size;
    DMA_InitStructure.DMA_Channel                 = DMA_Channel_4;
    DMA_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralToMemory ;
    DMA_InitStructure.DMA_FIFOMode                = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold           = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_Memory0BaseAddr         = (u32)g_DMA_Dbus_Buff;
    DMA_InitStructure.DMA_MemoryBurst             = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode                    = DMA_Mode_Circular;
    DMA_InitStructure.DMA_PeripheralBaseAddr      = (u32)&(USART1->DR);
    DMA_InitStructure.DMA_PeripheralBurst         = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority                = DMA_Priority_Medium;
    DMA_Init(DMA2_Stream2,&DMA_InitStructure);
    DMA_Cmd(DMA2_Stream2, ENABLE);
    
    //USART6_TX  DMA2 数据流7通道5
    DMA_DeInit(DMA2_Stream7);    
    while (DMA_GetCmdStatus(DMA2_Stream7));
    DMA_InitStructure.DMA_Channel                 = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr      = (u32)&USART6->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr         = (u32)g_DMA_Judge_Reve_Buff;
    DMA_InitStructure.DMA_DIR                     = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize              = DMA_Judge_Reve_Buff_Size;
    DMA_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode                    = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority                = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode                = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold           = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst             = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst         = DMA_PeripheralBurst_Single; 
    DMA_Init(DMA2_Stream7,&DMA_InitStructure);
 //   DMA_Cmd(DMA2_Stream7, ENABLE);
  
    //USART6_RX  DMA2 数据流1通道5
    DMA_DeInit(DMA2_Stream1);       
    while (DMA_GetCmdStatus(DMA2_Stream1));
    DMA_InitStructure.DMA_BufferSize              = DMA_Judge_Send_Buff_Size;
    DMA_InitStructure.DMA_Channel                 = DMA_Channel_5;
    DMA_InitStructure.DMA_DIR                     = DMA_DIR_PeripheralToMemory ;
    DMA_InitStructure.DMA_FIFOMode                = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold           = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_Memory0BaseAddr         = (u32)g_DMA_Judge_Send_Buff;
    DMA_InitStructure.DMA_MemoryBurst             = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_MemoryDataSize          = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_MemoryInc               = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Mode                    = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBaseAddr      = (u32)&(USART6->DR);
    DMA_InitStructure.DMA_PeripheralBurst         = DMA_PeripheralBurst_Single;
    DMA_InitStructure.DMA_PeripheralDataSize      = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralInc           = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_Priority                = DMA_Priority_Medium;
    DMA_Init(DMA2_Stream1,&DMA_InitStructure);
    DMA_Cmd(DMA2_Stream1, ENABLE);
    
}
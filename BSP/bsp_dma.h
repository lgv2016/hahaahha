#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include <stm32f4xx.h>

#define DMA_Dbus_Buff_Size 18
#define DMA_Judge_Send_Buff_Size 255
#define DMA_Judge_Reve_Buff_Size 255


#define DMA_MiniPC_Send_Buff_Size 30
#define DMA_MiniPC_Reve_Buff_Size 30

extern u8 g_DMA_Dbus_Buff[DMA_Dbus_Buff_Size];
extern u8 g_DMA_Judge_Send_Buff[DMA_Judge_Send_Buff_Size];
extern u8 g_DMA_Judge_Reve_Buff[DMA_Judge_Reve_Buff_Size];
extern u8 g_DMA_MiniPC_Reve_Buff[DMA_MiniPC_Reve_Buff_Size];
extern u8 g_DMA_MiniPC_Send_Buff[DMA_MiniPC_Send_Buff_Size];

extern void BSP_DMA_Init(void);
#endif  

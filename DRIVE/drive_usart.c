#include <drive_usart.h>
#include <math_tool.h>
#include <bsp_dma.h>

#include <drive_control.h>
#include <motor_cradle_head.h>

#include <drive_imu.h>

minipc_data_t minipc_data;

//解算妙算的数据
void MiniPC_Rece_Resolver()
{
	if(g_DMA_MiniPC_Reve_Buff[0]==0XA5)
	{
		if(g_DMA_MiniPC_Reve_Buff[1]==0X03)
		{
			if(Verify_Check_SUM(g_DMA_MiniPC_Reve_Buff,11))
			{
				minipc_data.get_target_angle_yaw   = HEX_TO_Float(&g_DMA_MiniPC_Reve_Buff[2]);
				minipc_data.get_target_angle_pit   = HEX_TO_Float(&g_DMA_MiniPC_Reve_Buff[6]);
			}
		}
		
	}
}

void MiniPC_Send_Data(u8 cmd)
{
	u8 dwLength;
	if(cmd==0X03)   //发送云台角度数据
	{
		dwLength=11;
		g_DMA_MiniPC_Send_Buff[0]=0XA6;
		g_DMA_MiniPC_Send_Buff[1]=cmd;
		
		
		FLOAT_TO_Hex(&g_DMA_MiniPC_Send_Buff[2] ,   g_imu_data.absolute_yaw);
		FLOAT_TO_Hex(&g_DMA_MiniPC_Send_Buff[2+4] ,g_data_6623.angle[PITCH]);
		
		
		Append_Check_SUM(g_DMA_MiniPC_Send_Buff,2+4+4+1);
	}
	DMA_Cmd(DMA1_Stream0, DISABLE);
	while (DMA_GetCmdStatus(DMA1_Stream0)); 
	DMA_ClearFlag(DMA1_Stream0,DMA_FLAG_TCIF0|DMA_FLAG_HTIF0|DMA_FLAG_TEIF0|DMA_FLAG_DMEIF0|DMA_FLAG_FEIF0);                                                                                                 
	DMA_ClearITPendingBit(DMA1_Stream0,DMA_IT_TEIF0|DMA_IT_HTIF0|DMA_IT_TCIF0|DMA_IT_DMEIF0|DMA_IT_FEIF0);
	DMA_SetCurrDataCounter(DMA1_Stream0,dwLength);
	DMA_Cmd(DMA1_Stream0, ENABLE);
}

#include <task_pc_parse.h>

#include <drive_usart.h>

#include <robotstatus.h>

#include <bsp_dma.h>

//char a='0',b='1',c='0',d='0';
//char last_a,last_b,last_c,last_d;
//char ff=0;
//char flag=0;
//char flagg[6]={1,1,1,1,1,1};
void vTaskPCParse(void *pvParameters)
{
	printf("��ӭʹ�ó���������\r\n");
	const TickType_t xBlockTime = pdMS_TO_TICKS(100); // �������ȴ�ʱ��Ϊ 100ms
	while(1)
	{
		ulTaskNotifyTake(pdTRUE,xBlockTime);
		
		MiniPC_Rece_Resolver();
		
//		if(g_DMA_MiniPC_Reve_Buff[0]==0XBF&&g_DMA_MiniPC_Reve_Buff[1]==0XAA)
//		{
//			if(last_a!=g_DMA_MiniPC_Reve_Buff[0]||last_b!=g_DMA_MiniPC_Reve_Buff[1]||last_c!=g_DMA_MiniPC_Reve_Buff[2]||last_d!=g_DMA_MiniPC_Reve_Buff[3])
//			{
//				printf("����������\r\n");
//				ff=1;
//			}
//			
//		}
//		
//		
//		if(g_DMA_MiniPC_Reve_Buff[0]==0XB8&&g_DMA_MiniPC_Reve_Buff[1]==0XFC)
//		{
//			if(last_a!=g_DMA_MiniPC_Reve_Buff[0]||last_b!=g_DMA_MiniPC_Reve_Buff[1]||last_c!=g_DMA_MiniPC_Reve_Buff[2]||last_d!=g_DMA_MiniPC_Reve_Buff[3])
//			{
//			   printf("������������\r\n");
//			   flag=1;
//			}
//		}
//		
//		if(flag==1&&g_DMA_MiniPC_Reve_Buff[0]!=0XD0&&g_DMA_MiniPC_Reve_Buff[0]<='9')
//		{
//			if(last_a!=g_DMA_MiniPC_Reve_Buff[0]||last_b!=g_DMA_MiniPC_Reve_Buff[1]||last_c!=g_DMA_MiniPC_Reve_Buff[2]||last_d!=g_DMA_MiniPC_Reve_Buff[3])
//			{
//	
//				
//			a=g_DMA_MiniPC_Reve_Buff[0];
//			b=g_DMA_MiniPC_Reve_Buff[1];
//			c=g_DMA_MiniPC_Reve_Buff[2];
//			d=g_DMA_MiniPC_Reve_Buff[3];
//			flag=0;
//			printf("�������óɹ������������Ϊ%c%c%c%c\r\n",a,b,c,d);
//			}
//		}
//		
//		if(ff==1&&g_DMA_MiniPC_Reve_Buff[0]==a&&g_DMA_MiniPC_Reve_Buff[1]==b&&g_DMA_MiniPC_Reve_Buff[2]==c&&g_DMA_MiniPC_Reve_Buff[3]==d)
//		{

//			if(last_a!=g_DMA_MiniPC_Reve_Buff[0]||last_b!=g_DMA_MiniPC_Reve_Buff[1]||last_c!=g_DMA_MiniPC_Reve_Buff[2]||last_d!=g_DMA_MiniPC_Reve_Buff[3])
//			{
//	
//			   printf("������ȷ���ѿ���\r\n");
//				ff=0;
//			}
//		}
//		
//		else if(ff==1&&(g_DMA_MiniPC_Reve_Buff[0]!=a||g_DMA_MiniPC_Reve_Buff[1]!=b||g_DMA_MiniPC_Reve_Buff[2]!=c||g_DMA_MiniPC_Reve_Buff[3]!=d))
//		{
//			if(g_DMA_MiniPC_Reve_Buff[0]!=0XBF&&g_DMA_MiniPC_Reve_Buff[1]!=0XAA)
//			{
//				if(g_DMA_MiniPC_Reve_Buff[0]!=0XB8&&g_DMA_MiniPC_Reve_Buff[1]!=0XFC)
//				{
//		  if(last_a!=g_DMA_MiniPC_Reve_Buff[0]||last_b!=g_DMA_MiniPC_Reve_Buff[1]||last_c!=g_DMA_MiniPC_Reve_Buff[2]||last_d!=g_DMA_MiniPC_Reve_Buff[3])
//			{

//			   printf("�������\r\n");
//			}
//		}
//		}
//		}
//		last_a=g_DMA_MiniPC_Reve_Buff[0];
//			last_b=g_DMA_MiniPC_Reve_Buff[1];
//			last_c=g_DMA_MiniPC_Reve_Buff[2];
//			last_d=g_DMA_MiniPC_Reve_Buff[3];
//		
//	
	}
}


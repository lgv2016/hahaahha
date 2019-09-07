#include <drive_judge.h>

#include <robotstatus.h>
#include <drive_usart.h>

#include <math_tool.h>

judge_data_t judge_data;


void Get_Judge_data(CanRxMsg rx_message)
{
    switch(rx_message.StdId)
    {
      case 0x208:
      {
		  judge_data.shooter_heat0  = rx_message.Data[0]<<8|rx_message.Data[1];
		  judge_data.power_buffer   = rx_message.Data[2]<<8|rx_message.Data[3];
		  
		  judge_data.shoot_rate     = rx_message.Data[4]<<8|rx_message.Data[5];
		  judge_data.robot_level    = rx_message.Data[6];
		  judge_data.robot_id       = rx_message.Data[7];
		  
		  if(judge_data.robot_id<=7)
		  {
			  robot_status.enemy_color=COLOR_RED;
		  }
		  else
		  {
			  robot_status.enemy_color=COLOR_BLUE;
		  }
		  MiniPC_Send_Data(0x02);
          break;
      }
      default:
        break;
    } 
}

void Cmd_Judge_ESC()
{
    
   CanTxMsg tx_message;
	
	tx_message.StdId = 0x1FD;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
	
	
    tx_message.Data[0] = robot_status.fric_status;     //1:打开 0：关闭
  //  tx_message.Data[1] = robot_status.gimbal_data;     //1：校准成功  0：失败
    tx_message.Data[2] = robot_status.enemy_armor;     //1:小装甲 0：大装甲
	
	tx_message.Data[3] = robot_status.depot_status;

	
	
	
    CAN_Transmit(CAN1,&tx_message);
}


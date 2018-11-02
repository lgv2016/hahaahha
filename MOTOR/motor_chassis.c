#include <motor_chassis.h>

data_3510_t g_data_3510;

void Cmd_3510_ESC(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
   
    tx_message.Data[0] = (unsigned char)(current_201 >> 8);
    tx_message.Data[1] = (unsigned char)current_201;
    tx_message.Data[2] = (unsigned char)(current_202 >> 8);
    tx_message.Data[3] = (unsigned char)current_202;
    tx_message.Data[4] = (unsigned char)(current_203 >> 8);
    tx_message.Data[5] = (unsigned char)current_203;
    tx_message.Data[6] = (unsigned char)(current_204 >> 8);
    tx_message.Data[7] = (unsigned char)current_204;
    CAN_Transmit(CAN1,&tx_message);
}

void Get_3510_data(CanRxMsg rx_message)
{
     switch(rx_message.StdId)
    {
       case 0x201:
      {
          g_data_3510.angle[0]=rx_message.Data[0]<<8|rx_message.Data[1];
          g_data_3510.speed[0]=rx_message.Data[2]<<8|rx_message.Data[3];
          break;
      }
       case 0x202:
      {
          g_data_3510.angle[1]=rx_message.Data[0]<<8|rx_message.Data[1];
          g_data_3510.speed[1]=rx_message.Data[2]<<8|rx_message.Data[3];
          break;
      }
       case 0x203:
      {
          g_data_3510.angle[2]=rx_message.Data[0]<<8|rx_message.Data[1];
          g_data_3510.speed[2]=rx_message.Data[2]<<8|rx_message.Data[3];
          break;
      }
       case 0x204:
      {
          g_data_3510.angle[3]=rx_message.Data[0]<<8|rx_message.Data[1];
          g_data_3510.speed[3]=rx_message.Data[2]<<8|rx_message.Data[3];
          break;
      }
      default:
        break;
    } 
}

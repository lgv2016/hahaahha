//此云台电机为6623
//所用驱动为2016年旧版电调
#include <motor_cradle_head.h>

static int16_t err_angle[2]={0};
static int16_t last_angle[2]={0};

data_6623_t g_data_6623;

void Cmd_6623_ESC(int16_t  current_205,int16_t current_206)
{
    CanTxMsg tx_message;
    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(current_205 >> 8);
    tx_message.Data[1] = (unsigned char)current_205;
    tx_message.Data[2] = (unsigned char)(current_206 >> 8);
    tx_message.Data[3] = (unsigned char)current_206;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
  
    CAN_Transmit(CAN1,&tx_message);
}

void Get_6623_data(CanRxMsg rx_message)
{
     switch(rx_message.StdId)
    {
       case 0x205:
      {
          g_data_6623.angle[0]             =   rx_message.Data[0]<<8|rx_message.Data[1];
          g_data_6623.actual_current[0]    =   rx_message.Data[2]<<8|rx_message.Data[3];
          g_data_6623.set_current[0]       =   rx_message.Data[4]<<8|rx_message.Data[5];

          break;
      }
      case 0x206:
      {
          g_data_6623.angle[1]             =   rx_message.Data[0]<<8|rx_message.Data[1];
          g_data_6623.actual_current[1]    =   rx_message.Data[2]<<8|rx_message.Data[3];
          g_data_6623.set_current[1]       =   rx_message.Data[4]<<8|rx_message.Data[5];
    
          break;
      }
      default:
        break;
    } 
}

void Get_6623_Speed(data_6623_t data)//M法 10ms
{

    float temp;
    err_angle[0]=data.angle[0]-last_angle[0];
    
    if((err_angle[0]<4000)&&(err_angle[0]>-4000))
    {
        temp=((err_angle[0]*360)/8191.0f)*100.0f;
          if((temp<4000)&&(temp>-4000))
            g_data_6623.speed[0]=temp;
        
    }
    last_angle[0]=data.angle[0];
    
    
    err_angle[1]=data.angle[1]-last_angle[1];
    
     if((err_angle[1]<4000)&&(err_angle[1]>-4000))
    {
        temp=((err_angle[1]*360)/8191.0f)*100.0f;
          if((temp<4000)&&(temp>-4000))
            g_data_6623.speed[1]   =   temp;
        
    }
    last_angle[1]=g_data_6623.angle[1];
}




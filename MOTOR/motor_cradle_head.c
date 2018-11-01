//此云台电机为6623
//所用驱动为2016年旧版电调
#include <motor_cradle_head.h>
#include <math_tool.h>

static CanTxMsg s_tx_message;

data_6623_t g_data_6623;
data_2006_t g_data_2006;

void Cmd_6623_ESC(int16_t  current_205,int16_t current_206)
{
    s_tx_message.StdId = 0x1FF;
    s_tx_message.IDE = CAN_Id_Standard;
    s_tx_message.RTR = CAN_RTR_Data;
    s_tx_message.DLC = 0x08;
    
    s_tx_message.Data[0] = (unsigned char)(current_205 >> 8);
    s_tx_message.Data[1] = (unsigned char)current_205;
    s_tx_message.Data[2] = (unsigned char)(current_206 >> 8);
    s_tx_message.Data[3] = (unsigned char)current_206;

  
    CAN_Transmit(CAN1,&s_tx_message);
}

void Cmd_2006_ESC(int16_t  current_207)
{
     CanTxMsg s_tx_message;
    
    s_tx_message.StdId = 0x1FF;
    s_tx_message.IDE = CAN_Id_Standard;
    s_tx_message.RTR = CAN_RTR_Data;
    s_tx_message.DLC = 0x08;
    
    s_tx_message.Data[4] = (unsigned char)(current_207 >> 8);
    s_tx_message.Data[5] = (unsigned char)current_207;
    s_tx_message.Data[6] = 0x00;
    s_tx_message.Data[7] = 0x00;
  
    CAN_Transmit(CAN1,&s_tx_message);
}

void Get_6623_data(CanRxMsg rx_message)
{
     switch(rx_message.StdId)
    {
       case 0x205:
      {
          g_data_6623.angle[YAW]             =   rx_message.Data[0]<<8|rx_message.Data[1];
          g_data_6623.actual_current[YAW]    =   rx_message.Data[2]<<8|rx_message.Data[3];
          g_data_6623.set_current[YAW]       =   rx_message.Data[4]<<8|rx_message.Data[5];

          break;
      }
      case 0x206:
      {
          g_data_6623.angle[PITCH]             =   rx_message.Data[0]<<8|rx_message.Data[1];
          g_data_6623.actual_current[PITCH]    =   rx_message.Data[2]<<8|rx_message.Data[3];
          g_data_6623.set_current[PITCH]       =   rx_message.Data[4]<<8|rx_message.Data[5];
    
          break;
      }
      default:
        break;
    } 
}

void Get_2006_data(CanRxMsg rx_message)
{
    
    //中间变量 便于采样
    int32_t angle1;  
    float angle2;
     switch(rx_message.StdId)
    {
      case 0x207:
      {
          g_data_2006.last_angle=g_data_2006.pre_angle;
          g_data_2006.pre_angle               =   rx_message.Data[0]<<8|rx_message.Data[1];
          g_data_2006.speed                   =   rx_message.Data[2]<<8|rx_message.Data[3];
          g_data_2006.torque                  =   rx_message.Data[4]<<8|rx_message.Data[5];
          
          
          
          if(g_data_2006.pre_angle-g_data_2006.last_angle>4096)
              g_data_2006.count--;
          else if(g_data_2006.pre_angle-g_data_2006.last_angle<-4096)
              g_data_2006.count++;
          
          angle1=g_data_2006.count*8192+g_data_2006.pre_angle-g_data_2006.offset_angle;
          g_data_2006.total_angle=angle1;
          
          
          angle2=(g_data_2006.total_angle*360.0f)/(8192.0f*36.0f);
          g_data_2006.angle=angle2;

         if(!ApplyDeadbandFloat((abs(g_data_2006.angle)-360.0f),0.5f))
             g_data_2006.count=0;
          break;
      }
      default:
        break;
    } 
}

void Get_2006_Offset_angle(CanRxMsg rx_message)
{
     switch(rx_message.StdId)
    {
      case 0x207:
      {
          g_data_2006.offset_angle=rx_message.Data[0]<<8|rx_message.Data[1];
          break;
      }
      default:
        break;
    } 
}




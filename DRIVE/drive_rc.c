#include <drive_rc.h>
#include <bsp_dma.h>
#include <drive_delay.h>

#include <math_tool.h>

#include <drive_control.h>
#include <robotstatus.h>

#include <drive_gimble.h>

#include <drive_imu.h>

// Û±Í¡È√Ù∂»…Ë÷√
#define YAW_SENSITIVITY   2.5f
#define PIT_SENSITIVITY   1.5f



rc_control_t g_rc_control={0};




void RC_Data_Parse()
{
	
	
	  u8 last_key_g;
	  static uint64_t previousT;
      float deltaT = (Get_SysTimeUs() - previousT) * 1e-6f;
      previousT = Get_SysTimeUs();
	
	  last_key_g=g_rc_control.key.k[G];
	
      g_rc_control.rc.ch0         =  (g_DMA_Dbus_Buff[0]       | (g_DMA_Dbus_Buff[1] << 8)) & 0x07ff; 
      g_rc_control.rc.ch1         = ((g_DMA_Dbus_Buff[1] >> 3) | (g_DMA_Dbus_Buff[2] << 5)) & 0x07ff; 
      g_rc_control.rc.ch2         = ((g_DMA_Dbus_Buff[2] >> 6) | (g_DMA_Dbus_Buff[3] << 2) | (g_DMA_Dbus_Buff[4] << 10)) & 0x07ff;  
      g_rc_control.rc.ch3         = ((g_DMA_Dbus_Buff[4] >> 1) | (g_DMA_Dbus_Buff[5] << 7)) & 0x07ff; 
      g_rc_control.rc.s1          = ((g_DMA_Dbus_Buff[5] >> 4)& 0x000C) >> 2; 
      g_rc_control.rc.s2          = ((g_DMA_Dbus_Buff[5] >> 4)& 0x0003); 
          
    
      g_rc_control.mouse.x        =   g_DMA_Dbus_Buff[6]  | (g_DMA_Dbus_Buff[7]<< 8); 
      g_rc_control.mouse.y        =   g_DMA_Dbus_Buff[8]  | (g_DMA_Dbus_Buff[9]<< 8); 
      g_rc_control.mouse.z        =   g_DMA_Dbus_Buff[10] | (g_DMA_Dbus_Buff[11]<<8); 
      g_rc_control.mouse.press_l  =   g_DMA_Dbus_Buff[12];  
      g_rc_control.mouse.press_r  =   g_DMA_Dbus_Buff[13];  
      g_rc_control.key.v          =   g_DMA_Dbus_Buff[14] | (g_DMA_Dbus_Buff[15]<< 8); 
      
	  
      g_rc_control.key.k[W]       =   (g_rc_control.key.v&(0X0001<<0))>>0;
	  g_rc_control.key.k[S]       =   (g_rc_control.key.v&(0X0001<<1))>>1;   
      g_rc_control.key.k[A]       =   (g_rc_control.key.v&(0X0001<<2))>>2;  
      g_rc_control.key.k[D]       =   (g_rc_control.key.v&(0X0001<<3))>>3;
      g_rc_control.key.k[SHIFT]   =   (g_rc_control.key.v&(0X0001<<4))>>4;
      g_rc_control.key.k[CTRL]    =   (g_rc_control.key.v&(0X0001<<5))>>5;
     
	  g_rc_control.key.k[Q]       =   (g_rc_control.key.v&(0X0001<<6))>>6;
	  g_rc_control.key.k[E]       =   (g_rc_control.key.v&(0X0001<<7))>>7;
	  
	  g_rc_control.key.k[R]       =   (g_rc_control.key.v&(0X0001<<8))>>8;
	  
	  g_rc_control.key.k[F]       =   (g_rc_control.key.v&(0X0001<<9))>>9;
	  g_rc_control.key.k[G]       =   (g_rc_control.key.v&(0X0001<<10))>>10;
	  
	  g_rc_control.key.k[Z]       =   (g_rc_control.key.v&(0X0001<<11))>>11;
	  
	  g_rc_control.key.k[X]       =   (g_rc_control.key.v&(0X0001<<12))>>12;
	  
	  g_rc_control.key.k[C]       =   (g_rc_control.key.v&(0X0001<<13))>>13;
	  g_rc_control.key.k[V]       =   (g_rc_control.key.v&(0X0001<<14))>>14;
	  
	  g_rc_control.key.k[B]       =   (g_rc_control.key.v&(0X0001<<15))>>15;
	  
	  
	  
	  if(robot_status.gimbal_mode==GIMBLE_PC)
	  {
		  g_rc_control.mouse.x_distance+=-g_rc_control.mouse.x*deltaT*YAW_SENSITIVITY;
		  g_rc_control.mouse.y_distance+=-g_rc_control.mouse.y*deltaT*PIT_SENSITIVITY;
	  }
	  
	  if(!last_key_g&&g_rc_control.key.k[G])
	  {
		   g_rc_control.mouse.x_distance+=-180.0f;
	  }
}






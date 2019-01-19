#include <drive_rc.h>
#include <bsp_dma.h>

rc_control_t g_rc_control;

void RC_Data_Parse()
{
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
      
      g_rc_control.key.k[W]       =   (g_rc_control.key.v&(0X01<<0))>>0;
	  g_rc_control.key.k[S]       =   (g_rc_control.key.v&(0X01<<1))>>1;   
      g_rc_control.key.k[A]       =   (g_rc_control.key.v&(0X01<<2))>>2;  
      g_rc_control.key.k[D]       =   (g_rc_control.key.v&(0X01<<3))>>3;
      g_rc_control.key.k[Q]       =   (g_rc_control.key.v&(0X01<<4))>>4;
      g_rc_control.key.k[E]       =   (g_rc_control.key.v&(0X01<<5))>>5;
      g_rc_control.key.k[SHIFT]   =   (g_rc_control.key.v&(0X01<<6))>>6;
      g_rc_control.key.k[CTRL]    =   (g_rc_control.key.v&(0X01<<7))>>7;
}


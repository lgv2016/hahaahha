#include <drive_imu.h>
#include <drive_delay.h>

static u8             s_mpu_buff[6];                       
static mpu_data_t     s_mpu_data;
imu_data_t            g_imu_data;


void IIC_Start(void)
{
	SDA_OUT();    
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	Delay_US(4);
 	IIC_SDA=0;
	Delay_US(4);
	IIC_SCL=0;
}	  

void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
 	Delay_US(4);
	IIC_SCL=1; 
	Delay_US(4);			
	IIC_SDA=1;	   	
}

u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      
	IIC_SDA=1;Delay_US(1);	   
	IIC_SCL=1;Delay_US(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;   
	return 0;  
} 

void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	Delay_US(2);
	IIC_SCL=1;
	Delay_US(2);
	IIC_SCL=0;
}
	    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	Delay_US(2);
	IIC_SCL=1;
	Delay_US(2);
	IIC_SCL=0;
}					 				     
	  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delay_US(2);  
		IIC_SCL=1;
		Delay_US(2); 
		IIC_SCL=0;	
		Delay_US(2);
    }	 
} 	    

u8 IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        Delay_US(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		Delay_US(1); 
    }					 
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();  
    return receive;
}




u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); 
    if(IIC_Wait_Ack())       
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         
    IIC_Wait_Ack();             
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  
        if(IIC_Wait_Ack())      
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 


u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); 
    if(IIC_Wait_Ack())         
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         
    IIC_Wait_Ack();            
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); 
    IIC_Wait_Ack();            
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);
		else *buf=IIC_Read_Byte(1);		
		len--;
		buf++;  
    }
    IIC_Stop();                
    return 0;       
}

 

u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); 
    if(IIC_Wait_Ack())         
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         
    IIC_Wait_Ack();           
    IIC_Send_Byte(data);        
    if(IIC_Wait_Ack())          
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}


u8 MPU_Read_Byte(u8 addr,u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); 
    IIC_Wait_Ack();            
    IIC_Send_Byte(reg);         
    IIC_Wait_Ack();            
	  IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); 
    IIC_Wait_Ack();             
    res=IIC_Read_Byte(0);		
    IIC_Stop();                 
    return res;  
}
void MPU_offset_call(void)
{
  int i;
	for (i=0; i<300;i++)
	{
        MPU_Read_Len(MPU6500_ADDR,MPU_GYRO_XOUTH_REG,6, s_mpu_buff);
		s_mpu_data.gx_offset +=  s_mpu_buff[0]  << 8 |  s_mpu_buff[1];
		s_mpu_data.gy_offset +=  s_mpu_buff[2] << 8  |  s_mpu_buff[3];
		s_mpu_data.gz_offset +=  s_mpu_buff[4] << 8  |  s_mpu_buff[5];
		Delay_MS(5);
	}
	s_mpu_data.gx_offset=s_mpu_data.gx_offset / 300;
	s_mpu_data.gy_offset=s_mpu_data.gx_offset / 300;
	s_mpu_data.gz_offset=s_mpu_data.gz_offset / 300;
}

void MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	MPU_Read_Len(MPU6500_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    g_imu_data.temp=21+((double)raw)/333.87;  
}

u8 MPU_Get_Gyroscope()
{
    u8 res; 
	res=MPU_Read_Len(MPU6500_ADDR,MPU_GYRO_XOUTH_REG,6, s_mpu_buff);
	if(res==0)
	{
		s_mpu_data.gx = (( s_mpu_buff[0] << 8 |  s_mpu_buff[1]) - s_mpu_data.gx_offset);
		s_mpu_data.gy = (( s_mpu_buff[2] << 8  |  s_mpu_buff[3]) - s_mpu_data.gy_offset);
		s_mpu_data.gz = (( s_mpu_buff[4] << 8  |  s_mpu_buff[5]) - s_mpu_data.gz_offset);
    
		g_imu_data.pit_speed   = s_mpu_data.gx / 16.384f ;
		g_imu_data.rol_speed   = s_mpu_data.gy / 16.384f ; 
		g_imu_data.yaw_speed   = s_mpu_data.gz / 16.384f ;
	} 	
    return res;
}

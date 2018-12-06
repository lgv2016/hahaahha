#ifndef __DRIVE_IMU_H
#define __DRIVE_IMU_H
#include <stm32f4xx.h>


#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 
#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����




#define MPU6500_ADDR            0X68    //MPU6500������IIC��ַ
#define MPU_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_TEMP_OUTH_REG		0X41	//�¶�ֵ�߰�λ�Ĵ���
#define MPU_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���



#define SDA_IN()  {GPIOF->MODER&=~(3<<(9*2));GPIOF->MODER|=0<<9*2;}	
#define SDA_OUT() {GPIOF->MODER&=~(3<<(9*2));GPIOF->MODER|=1<<9*2;} 

#define IIC_SCL   PFout(7) //SCL
#define IIC_SDA   PFout(9) //SDA
#define READ_SDA  PFin(9)  //����SDA

				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(u8 ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	 


void MPU_offset_call(void);
u8 MPU_Write_Byte(u8 devaddr,u8 reg,u8 data);
u8 MPU_Read_Byte(u8 devaddr,u8 reg);

u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);

void MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(void);

typedef struct
{
	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
}mpu_data_t;

typedef struct
{
	float temp;
	
	float pit_speed;
	float rol_speed;
	float yaw_speed;


	float rol;
	float pit;
	float yaw;
}imu_data_t;

extern imu_data_t   g_imu_data;
#endif




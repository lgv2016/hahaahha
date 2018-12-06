#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include <stm32f4xx.h>
extern void  BSP_GPIO_Init(void);


#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOF_ODR_Addr    (GPIOF_BASE+20) //0x40021414    
   
#define GPIOF_IDR_Addr    (GPIOF_BASE+16) //0x40021410 

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define AD PFout(8)



#endif  

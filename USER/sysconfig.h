#ifndef __SYSCONFIG_H
#define __SYSCONFIG_H

#include <stdio.h>
#include <delay.h>
#include <bsp_gpio.h>
#include <bsp_usart.h>
#include <bsp_dma.h>
#include <bsp_nvic.h>
#include <bsp_can.h>



extern void SYS_Config_Init(void);
extern int fputc(int ch, FILE *f);
#endif
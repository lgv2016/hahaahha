#ifndef __SYSCONFIG_H
#define __SYSCONFIG_H


#include <stdio.h>
#include <bsp_can.h>
#include <bsp_dma.h>
#include <bsp_time.h>
#include <bsp_gpio.h>
#include <bsp_nvic.h>
#include <bsp_delay.h>
#include <bsp_usart.h>


#include <drive_rc.h>
#include <drive_imu.h>
#include <drive_delay.h>
#include <drive_control.h>


#include <motor_chassis.h>
#include <motor_cradle_head.h>


#include <task_config.h>
#include <task_can_parse.h>


#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 


extern void SYS_Config_Init(void);
extern int fputc(int ch, FILE *f);
#endif

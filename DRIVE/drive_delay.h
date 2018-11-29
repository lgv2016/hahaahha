#ifndef __DRIVE_DELAY_H
#define __DRIVE_DELAY_H

#include <stm32f4xx.h>

extern void Delay_US(uint32_t us);
extern void Delay_MS(uint32_t ms);

extern uint64_t Get_SysTimeUs(void);
extern uint32_t Get_SysTimeMs(void);



#endif

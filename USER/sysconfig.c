#include <sysconfig.h>

void SYS_Config_Init()
{
    delay_init(180);
    BSP_GPIO_Init();
}
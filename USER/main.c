#include <sysconfig.h>
int main()
{
    
    SYS_Config_Init();
    while(1)
    {
        GPIO_ResetBits(GPIOG, GPIO_Pin_1);
        delay_ms(500);
        GPIO_SetBits(GPIOG, GPIO_Pin_1);
        delay_ms(500);
    }
    return 0;
}



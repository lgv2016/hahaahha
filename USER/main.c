#include <sysconfig.h>
int main()
{
    SYS_Config_Init();
    while(1)
    {
        delay_ms(200);
        GPIO_ResetBits(GPIOG, GPIO_Pin_1);
        delay_ms(200);
        GPIO_SetBits(GPIOG, GPIO_Pin_1);
    }
    return 0;
}


  


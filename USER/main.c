#include <sysconfig.h>
int main()
{
    SYS_Config_Init();
    while(1)
    {
        Cmd_3510_ESC(1000,1000,1000,1000);
        delay_ms(2000);
        GPIO_ResetBits(GPIOG, GPIO_Pin_1);
        delay_ms(2000);
        GPIO_SetBits(GPIOG, GPIO_Pin_1);
     
    }
    return 0;
}



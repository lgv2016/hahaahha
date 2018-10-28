#include <sysconfig.h>
int main()
{
    SYS_Config_Init();
     Cmd_6623_ESC(-1500,1500);
    while(1)
    {
       
        delay_ms(2000);
        GPIO_ResetBits(GPIOG, GPIO_Pin_1);
        delay_ms(2000);
        GPIO_SetBits(GPIOG, GPIO_Pin_1);
     
    }
    return 0;
}



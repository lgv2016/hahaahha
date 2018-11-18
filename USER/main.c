#include <sysconfig.h>
int main()
{
    
    SYS_Config_Init();
    while(1)
    {
        printf("fdfd");
        //Cmd_6623_ESC(1200,0);
        GPIO_ResetBits(GPIOG, GPIO_Pin_1);
        delay_ms(2000);
        GPIO_SetBits(GPIOG, GPIO_Pin_1);
        delay_ms(2000);
    }
    return 0;
}



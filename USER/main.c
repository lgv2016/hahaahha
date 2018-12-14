#include <sysconfig.h>

int main()
{
	
    SYS_Config_Init();
    xTaskCreate(vTaskStart,             
                "vTaskStart",           
                128,        
                NULL,                  
                1,        
                &xHandleTaskStart);
    vTaskStartScheduler();
}





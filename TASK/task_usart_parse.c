#include <task_usart_parse.h>
#include <drive_delay.h>
#include <motor_cradle_head.h>
#include <drive_control.h>

#include <robotstatus.h>

#include <drive_judge.h>


//����ϵͳ���ݽ�������
void vTaskUSARTParse(void *pvParameters)
{
    while(1)
    {
	 Cmd_Judge_ESC();
	 vTaskDelay(20);
    }
}

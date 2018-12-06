#include <task_control.h>
#include <drive_control.h>
#include <drive_delay.h>

#include <robotstatus.h>


void vTaskControl(void *pvParameters)
{
	vTaskDelay(1000);  //6623��ʼʱ���ݲ���ȷ����Ҫ��ʱ�ȴ�
    while(1)
    {
		Speed_2006_Control(g_speed_target);
	    Speed_3510_Control(g_speed_target);
		Angle_6623_Control(g_angle_target);
		vTaskDelay(10);
    }
}

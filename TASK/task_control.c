#include <task_control.h>
#include <drive_control.h>
#include <drive_delay.h>
#include <robotstatus.h>
#include <motor_cradle_head.h>
#include <drive_rc.h>

#include <drive_chassis.h>
#include <drive_gimble.h>
#include <drive_shoot.h>


control_cycle_t s_control_cycle;
void vTaskControl(void *pvParameters)
{

	SHOOT_Init();
	vTaskDelay(1500);  //6623开始时数据不正确，需要延时等待
    while(1)
    {	
		s_control_cycle.gimble_cycle++;
		s_control_cycle.shoot_cycle++;
		
		if(s_control_cycle.shoot_cycle==SHOOT_CONTROL_CYCLE)
		{
			SHOOT_Loop_Control();
			s_control_cycle.shoot_cycle=0;
		}
		
		vTaskDelay(1);
	}
}

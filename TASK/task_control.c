
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
	
	GIMBLE_Init();
	CHASSIS_Init();
	SHOOT_Init();

	
	vTaskDelay(1000);
    while(1)
    {	
		s_control_cycle.shoot_cycle++;
		s_control_cycle.gimble_cycle++;
		s_control_cycle.chassis_cycyle++;
		
		if(s_control_cycle.shoot_cycle==SHOOT_CONTROL_CYCLE)
		{
			SHOOT_Loop_Control();
			s_control_cycle.shoot_cycle=0;
		}
		
		if(s_control_cycle.gimble_cycle==GIMBLE_CONTROL_CYCLE)
		{
			
		    GIMBLE_Loop_Control();
			s_control_cycle.gimble_cycle=0;
		}
		
		if(s_control_cycle.chassis_cycyle==SHOOT_CONTROL_CYCLE)
		{
			CHASSIS_Loop_Control();
			s_control_cycle.chassis_cycyle=0;
		}


		vTaskDelay(1);
	}
}

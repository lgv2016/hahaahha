#include <task_control.h>
#include <drive_control.h>
#include <drive_delay.h>
#include <robotstatus.h>
#include <motor_cradle_head.h>
#include <drive_rc.h>




#include <drive_chassis.h>
#include <drive_gimble.h>
#include <drive_shoot.h>



char g_2006_angle_reset=0;
char g_2006_angle_flag=1;
void vTaskControl(void *pvParameters)
{

	robot_status.chassis_mode=CH_SPEED;
	u8 last_shoot_mode;
	vTaskDelay(1500);  //6623开始时数据不正确，需要延时等待
    while(1)
    {
		if(robot_status.shoot_mode==AWM)           //单发模式
		{
			last_shoot_mode=AWM;
			if(g_2006_angle_reset==1)             //2006角度置零
			{
				g_2006_angle_flag=1;              //2006偏移角度记录
				g_2006_angle_reset=0;
				g_data_2006.count=0;              //2006旋转圈数
			}
			g_angle_target.shoot=60;              //目标角度控制
			Angle_2006_Control(g_angle_target);
		}
		else if(robot_status.shoot_mode==AK47)    //连发模式
		{
			last_shoot_mode=AK47;
			g_speed_target.shoot=1200;            //目标速度控制
			Speed_2006_Control(g_speed_target);
		}
		else if(robot_status.shoot_mode==RELOAD)  //拨弹关闭并维持角度和速度
		{
			if(last_shoot_mode==AWM)              
			{
				g_angle_target.shoot=60;
				Angle_2006_Control(g_angle_target);
			}
			else if(last_shoot_mode==AK47)
			{
				g_speed_target.shoot=0;
				Speed_2006_Control(g_speed_target);
			}
		}
		if(robot_status.control_mode==USE_RC)
		{
			
			//CHASSIS_Move_RC_Control();
			
		CH_Speed_Control(2000,2000);
		}

		
		
	   GIMBLE_Control();
		
		
		

	//CHASSIS_Follow_Gimble_Control();
		//CHASSIS_Move_Control(1500,1500);
	///CHASSIS_Rotate_Control(100);
	
		vTaskDelay(3);
	}
}

#include <task_control.h>
#include <drive_control.h>
#include <drive_delay.h>
#include <robotstatus.h>
#include <motor_cradle_head.h>
#include <drive_rc.h>


#define YAW_INIT_ANGLE    281.0f
#define PIT_INIT_ANGLE    110.0f

void Chassis_Move(u16 xmaxval,u16 ymaxval)
{
	float xspeed,yspeed;
	if(g_rc_control.rc.ch2==1684)
	{
		xspeed=xmaxval;
	}
	else if(g_rc_control.rc.ch2==364)
	{
		xspeed=-xmaxval;
	}
	else
	{
		xspeed=0;
	}
	
	if(g_rc_control.rc.ch3==1684)
	{
		yspeed=ymaxval;
	}
	else if(g_rc_control.rc.ch3==364)
	{
		yspeed=-ymaxval;
	}
	else
	{
		yspeed=0;
	}
	Speed_Chassis_Control(xspeed,yspeed,0);
}
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
		if(robot_status.chassis_mode==CH_SPEED)
		{
			if(robot_status.control_mode==USE_RC)
			{
				Chassis_Move(2000,2000);
			}
		}
		
		else if(robot_status.chassis_mode==CH_ROTATE)
		{
			 if(robot_status.control_mode==USE_RC)
			{
				//Speed_Rotate_Control(g_speed_target);
				//Angle_Rotate_Control(g_angle_target);
			}
		}
		
		if(1)                                                //云台控制
		{

//			g_angle_target.pitch =PIT_INIT_ANGLE+g_pit_target;
//            g_angle_target.yaw   =YAW_INIT_ANGLE+g_yaw_target;
			
		     Angle_6623_Control(g_angle_target);
		}		
		
		
		vTaskDelay(3);
	}
}

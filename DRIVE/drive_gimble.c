#include <drive_imu.h>
#include <drive_gimble.h>
#include <drive_control.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>

#include <drive_usart.h>

static Gimbal_Motor_t Gimbal_Motor_yaw;
static Gimbal_Motor_t Gimbal_Motor_pit;


#define PIT_RC_LIMIT_ANGLE  15
#define YAW_RC_LIMIT_ANGLE  90



#define PIT_RC_LIMIT_ANGLE_DOWN  15
#define PIT_RC_LIMIT_ANGLE_UP  60




#define YAW_PC_LIMIT_ANGLE  90000    //最大圈数正负250圈

#define PIT_PC_LIMIT_ANGLE  15
#define PIT_PC_LIMIT_ANGLE_DOWN  15
#define PIT_PC_LIMIT_ANGLE_UP    60



#define MOTOR_INIT_TIME   (80/GIMBLE_CONTROL_CYCLE)
#define R_PRESS_LONG_TIME (350/GIMBLE_CONTROL_CYCLE)

#define RC_SW_UP   ((uint16_t)1)
#define RC_SW_MID  ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)


#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s)  (s == RC_SW_MID)
#define switch_is_up(s)   (s == RC_SW_UP)


static void GIMBLE_Set_Mode(void);
static void GIMBLE_RC_Control(void);
static void GIMBLE_PC_Control(void);
static void GIMBLE_Data_Update(void);
static void GIMBLE_Auto_Control(void);
static void GIMBLE_Init_Control(void);



void GIMBLE_Init()
{
	robot_status.gimbal_mode = GIMBAL_NO;
	robot_status.motor_yaw   = MOTOR_GIMBAL_NO;
	robot_status.motor_pit   = MOTOR_GIMBAL_NO;
	robot_status.gimbal_data = GIMBAL_MOTOR_NO;
}

/*
 *函 数 名: GIMBLE_Set_Mode
 *功能说明: 云台状态更新
 *形    参: void
 *返 回 值: void
 */




static void GIMBLE_Set_Mode()
{
	
	//CAN接收到电机数据
	if(robot_status.motor_yaw==MOTOR_GIMBAL_ENCODE&&robot_status.motor_pit==MOTOR_GIMBAL_ENCODE)
	{
		if(robot_status.gimbal_data==GIMBAL_MOTOR_NO)
		{
			robot_status.gimbal_data=GIMBAL_MOTOR_ENCONDE;
		}
	}
	
	//云台初始化
	if(robot_status.gimbal_data==GIMBAL_MOTOR_ENCONDE)
	{
		robot_status.gimbal_mode=GIMBLE_INIT;
	}
	
    //操作手云台控制
	if(robot_status.gimbal_data==GIMBAL_MOTOR_GYRO)
	{
		
		if(switch_is_down(g_rc_control.rc.s1))
		{
			robot_status.gimbal_mode=GIMBLE_AUTO;
		}
		if(switch_is_up(g_rc_control.rc.s1))
		{
			robot_status.gimbal_mode=GIMBLE_RC;
		}
		if(switch_is_mid(g_rc_control.rc.s1))
		{
			robot_status.gimbal_mode=GIMBLE_PC;
			
			if(Gimbal_Motor_yaw.press_r_time==R_PRESS_LONG_TIME)
			{
				robot_status.gimbal_mode=GIMBLE_AUTO;
			}
		}
	}	
}
/*
 *函 数 名: GIMBLE_Init_Control
 *功能说明: 云台回中控制
 *形    参: void
 *返 回 值: void
 */
static void GIMBLE_Init_Control()
{
	g_angle_target.yaw   = YAW_INIT_ANGLE;
	g_angle_target.pitch = PIT_INIT_ANGLE;
	
	GIMBLE_ENCONDE_Control(g_angle_target);
}



/*
 *函 数 名: GIMBLE_Data_Update
 *功能说明: 云台数据更新
 *形    参: void
 *返 回 值: void
 */
static void GIMBLE_Data_Update()
{
	
	if(g_rc_control.mouse.press_r)
	{
		if(Gimbal_Motor_yaw.press_r_time<R_PRESS_LONG_TIME)
		{
			Gimbal_Motor_yaw.press_r_time++;
		}
	}
	else 
	{
		Gimbal_Motor_yaw.press_r_time=0;
	}
}

/*
 *函 数 名: GIMBLE_RC_Control
 *功能说明: 云台遥控控制
 *形    参: void
 *返 回 值: void
 */
static void GIMBLE_RC_Control()
{
	//遥控角度计算
	Gimbal_Motor_yaw.rc_control_angle  = -(YAW_RC_LIMIT_ANGLE/(660.0f))*(g_rc_control.rc.ch0-1024);
	Gimbal_Motor_pit.rc_control_angle  =  (PIT_RC_LIMIT_ANGLE/(660.0f))*(g_rc_control.rc.ch1-1024);
	
	//目标角度赋值
	g_angle_target.yaw   = Gimbal_Motor_yaw.rc_control_angle+g_imu_data.count*360.0f;
	g_angle_target.pitch = Gimbal_Motor_pit.rc_control_angle+PIT_INIT_ANGLE;
	
	//角度环目标限幅赋值
	g_angle_target.yaw   = ConstrainFloat(g_angle_target.yaw,  - YAW_RC_LIMIT_ANGLE+g_imu_data.count*360.0f,YAW_RC_LIMIT_ANGLE+g_imu_data.count*360.0f);
	g_angle_target.pitch = ConstrainFloat(g_angle_target.pitch,- PIT_RC_LIMIT_ANGLE_DOWN+PIT_INIT_ANGLE,PIT_RC_LIMIT_ANGLE_UP+PIT_INIT_ANGLE);	 
	
	GIMBLE_GYRO_Control(g_angle_target);
}
/*
 *函 数 名: GIMBLE_PC_Control
 *功能说明: 云台电脑控制
 *形    参: void
 *返 回 值: void
 */
static void GIMBLE_PC_Control()
{
	
	//角度环目标限幅赋值
	g_rc_control.mouse.x_distance   = ConstrainFloat(g_rc_control.mouse.x_distance,  - YAW_PC_LIMIT_ANGLE,YAW_PC_LIMIT_ANGLE);
	g_rc_control.mouse.y_distance   = ConstrainFloat(g_rc_control.mouse.y_distance,  - PIT_PC_LIMIT_ANGLE_DOWN,PIT_PC_LIMIT_ANGLE_UP);	 
	//遥控角度计算
	Gimbal_Motor_yaw.pc_control_angle  =  g_rc_control.mouse.x_distance;
	Gimbal_Motor_pit.pc_control_angle  =  g_rc_control.mouse.y_distance;
	
	//目标角度赋值
	g_angle_target.yaw   = Gimbal_Motor_yaw.pc_control_angle;
	g_angle_target.pitch = Gimbal_Motor_pit.pc_control_angle+PIT_INIT_ANGLE;
	
	//角度环目标限幅赋值
	g_angle_target.yaw   = ConstrainFloat(g_angle_target.yaw,  - YAW_PC_LIMIT_ANGLE,YAW_PC_LIMIT_ANGLE);
	g_angle_target.pitch = ConstrainFloat(g_angle_target.pitch,- PIT_PC_LIMIT_ANGLE_DOWN+PIT_INIT_ANGLE,PIT_PC_LIMIT_ANGLE_UP+PIT_INIT_ANGLE);	 
	
	GIMBLE_GYRO_Control(g_angle_target);
	
}
/*
 *函 数 名: GIMBLE_Auto_Control
 *功能说明: 云台视觉
 *形    参: void
 *返 回 值: void
 */
static void GIMBLE_Auto_Control()
{
	Gimbal_Motor_yaw.vision_control_angle = minipc_data.get_target_angle_yaw;
	Gimbal_Motor_pit.vision_control_angle = minipc_data.get_target_angle_pit;
	
	if(Gimbal_Motor_yaw.vision_control_angle==0.0f)
	{
		g_angle_target.yaw=g_imu_data.absolute_yaw;
	}
	
	else
	{
	    g_angle_target.yaw                = Gimbal_Motor_yaw.vision_control_angle;
	}
	
	if(Gimbal_Motor_pit.vision_control_angle==0.0f)
	{
		Gimbal_Motor_pit.vision_control_angle=g_data_6623.angle[PITCH];
	}
	else
	{
		g_angle_target.pitch                  = Gimbal_Motor_pit.vision_control_angle;
	}
	
	//角度环目标限幅赋值
	g_angle_target.yaw   = ConstrainFloat(g_angle_target.yaw,  - YAW_PC_LIMIT_ANGLE,YAW_PC_LIMIT_ANGLE);
	g_angle_target.pitch = ConstrainFloat(g_angle_target.pitch,- PIT_PC_LIMIT_ANGLE_DOWN+PIT_INIT_ANGLE,PIT_PC_LIMIT_ANGLE_UP+PIT_INIT_ANGLE);	 
	
	GIMBLE_GYRO_Control(g_angle_target);
}

/*
 *函 数 名: GIMBLE_Loop_Control
 *功能说明: 云台循环控制函数
 *形    参: void
 *返 回 值: void
 */
void GIMBLE_Loop_Control()
{
	GIMBLE_Set_Mode();
	
	GIMBLE_Data_Update();
	
	if(robot_status.gimbal_mode==GIMBLE_INIT)
	{
		GIMBLE_Init_Control();
	}
	
	else if(robot_status.gimbal_mode==GIMBLE_AUTO)
	{
		GIMBLE_Auto_Control();

	}
	else if(robot_status.gimbal_mode==GIMBLE_RC)
	{
		GIMBLE_RC_Control();
	
	}
	else if(robot_status.gimbal_mode==GIMBLE_PC)
	{
		GIMBLE_PC_Control();
	
	}
}






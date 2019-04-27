#include <drive_gimble.h>
#include <drive_control.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>

#include <drive_usart.h>

static Gimbal_Motor_t Gimbal_Motor_yaw;
static Gimbal_Motor_t Gimbal_Motor_pit;



#define YAW_INIT_ANGLE 349.0f
#define PIT_INIT_ANGLE 296.0f



#define YAW_RC_LIMIT_ANGLE  90
#define PIT_RC_LIMIT_ANGLE  20

#define YAW_PC_LIMIT_ANGLE  160
#define PIT_PC_LIMIT_ANGLE  20




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
static void GIMBLE_Cali_Control(void);
static void GIMBLE_Manual_Control(void);


void GIMBLE_Init()
{
	robot_status.gimbal_mode = GIMBAL_NO;
	robot_status.motor_yaw   = MOTOR_GIMBAL_NO;
	robot_status.motor_pit   = MOTOR_GIMBAL_NO;
	robot_status.gimbal_data = GIMBAL_MOTOR_NO;
	
	//robot_status.gimbal_data=GIMBAL_MOTOR_ENCONDE;
	
	
}

/*
 *函 数 名: GIMBLE_Set_Mode
 *功能说明: 云台状态更新
 *形    参: void
 *返 回 值: void
 */

float p1=500.0f,i1=5.0f;
float p2=2.5f, i2= 0.6f,d2=0.0f,f2=0.0f;


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
         PID_SetParam(&g_infc.pid[PITCH_ANGLE],120.5,865 ,0.03,8,5000,18.73);	
		robot_status.gimbal_mode=GIMBLE_INIT;
	}
	
	//云台初始化完成
	if(robot_status.motor_yaw==MOTOR_GIMBAL_GYRO&&robot_status.motor_pit==MOTOR_GIMBAL_GYRO&&robot_status.gimbal_data!=GIMBAL_MOTOR_GYRO)
	{
	    robot_status.gimbal_data=GIMBAL_MOTOR_GYRO;
		PID_ResetParam(&g_infc.pid[YAW_SPEED]);
		PID_ResetParam(&g_infc.pid[YAW_ANGLE]);
		PID_ResetParam(&g_infc.pid[PITCH_SPEED]);
		PID_ResetParam(&g_infc.pid[PITCH_ANGLE]);
		
		
		
		PID_SetParam(&g_infc.pid[PITCH_ANGLE],1,0 ,0,0,0,0);
	
	//	PID_SetParam(&g_infc.pid[PITCH_SPEED],80,4.2,0,100,6000,0);
		PID_SetParam(&g_infc.pid[PITCH_SPEED],80,4.2,0,100,6000,0);
		

		PID_SetParam(&g_infc.pid[YAW_SPEED],  500,5,0,100,30000,0);
		PID_SetParam(&g_infc.pid[YAW_ANGLE],  2.5,0,0,0,0,0.0);
		
	}
	if(robot_status.gimbal_data==GIMBAL_MOTOR_GYRO)
	{
		

		
		if(switch_is_down(g_rc_control.rc.s1)||Gimbal_Motor_yaw.press_r_time==R_PRESS_LONG_TIME)
		{
			robot_status.gimbal_mode=GIMBLE_AUTO;
		}
		if(switch_is_up(g_rc_control.rc.s1)||switch_is_mid(g_rc_control.rc.s1))
		{
			robot_status.gimbal_mode=GIMBLE_MANUAL;
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
	
	Angle_6623_Control(g_angle_target);
}

/*
 *函 数 名: GIMBLE_Cali_Control
 *功能说明: 云台陀螺仪校准
 *形    参: void
 *返 回 值: void
 */
static void GIMBLE_Cali_Control()
{
	
	
}


/*
 *函 数 名: GIMBLE_Manual_Control
 *功能说明: 云台手动控制
 *形    参: void
 *返 回 值: void
 */
static void GIMBLE_Manual_Control()
{
	//云台底盘遥控器控制
	if(switch_is_up(g_rc_control.rc.s1))
	{
		GIMBLE_RC_Control();
	}
	if(switch_is_mid(g_rc_control.rc.s1))
	{
		GIMBLE_PC_Control();
	}
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
	g_angle_target.yaw   = Gimbal_Motor_yaw.rc_control_angle;
	g_angle_target.pitch = Gimbal_Motor_pit.rc_control_angle;
	
	//角度环目标限幅赋值
	g_angle_target.yaw   = ConstrainFloat(g_angle_target.yaw,  - YAW_RC_LIMIT_ANGLE,YAW_RC_LIMIT_ANGLE);
	g_angle_target.pitch = ConstrainFloat(g_angle_target.pitch,- PIT_RC_LIMIT_ANGLE,PIT_RC_LIMIT_ANGLE);	 
	
	Angle_6623_Control(g_angle_target);
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
	g_rc_control.mouse.y_distance   = ConstrainFloat(g_rc_control.mouse.y_distance,  - PIT_PC_LIMIT_ANGLE,PIT_PC_LIMIT_ANGLE);	 
	//遥控角度计算
	Gimbal_Motor_yaw.pc_control_angle  =  g_rc_control.mouse.x_distance;
	Gimbal_Motor_pit.pc_control_angle  =  g_rc_control.mouse.y_distance;
	
	//目标角度赋值
	g_angle_target.yaw   = Gimbal_Motor_yaw.pc_control_angle;
	g_angle_target.pitch = Gimbal_Motor_pit.pc_control_angle;
	
	//角度环目标限幅赋值
	g_angle_target.yaw   = ConstrainFloat(g_angle_target.yaw,  - YAW_PC_LIMIT_ANGLE,YAW_PC_LIMIT_ANGLE);
	g_angle_target.pitch = ConstrainFloat(g_angle_target.pitch,- PIT_PC_LIMIT_ANGLE,PIT_PC_LIMIT_ANGLE);	 
	
	Angle_6623_Control(g_angle_target);
	
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
	
	g_angle_target.yaw                    = Gimbal_Motor_yaw.vision_control_angle;
	g_angle_target.pitch                  = Gimbal_Motor_pit.vision_control_angle;
	
	//角度环目标限幅赋值
	g_angle_target.yaw   = ConstrainFloat(g_angle_target.yaw,  - YAW_PC_LIMIT_ANGLE,YAW_PC_LIMIT_ANGLE);
	g_angle_target.pitch = ConstrainFloat(g_angle_target.pitch,- PIT_PC_LIMIT_ANGLE,PIT_PC_LIMIT_ANGLE);	 
	
	Angle_6623_Control(g_angle_target);
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
	
	else if(robot_status.gimbal_mode==GIMBLE_MANUAL)
	{
		GIMBLE_Manual_Control();
	
	}
}
/*
 *函 数 名: Get_GIMBLE_data
 *功能说明: 获取云台 YAW 轴 角速度及角度信息
 *形    参: CAN数据结构体
 *返 回 值: void
 */

void Get_GIMBLE_data(CanRxMsg rx_message)
{
	switch(rx_message.StdId)
    {
		
      case 0x208:
      {
		  
		  g_data_6623.speed[YAW]   = HEX_TO_Float(&rx_message.Data[0]);
		  g_data_6623.speed[PITCH] = HEX_TO_Float(&rx_message.Data[4]);
		  robot_status.motor_yaw=MOTOR_GIMBAL_GYRO;
          break;
      }
	  
	  case 0x209:
	  {
		  g_data_6623.angle[YAW]   = HEX_TO_Float(&rx_message.Data[0]);
		  g_data_6623.angle[PITCH] = HEX_TO_Float(&rx_message.Data[4]);
		  robot_status.motor_pit=MOTOR_GIMBAL_GYRO;
		 
          break;
	  }
	  
      default:
      break;
    } 
}



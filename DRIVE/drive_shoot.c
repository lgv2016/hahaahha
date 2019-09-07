#include <drive_shoot.h>
#include <drive_control.h>
#include <drive_judge.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>

#include <math_tool.h>

#include <drive_usart.h>

#include <bsp_dma.h>

#define RAPID_FIRE_TIME 5  //S
#define LEVE1_HEAT  180
#define LEVE2_HEAT  300
#define LEVE3_HEAT  420


#define PRESS_LONG_TIME     (200/SHOOT_CONTROL_CYCLE)    //左键按下时间
#define NO_PRESS_LONG_TIME  (150/SHOOT_CONTROL_CYCLE)    //左键未按时间
#define BLOCK_LONG_TIME     (2000/SHOOT_CONTROL_CYCLE)   //堵转时间
#define KEY_E_TIME          (50/SHOOT_CONTROL_CYCLE)    //E键按下时间

#define KEY_Q_TIME          (20/SHOOT_CONTROL_CYCLE)    //Q键按下时间
#define NO_KEY_Q_TIME       (100/SHOOT_CONTROL_CYCLE)    //Q键未按下时间

#ifdef   RED1
#define SHOOT_MAX_SPEED      1300
#endif


#ifdef RED2
#define SHOOT_MAX_SPEED     1360

#endif






#define RC_SW_UP   ((uint16_t)1)
#define RC_SW_MID  ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)


#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s)  (s == RC_SW_MID)
#define switch_is_up(s)   (s == RC_SW_UP)

static fric_motor_t s_fric_motor1;
static fric_motor_t s_fric_motor2;




static shoot_motor_t s_shoot_motor;

static void SHOOT_Set_Mode(void);
static float AK47_Speed_Cal(void);


void SHOOT_Init()
{
	g_data_2006.angle_reset_flag=1;
	
	robot_status.shoot_mode=SHOOT_NO;
	robot_status.fric_status=FRIC_OFF;

	
}
/*
 *函 数 名: SHOOT_Set_Mode
 *功能说明: 射击状态更新
 *形    参: void
 *返 回 值: void
 */
static void SHOOT_Set_Mode()
{
	static u8 last_s=RC_SW_UP;
	
		
	 //上拨判断， 一次单发
	if((switch_is_up(g_rc_control.rc.s2)&&!switch_is_up(last_s))||(s_shoot_motor.press_l&&!s_shoot_motor.last_press_l))
	{
		
		//
		AK47_Speed_Cal();
		if(robot_status.fric_status==FRIC_OFF&&s_shoot_motor.key_time==KEY_E_TIME)	
	    {
			robot_status.shoot_mode=SHOOT_READY;
			robot_status.fric_status=FRIC_ON_START;
			
	    }
		if(robot_status.fric_status==FRIC_ON)
		{
			if(!s_shoot_motor.shoot_flag)
			{
			robot_status.shoot_mode=SHOOT_AWM;
			g_data_2006.angle_reset_flag=1;
	        g_data_2006.count=0;
			}
		}
	}
	
	//连发
	else if((s_shoot_motor.press_l_time==PRESS_LONG_TIME)||(switch_is_down(g_rc_control.rc.s2)))
	{
		if(robot_status.fric_status==FRIC_OFF&&s_shoot_motor.key_time==KEY_E_TIME)
	    {
			robot_status.shoot_mode=SHOOT_READY;
			robot_status.fric_status=FRIC_ON_START;
	    }
		
		if(robot_status.fric_status==FRIC_ON)
		{
			robot_status.shoot_mode=SHOOT_AK47;
		}
	}
	
	else if((switch_is_mid(g_rc_control.rc.s2))&&s_shoot_motor.no_press_l_time==NO_PRESS_LONG_TIME)
	{
		if((robot_status.fric_status==FRIC_ON)&&(s_shoot_motor.key_time==KEY_E_TIME))
		{
			//开始关闭摩擦轮 停止射击
			//1 开启 2 关闭
			robot_status.fric_status=FRIC_OFF_START;
		}
		robot_status.shoot_mode=SHOOT_STOP;
	}
	
	last_s=g_rc_control.rc.s2;
	
}


/*
 *函 数 名: SHOOT_Data_Update
 *功能说明: 射击控制数据更新
 *形    参: void
 *返 回 值: void
 */

static void SHOOT_Data_Update()
{
	s_shoot_motor.last_press_l=s_shoot_motor.press_l;
	s_shoot_motor.press_l=g_rc_control.mouse.press_l;
	
	s_shoot_motor.last_press_r=s_shoot_motor.press_r;
	s_shoot_motor.press_r=g_rc_control.mouse.press_r;
	
	
	
	if(s_shoot_motor.press_l)
	{
		if(s_shoot_motor.press_l_time<PRESS_LONG_TIME)
		{
			s_shoot_motor.press_l_time++;
		}
		s_shoot_motor.no_press_l_time=0;
	}
	else
	{
		if(s_shoot_motor.no_press_l_time<NO_PRESS_LONG_TIME)
		{
			s_shoot_motor.no_press_l_time++;
		}
		s_shoot_motor.press_l_time=0;
	}
	
	if(g_rc_control.key.k[E])
	{
		if(s_shoot_motor.key_time<KEY_E_TIME)
		{
			s_shoot_motor.key_time++;
		}
	}
	
	else
	{
		s_shoot_motor.key_time=0;
	}
	//////////////////////////////////////
	if(g_rc_control.key.k[Q])
	{
		if(s_shoot_motor.key_q_time<KEY_Q_TIME)
		{
			s_shoot_motor.key_q_time++;
		}
	}
	
	else
	{
		s_shoot_motor.key_q_time=0;
	}
	/////////////////////////////////////
	if(!g_rc_control.key.k[Q])
	{
		if(s_shoot_motor.no_key_q_time<NO_KEY_Q_TIME)
		{
			s_shoot_motor.no_key_q_time++;
		}
	}
	
	else
	{
		s_shoot_motor.no_key_q_time=0;
	}
	
	
	if(s_shoot_motor.no_key_q_time==NO_KEY_Q_TIME)
	{
		s_shoot_motor.last_key_q=0;
	}
	
	////////////////////////////装甲板切换
	if(s_shoot_motor.key_q_time==KEY_Q_TIME&&robot_status.enemy_armor==ARMOR_BIG&&!s_shoot_motor.last_key_q)
	{
		s_shoot_motor.last_key_q=1;
		robot_status.enemy_armor=ARMOR_SMALL;
		if(s_shoot_motor.press_r)
		{
			MiniPC_Send_Data(0x01);
		}
		 
		Cmd_Judge_ESC();
	}
	
	if(s_shoot_motor.key_q_time==KEY_Q_TIME&&robot_status.enemy_armor==ARMOR_SMALL&&!s_shoot_motor.last_key_q)
	{
		s_shoot_motor.last_key_q=1;
		robot_status.enemy_armor=ARMOR_BIG;
		if(s_shoot_motor.press_r)
		{
			MiniPC_Send_Data(0x01);
		}
		Cmd_Judge_ESC();
	}
	////////////////////////////////////
	
	if(s_shoot_motor.press_r&&!s_shoot_motor.last_press_r)
	{
		MiniPC_Send_Data(0x01);
	}
	
	if(!s_shoot_motor.press_r&&s_shoot_motor.last_press_r)
	{
		u8 i;
		MiniPC_Send_Data(0x01);
		
		for(i=0;i<DMA_MiniPC_Reve_Buff_Size;i++)
		{
			g_DMA_MiniPC_Reve_Buff[i]=0x00;
		}
		
	}
	
	
	/////弹仓开启关闭
	
	if(g_rc_control.key.k[V])  //开启
	{
		robot_status.depot_status=DEPOT_ON;
	}
	
	if(g_rc_control.key.k[B])   //关闭
	{
		robot_status.depot_status=DEPOT_OFF;
	}
	
	
}
/*
 *函 数 名: SHOOT_Stop_Control
 *功能说明: 使2006无力
 *形    参: void
 *返 回 值: void
 */
static void SHOOT_Stop_Control()
{
	Cmd_2006_ESC(0);
}
/*
 *函 数 名: SHOOT_AWM_Control
 *功能说明: 单发发射击控制
 *形    参: 单发角度
 *返 回 值: void
 */
static void SHOOT_AWM_Control(float set_angle)
{	
	g_angle_target.shoot=set_angle;
	Angle_2006_Control(g_angle_target);
}
/*
 *函 数 名: Shoot_AK47_Control
 *功能说明: 连发射击控制
 *形    参: 连发速度
 *返 回 值: void
 */
static void SHOOT_AK47_Control()
{
	g_speed_target.shoot=AK47_Speed_Cal();
	Speed_2006_Control(g_speed_target);	
}
/*
 *函 数 名: Shoot_Ready_Control
 *功能说明: 射击准备控制，向云台发送打开摩擦轮的命令
 *形    参: void
 *返 回 值: void
 */
static void SHOOT_Ready_Control()
{
	robot_status.fric_status=FRIC_ON_START;
	//开始开启摩擦轮
	//Cmd_Fric_ESC(1);
}

/*
 *函 数 名: SHOOT_Loop_Control
 *功能说明: 射击循环控制
 *形    参: void
 *返 回 值: void
 */
void SHOOT_Loop_Control()
{
	SHOOT_Set_Mode();
	SHOOT_Data_Update();
	Fric_Motor_Speed_Control();
	
	if(robot_status.shoot_mode==SHOOT_AWM)
	{
		SHOOT_AWM_Control(60);
	}
	else if(robot_status.shoot_mode==SHOOT_AK47)
	{
		
		    SHOOT_AK47_Control();
		
	}
	else if(robot_status.shoot_mode==SHOOT_STOP)
	{
		SHOOT_Stop_Control();
	}
	else if(robot_status.shoot_mode==SHOOT_READY)
	{
		SHOOT_Ready_Control();
	}
}




/*
 *函 数 名: Fric_Init
 *功能说明: 摩擦轮结构体初始化
 *形    参: 摩擦轮结构体
 *返 回 值: void
 */
static void Fric_Init(fric_motor_t *fric_motor)
{
	fric_motor->input  = 100.0f;
	fric_motor->out    = 1000.0f;
	fric_motor->min    = 1000;
	fric_motor->max    = SHOOT_MAX_SPEED;
	fric_motor->period = 0.001f;
}
/*
 *函 数 名: Fric_Control
 *功能说明: 摩擦轮结构体输出循环赋值
 *形    参: 摩擦轮结构体
 *返 回 值: void
 */
static void Fric_Control(fric_motor_t *fric_motor)
{
	if(robot_status.fric_status==FRIC_ON_START)
	{
		fric_motor->out+=fric_motor->input*fric_motor->period*0.8f*SHOOT_CONTROL_CYCLE;
		
		if(fric_motor->out>fric_motor->max)
		{
			fric_motor->out=fric_motor->max;
		}
	}
	
	if(robot_status.fric_status==FRIC_OFF_START)
	{
		fric_motor->out-=fric_motor->input*fric_motor->period*0.7f*SHOOT_CONTROL_CYCLE;
		if(fric_motor->out<fric_motor->min)
		{
			fric_motor->out=fric_motor->min;
		}
	}
}
/*
 *函 数 名: Fric_Motor_Speed_Init
 *功能说明: 摩擦轮初始化
 *形    参: void
 *返 回 值: void
 */
void Fric_Motor_Speed_Init()
{
	Fric_Init(&s_fric_motor1);
	Fric_Init(&s_fric_motor2);
	
	TIM_SetCompare1(TIM5,1000-1);
    TIM_SetCompare2(TIM5,1000-1);
}

/*函 数 名: Fric_Motor_Speed_Control
 *功能说明: 摩擦轮速度控制
 *形    参: void
 *返 回 值: void
 */
void Fric_Motor_Speed_Control()
{
	Fric_Control(&s_fric_motor1);
	
	if(robot_status.fric_status==FRIC_ON_START)
	{
		if(s_fric_motor1.out==s_fric_motor1.max)
		{
			Fric_Control(&s_fric_motor2);
		}
		
		if(s_fric_motor2.out==s_fric_motor2.max)
		{
			robot_status.fric_status=FRIC_ON;
		//	Cmd_Judge_ESC();
		}
	}
	
	if(robot_status.fric_status==FRIC_OFF_START)
	{
		if(s_fric_motor1.out==s_fric_motor1.min)
		{
			Fric_Control(&s_fric_motor2);
		}
		if(s_fric_motor2.out==s_fric_motor2.min)
		{
			robot_status.fric_status=FRIC_OFF;
			//Cmd_Judge_ESC();
		}
	}
	TIM_SetCompare1(TIM5,(u16)s_fric_motor1.out);
	TIM_SetCompare2(TIM5,(u16)s_fric_motor2.out);
}


static float AK47_Speed_Cal()
{
	float shoot_frequency;
	float shoot_speed;   
	float shoot_target;
	u16 cool_rate;
	
	cool_rate=judge_data.shoot_rate;
	if(judge_data.robot_level==1)
	{
		if(judge_data.shooter_heat0>=LEVE1_HEAT)
		{
			shoot_frequency=cool_rate/30.0f;
			s_shoot_motor.shoot_flag=1;
		}
		else
		{
			shoot_frequency=(cool_rate/30.0f)+2.0f;
			s_shoot_motor.shoot_flag=0;
		}
	}
	if(judge_data.robot_level==2)
	{
		if(judge_data.shooter_heat0>=LEVE2_HEAT)
		{
			shoot_frequency=cool_rate/30.0f;
			s_shoot_motor.shoot_flag=1;
		}
		else
		{
			shoot_frequency=(cool_rate/30.0f)+2.8f;
			s_shoot_motor.shoot_flag=0;
		}
	}
	
	if(judge_data.robot_level==3)
	{
		if(judge_data.shooter_heat0>=LEVE3_HEAT)
		{
			shoot_frequency=cool_rate/30.0f;
			s_shoot_motor.shoot_flag=1;
		}
		else
		{
			shoot_frequency=(cool_rate/30.0f)+4.0f;
			s_shoot_motor.shoot_flag=0;
		}
	}
	
	shoot_speed=shoot_frequency*0.5236f;
	shoot_target=(shoot_speed*36.0f*60.0f)/(2*M_PI);
	
	
	shoot_target=Constrainfloat(shoot_target,-3000,3000);
	return shoot_target;
}

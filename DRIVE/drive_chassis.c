#include <drive_chassis.h>
#include <drive_control.h>
#include <drive_delay.h> 
#include <drive_imu.h>
#include <motor_chassis.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>

#define RC_SW_UP   ((uint16_t)1)
#define RC_SW_MID  ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)


#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s)  (s == RC_SW_MID)
#define switch_is_up(s)   (s == RC_SW_UP)

#define KEY_NO_Q_TIME £¨80/SHOOT_CONTROL_CYCLE£©

#define CHASSIS_MAX_SPEED_X 2000
#define CHASSIS_MAX_SPEED_Y 2000

#define CHASSIS_MAX_SPEED_W 50

#define CHASSIS_ADD_SPEEED 1000

chassis_move_t chassis_move;

static void INC_SetParam(INC_t* inc, float input, float max, float min, float out,float period);
static void INC_Set_Value(INC_t* inc,int16_t measure,int16_t target);
static void INC_Cal(INC_t* inc,int16_t measure,int16_t target);

static void CHASSIS_Follow_Gimble_Control(float target);
static void CHASSIS_SPEED_Control(int16_t vx,int16_t vy,int16_t wz);
static void CHASSIS_ROTATE_Control(void);


void CHASSIS_Init()
{
	INC_SetParam(&g_infc.inc[CH_ROTATE_SPEED],80,100,-100,0,0.001);
	
}


static void CHASSIS_Set_Mode()
{
	
	if(switch_is_up(g_rc_control.rc.s1))
	{
		robot_status.chassis_mode=CHASSIS_SPEED;
	}
	
	else
	{
		if(g_rc_control.key.k[Q]&&robot_status.chassis_mode==CHASSIS_STOP)
		{
			robot_status.chassis_mode=CHASSIS_ROTATE;
		}
		else if(g_rc_control.key.k[CTRL]&&robot_status.chassis_mode==CHASSIS_STOP)
		{
			robot_status.chassis_mode=CHASSIS_FOLLOW_GIMBLE;
		}
		
		else if((g_rc_control.key.k[W]||g_rc_control.key.k[A]||g_rc_control.key.k[S]||g_rc_control.key.k[D]))
		{
			robot_status.chassis_mode=CHASSIS_SPEED;
		}
		
		else 
		{
			robot_status.chassis_mode=CHASSIS_STOP;
			chassis_move.inc_cal_flag=1;
		}
	}
	
}


void CHASSIS_Loop_Control()
{
	
	CHASSIS_Set_Mode();
	if(robot_status.chassis_mode==CHASSIS_STOP)
	{
		Speed_Chassis_Control(0,0,0);
	}
	if(robot_status.chassis_mode==CHASSIS_SPEED)
	{
		if(switch_is_up(g_rc_control.rc.s1))
		{
			CHASSIS_RC_Control(CHASSIS_MAX_SPEED_X,CHASSIS_MAX_SPEED_Y);
		}
		else
		{
			CHASSIS_SPEED_Control(CHASSIS_MAX_SPEED_X,CHASSIS_MAX_SPEED_Y,CHASSIS_MAX_SPEED_W);
		}
	}
	
	if(robot_status.chassis_mode==CHASSIS_FOLLOW_GIMBLE)
	{
		CHASSIS_Follow_Gimble_Control(g_angle_target.yaw);
	}
	
	if(robot_status.chassis_mode==CHASSIS_ROTATE)
	{
		CHASSIS_ROTATE_Control();
	}
}

void CHASSIS_RC_Control(int16_t maxspeedx,int16_t maxspeedy)
{
	chassis_move.rc_control_speedx= (maxspeedx/(660.0f))*(g_rc_control.rc.ch2-1024);
	chassis_move.rc_control_speedy= (maxspeedx/(660.0f))*(g_rc_control.rc.ch3-1024);
	Speed_Chassis_Control(chassis_move.rc_control_speedx,chassis_move.rc_control_speedy,0);
}

static void CHASSIS_Follow_Gimble_Control(float target)
{
	g_angle_target.ch_rotate=target;
	
	Angle_Rotate_Control(g_angle_target);
}

static void CHASSIS_ROTATE_Control()
{
	if(g_rc_control.key.k[Q]&&robot_status.chassis_mode==CHASSIS_STOP)
	{
		if(chassis_move.inc_cal_flag==1)
		{
			chassis_move.inc_cal_flag=0;
			INC_Set_Value(&g_infc.inc[CH_ROTATE_SPEED],0,100);
		}
		
		INC_Cal(&g_infc.inc[CH_ROTATE_SPEED],0,100);
		g_speed_target.ch_rotate=g_infc.inc[CH_ROTATE_SPEED].out;
	}
	
	else if(!g_rc_control.key.k[Q])
	{
		g_speed_target.ch_rotate=0;
		robot_status.chassis_mode=CHASSIS_STOP;
	}
	Speed_Rotate_Control(g_speed_target);
}

static void CHASSIS_SPEED_Control(int16_t vx,int16_t vy,int16_t wz)
{
	int16_t speedx=0,speedy=0,speedw=0;
	
	if(g_rc_control.key.k[W]==1)
	{
		speedy=vy;
	}

	if(g_rc_control.key.k[A]==1)
	{
		speedx=vx;
	}
	
	if(g_rc_control.key.k[S]==1)
	{
		speedy=-vy;
	}
	
	if(g_rc_control.key.k[D]==1)
	{
		speedx=-vx;
	}
	
	if(g_rc_control.key.k[Q]==1)
	{
		speedw=wz;
	}
	
	Speed_Chassis_Control(vx,vy,wz);
}

static void INC_SetParam(INC_t* inc, float input, float max, float min, float out,float period)
{
	inc->input=input;
	inc->max=max;
	inc->min=min;
	inc->out=out;
	inc->period=period;
}


void INC_Set_Value(INC_t* inc,int16_t measure,int16_t target)
{
	if(target>measure)
	{
		inc->out=measure;
		inc->max=target;
	}
	else
	{
		inc->out=measure;
		inc->min=target;
	}
}

void INC_Cal(INC_t* inc,int16_t measure,int16_t target)
{

	if(target>measure)
	{
		inc->out+=inc->input*inc->period;
	}
	
	else if(target<measure)
	{
		inc->out-=inc->input*inc->period;
	}
	
	
	if(inc->out>inc->max)
	{
		inc->out=inc->max;
	}
	
	else if(inc->out<inc->min)
	{
		inc->out=inc->min;
	}
}
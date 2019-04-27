#include <drive_control.h>
#include <drive_delay.h> 
#include <drive_imu.h>

#include <motor_chassis.h>
#include <motor_cradle_head.h>

#include <robotstatus.h>
#include <math_tool.h>

#include <drive_gimble.h>


#define CHASSIS_W   19.8785   //cm
#define CHASSIS_L   17.5375   //cm

#define YAW_INIT_ERROR 0.5f
#define PIT_INIT_ERROR 0.5f



infantry_control_t g_infc;


object_t g_speed_target;
object_t g_angle_target;

static object_t s_angle_contorl_out;
static object_t s_speed_contorl_out;


static void PID_Reset(void)
{
	
    PID_SetParam(&g_infc.pid[SHOOT_SPEED],3.43,38.15,0.02,1000,5000,14.2);
    PID_SetParam(&g_infc.pid[SHOOT_ANGLE],100,0,0,0,0,0);
	
	
	PID_SetParam(&g_infc.pid[YAW_ANGLE],  45,0,0.33,0,0,50);
	PID_SetParam(&g_infc.pid[YAW_SPEED],  120,2,0,100,20000,0);

//	PID_SetParam(&g_infc.pid[YAW_SPEED],  40,3,0,100,20000,0);
	
	
	PID_SetParam(&g_infc.pid[LF_SPEED],   3.26, 26.64,0.06,1000,5000,61.3);
	PID_SetParam(&g_infc.pid[LA_SPEED],   3.26, 26.64,0.06,1000,5000,61.3);
	PID_SetParam(&g_infc.pid[RF_SPEED],   3.26, 26.64,0.06,1000,5000,61.3);
	PID_SetParam(&g_infc.pid[RA_SPEED],   3.26, 26.64,0.06,1000,5000,61.3);
	PID_SetParam(&g_infc.pid[CH_ROTATE_SPEED],0.09,9.15,0,200,120,0);
	PID_SetParam(&g_infc.pid[CH_ROTATE_ANGLE],2.2,0,0,0,0,0);
	
}

void Infan_Control_Init(void)
{
	PID_Reset();
	
}

void Angle_6623_Control(object_t target)
{
	
	static u8 error_temp=0;
	//函数运算间隔计算
    static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	//
	g_infc.angle_outer_target.yaw=target.yaw;
	
	
	//计算角度控制误差
	g_infc.angle_outer_error.yaw  	= g_infc.angle_outer_target.yaw-g_data_6623.angle[YAW];
    g_infc.angle_outer_error.yaw  	= ApplyDeadbandFloat(g_infc.angle_outer_error.yaw,0.1f);

	
	
	//PID算法，计算出角度环的控制量
    s_angle_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_ANGLE],g_infc.angle_outer_error.yaw,deltaT);
	//输出限幅
    s_angle_contorl_out.yaw         	= ConstrainFloat(s_angle_contorl_out.yaw,-66,66);
	
	
	
	
	g_infc.angle_outer_target.pitch     = target.pitch;
	g_infc.angle_outer_error.pitch      = g_infc.angle_outer_target.pitch - g_data_6623.angle[PITCH];
	g_infc.angle_outer_error.pitch  	= ApplyDeadbandFloat(g_infc.angle_outer_error.pitch,0.5f);
	
	
	if(robot_status.gimbal_mode==GIMBLE_INIT)
	{
		if((abs(g_infc.angle_outer_error.yaw)<YAW_INIT_ERROR)&&(abs(g_infc.angle_outer_error.pitch)<PIT_INIT_ERROR))
		{
			error_temp++;
			if(error_temp>80)
			{
				Cmd_GIMBAL_ESC(1,0);
				GPIO_ResetBits(GPIOG, GPIO_Pin_1);
				error_temp=0;
			}
		}
	}
	
	
	//使用陀螺仪数据
	if(robot_status.gimbal_data==GIMBAL_MOTOR_GYRO)
	{
		s_angle_contorl_out.pitch         	= PID_GetPID(&g_infc.pid[PITCH_ANGLE],g_infc.angle_outer_error.pitch,deltaT);
		s_angle_contorl_out.pitch         	= ConstrainFloat(s_angle_contorl_out.pitch,-35,35);
	}
	
	//使用电机编码器数据
	if(robot_status.gimbal_data==GIMBAL_MOTOR_ENCONDE) 
	{
		s_angle_contorl_out.pitch         	= PID_GetPID(&g_infc.pid[PITCH_ANGLE],g_infc.angle_outer_error.pitch,deltaT);
		s_angle_contorl_out.pitch         	= ConstrainFloat(s_angle_contorl_out.pitch,-5000,5000);
	}
	
	
	Speed_6623_Control(s_angle_contorl_out);
}


void Speed_6623_Control(object_t target)
{
	//函数运算间隔计算
	static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	//速度环目标赋值
    g_infc.speed_inner_target.yaw		= target.yaw;
	//计算速度环控制误差
    g_infc.speed_inner_error.yaw  	    = g_infc.speed_inner_target.yaw - g_data_6623.speed[YAW];
	//死区控制
    g_infc.speed_inner_error.yaw  	    = ApplyDeadbandFloat( g_infc.speed_inner_error.yaw,0);
	//PID算法，计算出速度环的控制量
    s_speed_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_SPEED],g_infc.speed_inner_error.yaw,1);
	//输出限幅
    s_speed_contorl_out.yaw         	= ConstrainFloat(s_speed_contorl_out.yaw,-30000,30000);
	

	
	if(robot_status.gimbal_data==GIMBAL_MOTOR_GYRO)
	{
		//速度环目标赋值
		g_infc.speed_inner_target.pitch		= target.pitch;
		//计算速度环控制误差
		g_infc.speed_inner_error.pitch  	= g_infc.speed_inner_target.pitch - g_data_6623.speed[PITCH];
		//死区控制
		g_infc.speed_inner_error.pitch  	= ApplyDeadbandFloat( g_infc.speed_inner_error.pitch,0);
		//PID算法，计算出速度环的控制量
		s_speed_contorl_out.pitch         	= PID_GetPID(&g_infc.pid[PITCH_SPEED],g_infc.speed_inner_error.pitch,1);
		//输出限幅
		
		s_speed_contorl_out.pitch         	= ConstrainFloat(s_speed_contorl_out.pitch,-6000,6000);
		
		Cmd_6623_ESC(s_speed_contorl_out.yaw,-s_speed_contorl_out.pitch);
	
	}
	
	
	if(robot_status.gimbal_data==GIMBAL_MOTOR_ENCONDE) 
	{
		Cmd_6623_ESC(s_speed_contorl_out.yaw,-s_angle_contorl_out.pitch);
	}
}


void Speed_3510_Control(object_t target)
{
	//函数运算间隔计算
   	static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	//速度环目标赋值
	g_infc.speed_inner_target.lf    = target.lf;
	g_infc.speed_inner_target.la    = target.la;
	g_infc.speed_inner_target.rf    = target.rf;
	g_infc.speed_inner_target.ra    = target.ra;
	
	//计算速度环控制误差
	g_infc.speed_inner_error.lf     = g_infc.speed_inner_target.lf      - g_data_3510.speed[LF];
	g_infc.speed_inner_error.la     = g_infc.speed_inner_target.la      - g_data_3510.speed[LA];
	g_infc.speed_inner_error.rf     = g_infc.speed_inner_target.rf      - g_data_3510.speed[RF];
	g_infc.speed_inner_error.ra     = g_infc.speed_inner_target.ra      - g_data_3510.speed[RA];
	
	//死区控制
	g_infc.speed_inner_error.lf     = ApplyDeadbandFloat( g_infc.speed_inner_error.lf,5);
	g_infc.speed_inner_error.la     = ApplyDeadbandFloat( g_infc.speed_inner_error.la,5);
	g_infc.speed_inner_error.rf     = ApplyDeadbandFloat( g_infc.speed_inner_error.rf,5);
	g_infc.speed_inner_error.ra     = ApplyDeadbandFloat( g_infc.speed_inner_error.ra,5);
	
	//PID算法，计算出速度环的控制量
	s_speed_contorl_out.lf          = PID_GetPID(&g_infc.pid[LF_SPEED],g_infc.speed_inner_error.lf,deltaT);
	s_speed_contorl_out.la          = PID_GetPID(&g_infc.pid[LA_SPEED],g_infc.speed_inner_error.la,deltaT);
	s_speed_contorl_out.rf          = PID_GetPID(&g_infc.pid[RF_SPEED],g_infc.speed_inner_error.rf,deltaT);
	s_speed_contorl_out.ra          = PID_GetPID(&g_infc.pid[RA_SPEED],g_infc.speed_inner_error.ra,deltaT);
	
	//输出限幅
	s_speed_contorl_out.lf          = ConstrainFloat(s_speed_contorl_out.lf,-6666,6666);
	s_speed_contorl_out.la          = ConstrainFloat(s_speed_contorl_out.la,-6666,6666);
	s_speed_contorl_out.rf          = ConstrainFloat(s_speed_contorl_out.rf,-6666,6666);
	s_speed_contorl_out.ra          = ConstrainFloat(s_speed_contorl_out.ra,-6666,6666);
	
	//can发送控制量
	Cmd_3510_ESC(s_speed_contorl_out.lf,s_speed_contorl_out.la,s_speed_contorl_out.rf,s_speed_contorl_out.ra);
}

void Speed_2006_Control(object_t target)
{
	//函数运算间隔计算
	static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	//速度环目标赋值
    g_infc.speed_inner_target.shoot		= target.shoot;
	
	//计算速度环控制误差
    g_infc.speed_inner_error.shoot  	= g_infc.speed_inner_target.shoot - g_data_2006.speed;
	
	//死区控制
    g_infc.speed_inner_error.shoot  	= ApplyDeadbandFloat( g_infc.speed_inner_error.shoot,10);
	
	//PID算法，计算出速度环的控制量
    s_speed_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
	
	//输出限幅
    s_speed_contorl_out.shoot         	= ConstrainFloat(s_speed_contorl_out.shoot,-6666,6666);
	
	//can发送控制量
	Cmd_2006_ESC(s_speed_contorl_out.shoot);
	//Cmd_2006_ESC(0);
}

void Angle_2006_Control(object_t target)
{
	//函数运算间隔计算
    static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	//角度环目标赋值
	g_infc.angle_outer_target.shoot		= target.shoot;
	
	//计算角度控制误差
	g_infc.angle_outer_error.shoot  	= g_infc.angle_outer_target.shoot-g_data_2006.angle;
	
	//死区控制
    g_infc.angle_outer_error.shoot  	= ApplyDeadbandFloat(g_infc.angle_outer_error.shoot,0.1);
	
	//PID算法，计算出角度环的控制量
    s_angle_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,deltaT);
	
	//输出限幅
    s_angle_contorl_out.shoot         	= ConstrainFloat(s_angle_contorl_out.shoot,-6666,6666);
	
	//将角度外环控制量作为速度内环的控制目标
	Speed_2006_Control(s_angle_contorl_out);
}
/*
*Y W
*|<-
*| <-
*|__________x
*/
void Speed_Chassis_Control(float vx,float vy,float wz)    //底盘速度控制
{
	g_speed_target.rf=-(vy-vx+wz*(CHASSIS_L+CHASSIS_W));
	g_speed_target.lf=  vy+vx-wz*(CHASSIS_L+CHASSIS_W);
	g_speed_target.la=  vy-vx-wz*(CHASSIS_L+CHASSIS_W);
	g_speed_target.ra=-(vy+vx+wz*(CHASSIS_L+CHASSIS_W));
	Speed_3510_Control(g_speed_target);
}

void Speed_Rotate_Control(object_t target)       //自旋speed
{
	static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	//速度环目标赋值
	g_infc.speed_inner_target.ch_rotate=target.ch_rotate;
	
	//计算速度环控制误差
    g_infc.speed_inner_error.ch_rotate  	= g_infc.speed_inner_target.ch_rotate-g_imu_data.yaw_speed;
	
	//死区控制
    g_infc.speed_inner_error.ch_rotate  	= ApplyDeadbandFloat( g_infc.speed_inner_error.ch_rotate,0);
	
	//PID算法，计算出速度环的控制量
    s_speed_contorl_out.ch_rotate         	= PID_GetPID(&g_infc.pid[CH_ROTATE_SPEED],g_infc.speed_inner_error.ch_rotate,deltaT);
	
	//输出限幅
    s_speed_contorl_out.ch_rotate         	= ConstrainFloat(s_speed_contorl_out.ch_rotate,-150,150);
	
	Speed_Chassis_Control(0,0,s_speed_contorl_out.ch_rotate);
	
}

void Angle_Rotate_Control(object_t target)         //
{
    static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	//角度环目标赋值
	g_infc.angle_outer_target.ch_rotate		= target.ch_rotate;
	
	//计算角度控制误差
	g_infc.angle_outer_error.ch_rotate  	= g_infc.angle_outer_target.ch_rotate-g_imu_data.yaw;
	
	//死区控制
    g_infc.angle_outer_error.ch_rotate  	= ApplyDeadbandFloat(g_infc.angle_outer_error.ch_rotate,1);
	
	//PID算法，计算出角度环的控制量
    s_angle_contorl_out.ch_rotate         	= PID_GetPID(&g_infc.pid[CH_ROTATE_ANGLE],g_infc.angle_outer_error.ch_rotate,deltaT);
	
	//输出限幅
    s_angle_contorl_out.ch_rotate         	= ConstrainFloat(s_angle_contorl_out.ch_rotate,-66,66);
	
	//将角度外环控制量作为速度内环的控制目标
	Speed_Rotate_Control(s_angle_contorl_out);
}

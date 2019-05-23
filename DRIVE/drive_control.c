#include <drive_control.h>
#include <drive_delay.h> 
#include <drive_imu.h>

#include <motor_chassis.h>
#include <motor_cradle_head.h>

#include <robotstatus.h>
#include <math_tool.h>

#include <drive_gimble.h>
#include <drive_chassis.h>




#define CHASSIS_W   19.8785   //cm
#define CHASSIS_L   17.5375   //cm

#define YAW_INIT_ERROR 0.5f
#define PIT_INIT_ERROR 0.2f



infantry_control_t g_infc;


object_t g_speed_target;
object_t g_angle_target;

static object_t s_angle_contorl_out;
static object_t s_speed_contorl_out;


static void PID_Reset(void)
{
	
	PID_SetParam(&g_infc.pid[YAW_ENCONDE_ANGLE],  1.3,0,0,0,0,0);
	PID_SetParam(&g_infc.pid[YAW_ENCONDE_SPEED],  200,4,0,100,22222,0);
	PID_SetParam(&g_infc.pid[PIT_ENCODNE_ANGLE],  300,1400,6,6,6666,18);
	

	PID_SetParam(&g_infc.pid[PIT_GYRO_ANGLE],  2,0,0,0,0,0);
	PID_SetParam(&g_infc.pid[PIT_GYRO_SPEED],  110,4.5,0,100,5000,0);
	
	PID_SetParam(&g_infc.pid[YAW_GYRO_ANGLE],  1.4,0,0,0,0,0);
	PID_SetParam(&g_infc.pid[YAW_GYRO_SPEED],  399,18,0,100,20000,0);
	
	
    PID_SetParam(&g_infc.pid[SHOOT_SPEED],3.43,38.15,0.02,5000,6666,14.2);
    PID_SetParam(&g_infc.pid[SHOOT_ANGLE],100,0,0,0,0,0);
	
	
	PID_SetParam(&g_infc.pid[LF_SPEED],   3.26, 26.64,0.06,1500,5000,61.3);
	PID_SetParam(&g_infc.pid[LA_SPEED],   3.26, 26.64,0.06,1500,5000,61.3);
	PID_SetParam(&g_infc.pid[RF_SPEED],   3.26, 26.64,0.06,1500,5000,61.3);
	PID_SetParam(&g_infc.pid[RA_SPEED],   3.26, 26.64,0.06,1500,5000,61.3);
	
	
	PID_SetParam(&g_infc.pid[CH_ROTATE_SPEED],0.09,9.15,0,200,120,0);
	PID_SetParam(&g_infc.pid[CH_ROTATE_ANGLE],2.0,0,0,0,0,0);//3.2
	
}

void Infan_Control_Init(void)
{
	PID_Reset();
	
}

void GIMBLE_ENCONDE_Control(object_t target)
{
	static u8 error_temp=0;
	//函数运算间隔计算
    static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	g_infc.angle_outer_target.yaw       = target.yaw;
	g_infc.angle_outer_error.yaw  	    = g_infc.angle_outer_target.yaw-g_data_6623.angle[YAW];
    g_infc.angle_outer_error.yaw  	    = ApplyDeadbandfloat(g_infc.angle_outer_error.yaw,0.5f);

	g_infc.angle_outer_target.pitch     = target.pitch;
	g_infc.angle_outer_error.pitch      = g_infc.angle_outer_target.pitch - g_data_6623.angle[PITCH];
	g_infc.angle_outer_error.pitch  	= ApplyDeadbandfloat(g_infc.angle_outer_error.pitch,0.001f);
	
	
	
	if(robot_status.gimbal_mode==GIMBLE_INIT&&robot_status.mpu6500_status==MPU6500_NO)
	{
		if((abs(g_infc.angle_outer_error.yaw)<YAW_INIT_ERROR)&&(abs(g_infc.angle_outer_error.pitch)<PIT_INIT_ERROR))
		{
			error_temp++;
			if(error_temp>160)
			{
				robot_status.mpu6500_status=MPU6500_INIT;   //6500初始化
				GPIO_ResetBits(GPIOG, GPIO_Pin_1);
				error_temp=0;
			}
		}
	}
	
	s_angle_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_ENCONDE_ANGLE],g_infc.angle_outer_error.yaw,deltaT);
    s_angle_contorl_out.yaw         	= Constrainfloat(s_angle_contorl_out.yaw,-66,66);
	
	s_angle_contorl_out.pitch         	= PID_GetPID(&g_infc.pid[PIT_ENCODNE_ANGLE],g_infc.angle_outer_error.pitch,deltaT);
	s_angle_contorl_out.pitch         	= Constrainfloat(s_angle_contorl_out.pitch,-6666,6666);
	
	
	g_infc.speed_inner_error.yaw  	    =  s_angle_contorl_out.yaw -g_data_6623.speed[YAW];

	//死区控制
    g_infc.speed_inner_error.yaw  	    = ApplyDeadbandfloat( g_infc.speed_inner_error.yaw,0.5);
	//PID算法，计算出速度环的控制量
    s_speed_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_ENCONDE_SPEED],g_infc.speed_inner_error.yaw,1);
	//输出限幅
    s_speed_contorl_out.yaw         	= Constrainfloat(s_speed_contorl_out.yaw,-22222,22222);
	
    Cmd_6623_ESC(s_speed_contorl_out.yaw,-s_angle_contorl_out.pitch);
	
	//Cmd_6623_ESC(0,0);
}

void GIMBLE_GYRO_Control(object_t target)
{
	
	//函数运算间隔计算
    static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	g_infc.angle_outer_target.yaw       = target.yaw;
	g_infc.angle_outer_target.pitch     = target.pitch;
	
	//g_infc.angle_outer_error.yaw  	    = g_infc.angle_outer_target.yaw   - +.yaw;
	
	g_infc.angle_outer_error.yaw  	    = g_infc.angle_outer_target.yaw   - g_imu_data.absolute_yaw;
	
	g_infc.angle_outer_error.pitch      = g_infc.angle_outer_target.pitch - g_data_6623.angle[PITCH];
	
    g_infc.angle_outer_error.yaw  	    = ApplyDeadbandfloat(g_infc.angle_outer_error.yaw,0.1f);
	g_infc.angle_outer_error.pitch  	= ApplyDeadbandfloat(g_infc.angle_outer_error.pitch,0);
	
	
	s_angle_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_GYRO_ANGLE],g_infc.angle_outer_error.yaw,deltaT);
	s_angle_contorl_out.pitch         	= PID_GetPID(&g_infc.pid[PIT_GYRO_ANGLE],g_infc.angle_outer_error.pitch,deltaT);
	

	
    s_angle_contorl_out.yaw         	= Constrainfloat(s_angle_contorl_out.yaw,-50,50);
	s_angle_contorl_out.pitch         	= Constrainfloat(s_angle_contorl_out.pitch,-50,50);
	
	
	
	g_infc.speed_inner_error.yaw  	    = s_angle_contorl_out.yaw    - g_imu_data.yaw_speed;
	g_infc.speed_inner_error.pitch  	= s_angle_contorl_out.pitch  - g_imu_data.pit_speed;
	
	//死区控制
    g_infc.speed_inner_error.yaw  	    = ApplyDeadbandfloat( g_infc.speed_inner_error.yaw,0);
	g_infc.speed_inner_error.pitch  	= ApplyDeadbandfloat( g_infc.speed_inner_error.pitch,0);
	
	//PID算法，计算出速度环的控制量
    s_speed_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_GYRO_SPEED],g_infc.speed_inner_error.yaw,1);
	s_speed_contorl_out.pitch         	= PID_GetPID(&g_infc.pid[PIT_GYRO_SPEED],g_infc.speed_inner_error.pitch,1);
	//输出限幅
    s_speed_contorl_out.yaw         	= Constrainfloat(s_speed_contorl_out.yaw,-22222,22222);
	s_speed_contorl_out.pitch         	= Constrainfloat(s_speed_contorl_out.pitch,-6666,6666);
	
    Cmd_6623_ESC(s_speed_contorl_out.yaw,-s_speed_contorl_out.pitch);
	//Cmd_6623_ESC(0,0);
	
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
	g_infc.speed_inner_error.lf     = ApplyDeadbandfloat( g_infc.speed_inner_error.lf,5);
	g_infc.speed_inner_error.la     = ApplyDeadbandfloat( g_infc.speed_inner_error.la,5);
	g_infc.speed_inner_error.rf     = ApplyDeadbandfloat( g_infc.speed_inner_error.rf,5);
	g_infc.speed_inner_error.ra     = ApplyDeadbandfloat( g_infc.speed_inner_error.ra,5);
	
	//PID算法，计算出速度环的控制量
	s_speed_contorl_out.lf          = PID_GetPID(&g_infc.pid[LF_SPEED],g_infc.speed_inner_error.lf,deltaT);
	s_speed_contorl_out.la          = PID_GetPID(&g_infc.pid[LA_SPEED],g_infc.speed_inner_error.la,deltaT);
	s_speed_contorl_out.rf          = PID_GetPID(&g_infc.pid[RF_SPEED],g_infc.speed_inner_error.rf,deltaT);
	s_speed_contorl_out.ra          = PID_GetPID(&g_infc.pid[RA_SPEED],g_infc.speed_inner_error.ra,deltaT);
	
	//输出限幅
	s_speed_contorl_out.lf          = Constrainfloat(s_speed_contorl_out.lf,-5000,5000);
	s_speed_contorl_out.la          = Constrainfloat(s_speed_contorl_out.la,-5000,5000);
	s_speed_contorl_out.rf          = Constrainfloat(s_speed_contorl_out.rf,-5000,5000);
	s_speed_contorl_out.ra          = Constrainfloat(s_speed_contorl_out.ra,-5000,5000);
	
	//can发送控制量
	Cmd_3510_ESC(s_speed_contorl_out.lf,s_speed_contorl_out.la,s_speed_contorl_out.rf,s_speed_contorl_out.ra);
	//Cmd_3510_ESC(0,0,0,0);
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
    g_infc.speed_inner_error.shoot  	= ApplyDeadbandfloat( g_infc.speed_inner_error.shoot,10);
	
	//PID算法，计算出速度环的控制量
    s_speed_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
	
	//输出限幅
    s_speed_contorl_out.shoot         	= Constrainfloat(s_speed_contorl_out.shoot,-6666,6666);
	
	//can发送控制量
	Cmd_2006_ESC(s_speed_contorl_out.shoot);
	//Cmd_2006_ESC(0);
}

void Angle_2006_Control(object_t target)
{
//	//函数运算间隔计算
//    static uint64_t previousT;
//    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
//    previousT = Get_SysTimeUs();
	//角度环目标赋值
	g_infc.angle_outer_target.shoot		= target.shoot;
	
	//计算角度控制误差
	g_infc.angle_outer_error.shoot  	= g_infc.angle_outer_target.shoot-g_data_2006.angle;
	
	//死区控制
    g_infc.angle_outer_error.shoot  	= ApplyDeadbandfloat(g_infc.angle_outer_error.shoot,0.1);
	
	//PID算法，计算出角度环的控制量
    s_angle_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,1);
	
	//输出限幅
    s_angle_contorl_out.shoot         	= Constrainfloat(s_angle_contorl_out.shoot,-6666,6666);
	
	//将角度外环控制量作为速度内环的控制目标
	Speed_2006_Control(s_angle_contorl_out);
}
/*
*x W
*|<-
*| <-
*|__________y
*/
void Speed_Chassis_Control(float vx,float vy,float wz)    //底盘速度控制
{
	g_speed_target.rf=-(vx+vy+wz*(CHASSIS_L+CHASSIS_W));
	g_speed_target.lf=  vx-vy-wz*(CHASSIS_L+CHASSIS_W);
	g_speed_target.la=  vx+vy-wz*(CHASSIS_L+CHASSIS_W);
	g_speed_target.ra=-(vx-vy+wz*(CHASSIS_L+CHASSIS_W));
	Speed_3510_Control(g_speed_target);
}

float Angle_Rotate_Control(object_t target)         //
{
    static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	//角度环目标赋值
	g_infc.angle_outer_target.ch_rotate		= target.ch_rotate;
	
	//计算角度控制误差
	g_infc.angle_outer_error.ch_rotate  	= g_infc.angle_outer_target.ch_rotate-g_data_6623.angle[YAW];
	
	//死区控制
    g_infc.angle_outer_error.ch_rotate  	= ApplyDeadbandfloat(g_infc.angle_outer_error.ch_rotate,1.0f);
	
	//PID算法，计算出角度环的控制量
    s_angle_contorl_out.ch_rotate         	= PID_GetPID(&g_infc.pid[CH_ROTATE_ANGLE],g_infc.angle_outer_error.ch_rotate,deltaT);
	
	//输出限幅
    s_angle_contorl_out.ch_rotate         	= Constrainfloat(s_angle_contorl_out.ch_rotate,-110,110);
	
	
	return s_angle_contorl_out.ch_rotate;
}

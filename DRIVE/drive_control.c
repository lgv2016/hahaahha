#include <drive_control.h>
#include <drive_delay.h> 
#include <drive_imu.h>

#include <motor_chassis.h>
#include <motor_cradle_head.h>

#include <robotstatus.h>
#include <math_tool.h>

#include <drive_gimble.h>
#include <drive_chassis.h>

#include <drive_judge.h>



#define CHASSIS_W   19.8785   //cm
#define CHASSIS_L   17.5375   //cm

#define YAW_INIT_ERROR 0.5f
#define PIT_INIT_ERROR 0.2f




#ifdef  RED1
#define CHASSIS_OUPUT 4800
#endif

#ifdef  RED2
#define CHASSIS_OUPUT 6000
#endif


infantry_control_t g_infc;


object_t g_speed_target;
object_t g_angle_target;

static object_t s_angle_contorl_out;
static object_t s_speed_contorl_out;

//object_t s_angle_contorl_out;
//object_t s_speed_contorl_out;


static void PID_Reset(void)
{
	
	PID_SetParam(&g_infc.pid[YAW_ENCONDE_ANGLE],  1.3,0,0,0,0,0);
	PID_SetParam(&g_infc.pid[YAW_ENCONDE_SPEED],  200,4,0,100,22222,0);
	PID_SetParam(&g_infc.pid[PIT_ENCODNE_ANGLE],  300,1400,6,6,6666,18);
	

	PID_SetParam(&g_infc.pid[PIT_GYRO_ANGLE],  2,0,0,0,0,0);
	PID_SetParam(&g_infc.pid[PIT_GYRO_SPEED],  110,4.5,0,100,5000,0);
	
	PID_SetParam(&g_infc.pid[YAW_GYRO_ANGLE],  1.4,0,0,0,0,0);
	PID_SetParam(&g_infc.pid[YAW_GYRO_SPEED],  399,18,0,100,20000,0);
	
	
   // PID_SetParam(&g_infc.pid[SHOOT_SPEED],3.43,38.15,0.02,5000,6666,14.2);
	PID_SetParam(&g_infc.pid[SHOOT_SPEED],5.0f,50.0f,0,9000,9000,0);
    PID_SetParam(&g_infc.pid[SHOOT_ANGLE],100,0,0,0,0,0);
	
	
	PID_SetParam(&g_infc.pid[LF_SPEED],   3.26, 26.64,0.06,8000,CHASSIS_OUPUT,61.3);
	PID_SetParam(&g_infc.pid[LA_SPEED],   3.26, 26.64,0.06,8000,CHASSIS_OUPUT,61.3);
	PID_SetParam(&g_infc.pid[RF_SPEED],   3.26, 26.64,0.06,8000,CHASSIS_OUPUT,61.3);
	PID_SetParam(&g_infc.pid[RA_SPEED],   3.26, 26.64,0.06,8000,CHASSIS_OUPUT,61.3);
	
	
	PID_SetParam(&g_infc.pid[CH_ROTATE_SPEED],0.09,9.15,0,200,120,0);
	PID_SetParam(&g_infc.pid[CH_ROTATE_ANGLE],2.3,0,0,0,0,0);//3.2
	
}


void Infan_Control_Init(void)
{
	PID_Reset();
	
}

void GIMBLE_ENCONDE_Control(object_t target)
{
	static u8 error_temp=0;
	//��������������
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
				robot_status.mpu6500_status=MPU6500_INIT;   //6500��ʼ��
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

	//��������
    g_infc.speed_inner_error.yaw  	    = ApplyDeadbandfloat( g_infc.speed_inner_error.yaw,0.5);
	//PID�㷨��������ٶȻ��Ŀ�����
    s_speed_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_ENCONDE_SPEED],g_infc.speed_inner_error.yaw,1);
	//����޷�
    s_speed_contorl_out.yaw         	= Constrainfloat(s_speed_contorl_out.yaw,-22222,22222);
	
  Cmd_6623_ESC(s_speed_contorl_out.yaw,-s_angle_contorl_out.pitch);
	
	//Cmd_6623_ESC(0,0);
}


void GIMBLE_GYRO_Control(object_t target)
{
	
	//��������������
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
	

	
    s_angle_contorl_out.yaw         	= Constrainfloat(s_angle_contorl_out.yaw,-80,80);
	s_angle_contorl_out.pitch         	= Constrainfloat(s_angle_contorl_out.pitch,-70,70);

//    s_angle_contorl_out.yaw         	= Constrainfloat(s_angle_contorl_out.yaw,-yawmax,yawmax);
//	s_angle_contorl_out.pitch         	= Constrainfloat(s_angle_contorl_out.pitch,-pitmax,pitmax);
	
	
	
	g_infc.speed_inner_error.yaw  	    = s_angle_contorl_out.yaw    - g_imu_data.yaw_speed;
	g_infc.speed_inner_error.pitch  	= s_angle_contorl_out.pitch  - g_imu_data.pit_speed;
	
	//��������
    g_infc.speed_inner_error.yaw  	    = ApplyDeadbandfloat( g_infc.speed_inner_error.yaw,0);
	g_infc.speed_inner_error.pitch  	= ApplyDeadbandfloat( g_infc.speed_inner_error.pitch,0);
	
	//PID�㷨��������ٶȻ��Ŀ�����
    s_speed_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_GYRO_SPEED],g_infc.speed_inner_error.yaw,1);
	s_speed_contorl_out.pitch         	= PID_GetPID(&g_infc.pid[PIT_GYRO_SPEED],g_infc.speed_inner_error.pitch,1);
	//����޷�
    s_speed_contorl_out.yaw         	= Constrainfloat(s_speed_contorl_out.yaw,-22222,22222);
	s_speed_contorl_out.pitch         	= Constrainfloat(s_speed_contorl_out.pitch,-6666,6666);
	
  Cmd_6623_ESC(s_speed_contorl_out.yaw,-s_speed_contorl_out.pitch);
	//Cmd_6623_ESC(0,0);
	
}


void Speed_3510_Control(object_t target)
{
	
	float chassis_out_limit;
	float chassis_out_sum;
	//��������������
   	static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	//�ٶȻ�Ŀ�긳ֵ
	g_infc.speed_inner_target.lf    = target.lf;
	g_infc.speed_inner_target.la    = target.la;
	g_infc.speed_inner_target.rf    = target.rf;
	g_infc.speed_inner_target.ra    = target.ra;
	
	//�����ٶȻ��������
	g_infc.speed_inner_error.lf     = g_infc.speed_inner_target.lf      - g_data_3510.speed[LF];
	g_infc.speed_inner_error.la     = g_infc.speed_inner_target.la      - g_data_3510.speed[LA];
	g_infc.speed_inner_error.rf     = g_infc.speed_inner_target.rf      - g_data_3510.speed[RF];
	g_infc.speed_inner_error.ra     = g_infc.speed_inner_target.ra      - g_data_3510.speed[RA];
	
	//��������
	g_infc.speed_inner_error.lf     = ApplyDeadbandfloat( g_infc.speed_inner_error.lf,2);
	g_infc.speed_inner_error.la     = ApplyDeadbandfloat( g_infc.speed_inner_error.la,2);
	g_infc.speed_inner_error.rf     = ApplyDeadbandfloat( g_infc.speed_inner_error.rf,2);
	g_infc.speed_inner_error.ra     = ApplyDeadbandfloat( g_infc.speed_inner_error.ra,2);
	
	//PID�㷨��������ٶȻ��Ŀ�����
	s_speed_contorl_out.lf          = PID_GetPID(&g_infc.pid[LF_SPEED],g_infc.speed_inner_error.lf,deltaT);
	s_speed_contorl_out.la          = PID_GetPID(&g_infc.pid[LA_SPEED],g_infc.speed_inner_error.la,deltaT);
	s_speed_contorl_out.rf          = PID_GetPID(&g_infc.pid[RF_SPEED],g_infc.speed_inner_error.rf,deltaT);
	s_speed_contorl_out.ra          = PID_GetPID(&g_infc.pid[RA_SPEED],g_infc.speed_inner_error.ra,deltaT);
	
	//����޷�
	s_speed_contorl_out.lf          = Constrainfloat(s_speed_contorl_out.lf,-CHASSIS_OUPUT,CHASSIS_OUPUT);
	s_speed_contorl_out.la          = Constrainfloat(s_speed_contorl_out.la,-CHASSIS_OUPUT,CHASSIS_OUPUT);
	s_speed_contorl_out.rf          = Constrainfloat(s_speed_contorl_out.rf,-CHASSIS_OUPUT,CHASSIS_OUPUT);
	s_speed_contorl_out.ra          = Constrainfloat(s_speed_contorl_out.ra,-CHASSIS_OUPUT,CHASSIS_OUPUT);
	
	
	if(judge_data.power_buffer<=20.0f)
	{
		chassis_out_sum=abs(s_speed_contorl_out.lf)+abs(s_speed_contorl_out.la)+abs(s_speed_contorl_out.rf)+abs(s_speed_contorl_out.ra);
		chassis_out_limit=judge_data.power_buffer*judge_data.power_buffer*6.0f;
		
		s_speed_contorl_out.lf=chassis_out_limit*s_speed_contorl_out.lf/chassis_out_sum;
		s_speed_contorl_out.la=chassis_out_limit*s_speed_contorl_out.la/chassis_out_sum;
		s_speed_contorl_out.rf=chassis_out_limit*s_speed_contorl_out.rf/chassis_out_sum;
		s_speed_contorl_out.ra=chassis_out_limit*s_speed_contorl_out.ra/chassis_out_sum;
	}
	
	//can���Ϳ�����
	Cmd_3510_ESC(s_speed_contorl_out.lf,s_speed_contorl_out.la,s_speed_contorl_out.rf,s_speed_contorl_out.ra);
	//Cmd_3510_ESC(0,0,0,0);
}


void Speed_2006_Control(object_t target)
{
	//��������������
	static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	//�ٶȻ�Ŀ�긳ֵ
    g_infc.speed_inner_target.shoot		= target.shoot;
	
	//�����ٶȻ��������
    g_infc.speed_inner_error.shoot  	= g_infc.speed_inner_target.shoot - g_data_2006.speed;
	
	//��������
    g_infc.speed_inner_error.shoot  	= ApplyDeadbandfloat( g_infc.speed_inner_error.shoot,0);
	
	//PID�㷨��������ٶȻ��Ŀ�����
    s_speed_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
	
	//����޷�
    s_speed_contorl_out.shoot         	= Constrainfloat(s_speed_contorl_out.shoot,-9000,9000);
	
	//can���Ϳ�����
	Cmd_2006_ESC(s_speed_contorl_out.shoot);
	//Cmd_2006_ESC(0);
}

void Angle_2006_Control(object_t target)
{
//	//��������������
//    static uint64_t previousT;
//    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
//    previousT = Get_SysTimeUs();
	//�ǶȻ�Ŀ�긳ֵ
	g_infc.angle_outer_target.shoot		= target.shoot;
	
	//����Ƕȿ������
	g_infc.angle_outer_error.shoot  	= g_infc.angle_outer_target.shoot-g_data_2006.angle;
	
	//��������
    g_infc.angle_outer_error.shoot  	= ApplyDeadbandfloat(g_infc.angle_outer_error.shoot,0.1);
	
	//PID�㷨��������ǶȻ��Ŀ�����
    s_angle_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,1);
	
	//����޷�
    s_angle_contorl_out.shoot         	= Constrainfloat(s_angle_contorl_out.shoot,-6666,6666);
	
	//���Ƕ��⻷��������Ϊ�ٶ��ڻ��Ŀ���Ŀ��
	Speed_2006_Control(s_angle_contorl_out);
}
/*
*x W
*|<-
*| <-
*|__________y
*/
void Speed_Chassis_Control(float vx,float vy,float wz)    //�����ٶȿ���
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
	//�ǶȻ�Ŀ�긳ֵ
	g_infc.angle_outer_target.ch_rotate		= target.ch_rotate;
	
	//����Ƕȿ������
	g_infc.angle_outer_error.ch_rotate  	= g_infc.angle_outer_target.ch_rotate-g_data_6623.angle[YAW];
	
	//��������
    g_infc.angle_outer_error.ch_rotate  	= ApplyDeadbandfloat(g_infc.angle_outer_error.ch_rotate,1.0f);
	
	//PID�㷨��������ǶȻ��Ŀ�����
    s_angle_contorl_out.ch_rotate         	= PID_GetPID(&g_infc.pid[CH_ROTATE_ANGLE],g_infc.angle_outer_error.ch_rotate,deltaT);
	
	//����޷�
    s_angle_contorl_out.ch_rotate         	= Constrainfloat(s_angle_contorl_out.ch_rotate,-120,120);
	
	
	return s_angle_contorl_out.ch_rotate;
}



void Chassis_Power_Control(object_t target)
{
	//��������������
	static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	//�ٶȻ�Ŀ�긳ֵ
    g_infc.speed_inner_target.ch_power		 = judge_data.max_power;
	
	//�����ٶȻ��������
    g_infc.speed_inner_error.ch_power     	 = g_infc.speed_inner_target.ch_power - judge_data.power;
	
	//��������
    g_infc.speed_inner_error.ch_power     	= ApplyDeadbandfloat( g_infc.speed_inner_error.ch_power,1.0f);
	
	//PID�㷨��������ٶȻ��Ŀ�����
    s_speed_contorl_out.ch_power         	= PID_GetPID(&g_infc.pid[CH_POWER],g_infc.speed_inner_error.ch_power,deltaT);
	
	//����޷�
    s_speed_contorl_out.ch_power         	= Constrainfloat(s_speed_contorl_out.ch_power,-9000,9000);
	
}


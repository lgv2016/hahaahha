#include <drive_control.h>
#include <drive_delay.h> 
#include <drive_imu.h>

#include <motor_chassis.h>
#include <motor_cradle_head.h>

#include <robotstatus.h>
#include <math_tool.h>


#define CHASSIS_W   19.8785   //cm
#define CHASSIS_L   17.5375   //cm

infantry_control_t g_infc;

object_t g_speed_target;
object_t g_angle_target;

static object_t s_angle_contorl_out;
static object_t s_speed_contorl_out;


void INC_SetParam(INC_t* inc, float input, float max, float min, float out,float period)
{
	inc->input=input;
	inc->max=max;
	inc->min=min;
	inc->out=out;
	inc->period=period;
}

void INC_fun(INC_t* inc)
{
	inc->out+=inc->input*inc->period;
	if(inc->out>inc->max)
	{
		inc->out=inc->max;
	}
	else if(inc->out<inc->min)
	{
		inc->out=inc->min;
	}
}


static void INC_Reset(void)
{
	INC_SetParam(&g_infc.inc[CH_ROTATE_SPEED],80,100,-100,0,0.001);
}

static void PID_Reset(void)
{
	
    PID_SetParam(&g_infc.pid[SHOOT_SPEED],3.43,38.15,0.02,1000,5000,14.2);
    PID_SetParam(&g_infc.pid[SHOOT_ANGLE],100,0,0,0,0,0);
	
	PID_SetParam(&g_infc.pid[PITCH_ANGLE],120.5,865 ,0.03,8,3000,18.73);

	//PID_SetParam(&g_infc.pid[YAW_SPEED],40,800,0,48,26000,0);
	PID_SetParam(&g_infc.pid[YAW_SPEED],40,15,0,300,30000,0);
	PID_SetParam(&g_infc.pid[YAW_ANGLE],0.7,0,0,0,0,0);
	
	PID_SetParam(&g_infc.pid[LF_SPEED],   3.26, 26.64,0.06,1000,5000,61.3);
	PID_SetParam(&g_infc.pid[LA_SPEED],   3.26, 26.64,0.06,1000,5000,61.3);
	PID_SetParam(&g_infc.pid[RF_SPEED],   3.26, 26.64,0.06,1000,5000,61.3);
	PID_SetParam(&g_infc.pid[RA_SPEED],   3.26, 26.64,0.06,1000,5000,61.3);
	
	PID_SetParam(&g_infc.pid[CH_ROTATE_SPEED],0.09,9.15,0,200,120,0);
	PID_SetParam(&g_infc.pid[CH_ROTATE_ANGLE],10,0,0,0,0,0);
	
}
void Infan_Control_Init(void)
{
   // g_angle_target.yaw=300;
	
	//g_speed_target.yaw=30;
	PID_Reset();
	INC_Reset();
	
	
}



void Speed_6623_Control(object_t target)
{
	//��������������
	static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	//YAW ��ֵ
	//PID_SetParam(&g_infc.pid[YAW_SPEED],40,800,0,abs(target.yaw)*0.8,26000,0);
	//�ٶȻ�Ŀ�긳ֵ
    g_infc.speed_inner_target.yaw		= target.yaw;
	
	//�����ٶȻ��������
    g_infc.speed_inner_error.yaw  	    = g_infc.speed_inner_target.yaw - g_data_6623.speed[YAW];
	
	
	//��������
    g_infc.speed_inner_error.yaw  	    = ApplyDeadbandFloat( g_infc.speed_inner_error.yaw,0.5);
	
	
	//PID�㷨��������ٶȻ��Ŀ�����
    s_speed_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_SPEED],g_infc.speed_inner_error.yaw,1);
	
	
	
	//����޷�
    s_speed_contorl_out.yaw         	= ConstrainFloat(s_speed_contorl_out.yaw,-30000,30000);
	
	Cmd_6623_ESC(s_speed_contorl_out.yaw,-s_angle_contorl_out.pitch);
	
	//Cmd_6623_ESC(0,0);
}



void Angle_6623_Control(object_t target)
{
	
	static u8 error_temp=0;
	//��������������
    static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	
	//�ǶȻ�Ŀ���޷���ֵ
	
	g_infc.angle_outer_target.yaw   = ConstrainFloat(target.yaw,  YAW_INIT_ANGLE - 100.0f,YAW_INIT_ANGLE + 100.0f);
	g_infc.angle_outer_target.pitch = ConstrainFloat(target.pitch,PIT_INIT_ANGLE - 20.0f, PIT_INIT_ANGLE + 20.0f);	 
	

	
	//����Ƕȿ������
	g_infc.angle_outer_error.yaw  	= g_infc.angle_outer_target.yaw-g_data_6623.angle[YAW];
	g_infc.angle_outer_error.pitch  = g_infc.angle_outer_target.pitch - g_data_6623.angle[PITCH];
	
	//��������
    g_infc.angle_outer_error.yaw  	= ApplyDeadbandFloat(g_infc.angle_outer_error.yaw,0.4f);
	g_infc.angle_outer_error.pitch  	= ApplyDeadbandFloat(g_infc.angle_outer_error.pitch,0.4f);
	
	if(robot_status.gimbal_status==NO_INIT)
	{
		if((abs(g_infc.angle_outer_error.yaw)<1.0f)&&(abs(g_infc.angle_outer_error.pitch)<0.8f))
		{
			error_temp++;
			if(error_temp>=80)
			{
				robot_status.gimbal_status=INIT_FINISH;
				Cmd_GIMBAL_ESC(1);
				error_temp=0;
			}
		}
		
	}
	//PID�㷨��������ǶȻ��Ŀ�����
    s_angle_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_ANGLE],g_infc.angle_outer_error.yaw,deltaT);
	s_angle_contorl_out.pitch         	= PID_GetPID(&g_infc.pid[PITCH_ANGLE],g_infc.angle_outer_error.pitch,deltaT);
	
	//����޷�
    s_angle_contorl_out.yaw         	= ConstrainFloat(s_angle_contorl_out.yaw,-110,110);
	s_angle_contorl_out.pitch         	= ConstrainFloat(s_angle_contorl_out.pitch,-3000,3000);
	
	//���Ƕ��⻷��������Ϊ�ٶ��ڻ��Ŀ���Ŀ��
	Speed_6623_Control(s_angle_contorl_out);
}

void Speed_3510_Control(object_t target)
{
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
	g_infc.speed_inner_error.lf     = ApplyDeadbandFloat( g_infc.speed_inner_error.lf,5);
	g_infc.speed_inner_error.la     = ApplyDeadbandFloat( g_infc.speed_inner_error.la,5);
	g_infc.speed_inner_error.rf     = ApplyDeadbandFloat( g_infc.speed_inner_error.rf,5);
	g_infc.speed_inner_error.ra     = ApplyDeadbandFloat( g_infc.speed_inner_error.ra,5);
	
	//PID�㷨��������ٶȻ��Ŀ�����
	s_speed_contorl_out.lf          = PID_GetPID(&g_infc.pid[LF_SPEED],g_infc.speed_inner_error.lf,deltaT);
	s_speed_contorl_out.la          = PID_GetPID(&g_infc.pid[LA_SPEED],g_infc.speed_inner_error.la,deltaT);
	s_speed_contorl_out.rf          = PID_GetPID(&g_infc.pid[RF_SPEED],g_infc.speed_inner_error.rf,deltaT);
	s_speed_contorl_out.ra          = PID_GetPID(&g_infc.pid[RA_SPEED],g_infc.speed_inner_error.ra,deltaT);
	
	//����޷�
	s_speed_contorl_out.lf          = ConstrainFloat(s_speed_contorl_out.lf,-6666,6666);
	s_speed_contorl_out.la          = ConstrainFloat(s_speed_contorl_out.la,-6666,6666);
	s_speed_contorl_out.rf          = ConstrainFloat(s_speed_contorl_out.rf,-6666,6666);
	s_speed_contorl_out.ra          = ConstrainFloat(s_speed_contorl_out.ra,-6666,6666);
	
	//can���Ϳ�����
	Cmd_3510_ESC(s_speed_contorl_out.lf,s_speed_contorl_out.la,s_speed_contorl_out.rf,s_speed_contorl_out.ra);
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
    g_infc.speed_inner_error.shoot  	= ApplyDeadbandFloat( g_infc.speed_inner_error.shoot,10);
	
	//PID�㷨��������ٶȻ��Ŀ�����
    s_speed_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
	
	//����޷�
    s_speed_contorl_out.shoot         	= ConstrainFloat(s_speed_contorl_out.shoot,-6666,6666);
	
	//can���Ϳ�����
	Cmd_2006_ESC(s_speed_contorl_out.shoot);
	//Cmd_2006_ESC(0);
}

void Angle_2006_Control(object_t target)
{
	//��������������
    static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	//�ǶȻ�Ŀ�긳ֵ
	g_infc.angle_outer_target.shoot		= target.shoot;
	
	//����Ƕȿ������
	g_infc.angle_outer_error.shoot  	= g_infc.angle_outer_target.shoot-g_data_2006.angle;
	
	//��������
    g_infc.angle_outer_error.shoot  	= ApplyDeadbandFloat(g_infc.angle_outer_error.shoot,0.1);
	
	//PID�㷨��������ǶȻ��Ŀ�����
    s_angle_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,deltaT);
	
	//����޷�
    s_angle_contorl_out.shoot         	= ConstrainFloat(s_angle_contorl_out.shoot,-6666,6666);
	
	//���Ƕ��⻷��������Ϊ�ٶ��ڻ��Ŀ���Ŀ��
	Speed_2006_Control(s_angle_contorl_out);
}
/*
*Y W
*|<-
*| <-
*|__________x
*/
void Speed_Chassis_Control(float vx,float vy,float wz)    //�����ٶȿ���
{
	g_speed_target.rf=-(vy-vx+wz*(CHASSIS_L+CHASSIS_W));
	g_speed_target.lf=vy+vx-wz*(CHASSIS_L+CHASSIS_W);
	g_speed_target.la=vy-vx-wz*(CHASSIS_L+CHASSIS_W);
	g_speed_target.ra=-(vy+vx+wz*(CHASSIS_L+CHASSIS_W));
	Speed_3510_Control(g_speed_target);
}

void Speed_Rotate_Control(object_t target)       //����speed
{
	static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	
	//�ٶȻ�Ŀ�긳ֵ
	g_infc.speed_inner_target.ch_rotate=target.ch_rotate;
	
	//�����ٶȻ��������
    g_infc.speed_inner_error.ch_rotate  	= g_infc.speed_inner_target.ch_rotate-g_imu_data.yaw_speed;
	
	//��������
    g_infc.speed_inner_error.ch_rotate  	= ApplyDeadbandFloat( g_infc.speed_inner_error.ch_rotate,2.2);
	
	//PID�㷨��������ٶȻ��Ŀ�����
    s_speed_contorl_out.ch_rotate         	= PID_GetPID(&g_infc.pid[CH_ROTATE_SPEED],g_infc.speed_inner_error.ch_rotate,deltaT);
	
	//����޷�
    s_speed_contorl_out.ch_rotate         	= ConstrainFloat(s_speed_contorl_out.ch_rotate,-178,178);
	
	Speed_Chassis_Control(0,0,s_speed_contorl_out.ch_rotate);
}

void Angle_Rotate_Control(object_t target)         //
{
    static uint64_t previousT;
    float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
    previousT = Get_SysTimeUs();
	//�ǶȻ�Ŀ�긳ֵ
	g_infc.angle_outer_target.ch_rotate		= target.ch_rotate;
	
	//����Ƕȿ������
	g_infc.angle_outer_error.ch_rotate  	= g_infc.angle_outer_target.ch_rotate-g_imu_data.yaw;
	
	//��������
    g_infc.angle_outer_error.ch_rotate  	= ApplyDeadbandFloat(g_infc.angle_outer_error.ch_rotate,1);
	
	//PID�㷨��������ǶȻ��Ŀ�����
    s_angle_contorl_out.ch_rotate         	= PID_GetPID(&g_infc.pid[CH_ROTATE_ANGLE],g_infc.angle_outer_error.ch_rotate,deltaT);
	
	//����޷�
    s_angle_contorl_out.ch_rotate         	= ConstrainFloat(s_angle_contorl_out.ch_rotate,-200,200);
	
	//���Ƕ��⻷��������Ϊ�ٶ��ڻ��Ŀ���Ŀ��
	Speed_Rotate_Control(s_angle_contorl_out);
}

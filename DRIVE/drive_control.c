#include <drive_control.h>
#include <motor_chassis.h>
#include <motor_cradle_head.h>
#include <drive_delay.h>
 
 #include <robotstatus.h>
 


infantry_control_t g_infc;

object_t g_speed_target;
object_t g_angle_target;

static object_t s_angle_contorl_out;
static object_t s_speed_contorl_out;




static void PID_Reset(void)
{
    PID_SetParam(&g_infc.pid[SHOOT_SPEED],5.0, 0.1 ,  0,  5000,   0);
    PID_SetParam(&g_infc.pid[SHOOT_ANGLE],180, 0   ,  0,  5000,   0);
	
    PID_SetParam(&g_infc.pid[PITCH_ANGLE],46, 253 ,  0,  3000,   0);
	PID_SetParam(&g_infc.pid[YAW_ANGLE],35.7, 64.89 ,  0,  3000,   0);
	
	PID_SetParam(&g_infc.pid[LF_SPEED],  1.8, 5.6,  0,  5000,   0);
	PID_SetParam(&g_infc.pid[LA_SPEED],  1.8, 5.6,  0,  5000,   0);
	PID_SetParam(&g_infc.pid[RF_SPEED],  1.8, 5.6,  0,  5000,   0);
	PID_SetParam(&g_infc.pid[RA_SPEED],  1.8, 5.6,  0,  5000,   0);
}
void Infan_Control_Init(void)
{
	
	g_angle_target.pitch=263;
	g_angle_target.yaw=88;
	
	g_speed_target.shoot=3500;
	g_speed_target.lf=4000;
	g_speed_target.la=4000;
	g_speed_target.rf=4000;
	g_speed_target.ra=4000;
	

	PID_Reset();
}
void Angle_6623_Control(object_t target)
{
	static u8 error_temp=0;
	if(robot_status.imu_data==DATA_FALSE&&robot_status.imu_status==CORRECT)
	{
		//��������������
		static uint64_t previousT;
		float deltaT = (Get_SysTimeUs() - previousT) * 1e-6;
		previousT = Get_SysTimeUs();
		
		g_infc.angle_outer_target.pitch		= target.pitch;
		g_infc.angle_outer_target.yaw		= target.yaw;
		
		//����Ƕȿ������
		g_infc.angle_outer_error.pitch  	= g_infc.angle_outer_target.pitch - g_data_6623.angle[PITCH];
		g_infc.angle_outer_error.yaw  	    = g_infc.angle_outer_target.yaw   - g_data_6623.angle[YAW];
		
		//��������
		g_infc.angle_outer_error.pitch  	= ApplyDeadbandFloat(g_infc.angle_outer_error.pitch,0.05);
		g_infc.angle_outer_error.yaw  	    = ApplyDeadbandFloat(g_infc.angle_outer_error.yaw,  0.05);
		
		if(abs(g_infc.angle_outer_error.pitch)<0.5f)//&&abs((g_infc.angle_outer_error.yaw)<0.5f//))
		{
			error_temp++;
			if(error_temp>=100)
			{
				//robot_status.imu_status=CORRECT_START;
				error_temp=0;
			}
			
		}
		//PID�㷨��������ǶȻ��Ŀ�����
		s_angle_contorl_out.pitch         	= PID_GetPID(&g_infc.pid[PITCH_ANGLE],g_infc.angle_outer_error.pitch,deltaT);
		s_angle_contorl_out.yaw         	= PID_GetPID(&g_infc.pid[YAW_ANGLE],  g_infc.angle_outer_error.yaw,  deltaT);
		
		//����޷�
		s_angle_contorl_out.pitch         	= ConstrainFloat(s_angle_contorl_out.pitch,-3000,3000);
		s_angle_contorl_out.yaw         	= ConstrainFloat(s_angle_contorl_out.yaw,  -3000,3000);
		
		Cmd_6623_ESC(0,-s_angle_contorl_out.pitch);
    }
//	if(robot_status.imu_data==DATA_TRUE)
//	   Cmd_6623_ESC(0,0);
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
	s_speed_contorl_out.lf          = ConstrainFloat(s_speed_contorl_out.lf,-8000,8000);
	s_speed_contorl_out.la          = ConstrainFloat(s_speed_contorl_out.la,-8000,8000);
	s_speed_contorl_out.rf          = ConstrainFloat(s_speed_contorl_out.rf,-8000,8000);
	s_speed_contorl_out.ra          = ConstrainFloat(s_speed_contorl_out.ra,-8000,8000);
	
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
    s_speed_contorl_out.shoot         	= ConstrainFloat(s_speed_contorl_out.shoot,-10000,10000);
	
	//can���Ϳ�����
	Cmd_2006_ESC(s_speed_contorl_out.shoot);
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
    g_infc.angle_outer_error.shoot  	= ApplyDeadbandFloat(g_infc.angle_outer_error.shoot,0.05);
	
	//PID�㷨��������ǶȻ��Ŀ�����
    s_angle_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,deltaT);
	
	//����޷�
    s_angle_contorl_out.shoot         	= ConstrainFloat(s_angle_contorl_out.shoot,-10000,10000);
	
	//���Ƕ��⻷��������Ϊ�ٶ��ڻ��Ŀ���Ŀ��
	g_infc.speed_inner_target.shoot		= s_angle_contorl_out.shoot;
	
	//�����ٶȻ��������
    g_infc.speed_inner_error.shoot  	= g_infc.speed_inner_target.shoot - g_data_2006.speed;
	
	//��������
    g_infc.speed_inner_error.shoot  	= ApplyDeadbandFloat( g_infc.speed_inner_error.shoot,10);
	
	 //PID�㷨��������ٶȻ��Ŀ�����
    s_speed_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
	
	 //����޷�
    s_speed_contorl_out.shoot         	= ConstrainFloat(s_speed_contorl_out.shoot,-10000,10000);
	
	//can���Ϳ�����
    Cmd_2006_ESC(s_speed_contorl_out.shoot);
}


/******************************************************************
*�� �� ��: Speed_In_Control
*����˵��: �ٶ��ڻ�����
*��    ��: �ٶȲ���ֵ ����ʱ����
*�� �� ֵ: �ٶ��ڻ�������
***********************************************************************/

void Speed_In_Control(object_t measure,float deltaT)
{
    //object_t s_speed_contorl_out;
    
    //�����ٶȻ��������
    g_infc.speed_inner_error.shoot  = g_infc.speed_inner_target.shoot   - measure.shoot;
    g_infc.speed_inner_error.yaw    = g_infc.speed_inner_target.yaw     - measure.yaw;
    g_infc.speed_inner_error.pitch  = g_infc.speed_inner_target.pitch   - measure.pitch;
    g_infc.speed_inner_error.lf     = g_infc.speed_inner_target.lf      - measure.lf;
    
    

    //��������
    g_infc.speed_inner_error.shoot  = ApplyDeadbandFloat( g_infc.speed_inner_error.shoot,10);
    g_infc.speed_inner_error.yaw    = ApplyDeadbandFloat( g_infc.speed_inner_error.yaw,0.1);
    g_infc.speed_inner_error.pitch  = ApplyDeadbandFloat( g_infc.speed_inner_error.pitch,0.1);
    g_infc.speed_inner_error.lf     = ApplyDeadbandFloat( g_infc.speed_inner_error.lf,10);

    
    //PID�㷨��������ٶȻ��Ŀ�����
    s_speed_contorl_out.shoot         = PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
    s_speed_contorl_out.yaw           = PID_GetPID(&g_infc.pid[YAW_SPEED],g_infc.speed_inner_error.yaw,deltaT);
    s_speed_contorl_out.pitch         = PID_GetPID(&g_infc.pid[PITCH_SPEED],g_infc.speed_inner_error.pitch,deltaT);
    s_speed_contorl_out.lf            = PID_GetPID(&g_infc.pid[LF_SPEED],g_infc.speed_inner_error.lf,deltaT);
    
    //����޷�
    s_speed_contorl_out.shoot         = ConstrainFloat(s_speed_contorl_out.shoot,-10000,10000);
    s_speed_contorl_out.yaw           = ConstrainFloat(s_speed_contorl_out.yaw,-5000,5000);
    s_speed_contorl_out.pitch         = ConstrainFloat(s_speed_contorl_out.pitch,-5000,5000);
    s_speed_contorl_out.lf            = ConstrainFloat(s_speed_contorl_out.lf,-8000,8000);
    
  
    
    Cmd_2006_ESC(s_speed_contorl_out.shoot);
    Cmd_6623_ESC(s_speed_contorl_out.yaw,s_speed_contorl_out.pitch);
    Cmd_3510_ESC(s_speed_contorl_out.lf,s_speed_contorl_out.la,s_speed_contorl_out.rf,s_speed_contorl_out.ra);
}
/**********************************************************************************************************
*�� �� ��: SET_Speed_Target
*����˵��: �����ٶ��ڻ�����Ŀ����
*��    ��: ����Ŀ��ֵ
*�� �� ֵ: ��
**********************************************************************************************************/
void SET_Speed_Target(object_t target)
{
    g_infc.speed_inner_target=target;
}

/**********************************************************************************************************
*�� �� ��: GET_Speed_Measure
*����˵��: �����ٶ��ڻ����Ʋ�����
*��    ��: ��
*�� �� ֵ: ����ֵ
**********************************************************************************************************/
object_t GET_Speed_Measure(void)
{
    object_t measure;
    measure.shoot = g_data_2006.speed;
  //  measure.yaw   = g_data_6623.speed[YAW];
    measure.lf    = g_data_3510.speed[LF];
    
    return measure;
}
/**********************************************************************************************************
*�� �� ��: Angle_Out_Control
*����˵��: ����Ƕ��⻷����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Angle_Out_Control(object_t measure,float deltaT)
{
    
    
    //����Ƕȿ������
    g_infc.angle_outer_error.shoot  = g_infc.angle_outer_target.shoot-measure.shoot;
    g_infc.angle_outer_error.yaw    = g_infc.angle_outer_target.yaw-measure.yaw;
    g_infc.angle_outer_error.pitch  = g_infc.angle_outer_target.pitch-measure.pitch;
    
    //��������
    g_infc.angle_outer_error.shoot  = ApplyDeadbandFloat(g_infc.angle_outer_error.shoot,0.05);
    g_infc.angle_outer_error.yaw    = ApplyDeadbandFloat(g_infc.angle_outer_error.yaw,0.05);
    g_infc.angle_outer_error.pitch  = ApplyDeadbandFloat(g_infc.angle_outer_error.pitch,0.05);
    
    
    //PID�㷨��������ǶȻ��Ŀ�����
    s_angle_contorl_out.shoot         = PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,deltaT);
    s_angle_contorl_out.yaw           = PID_GetPID(&g_infc.pid[YAW_ANGLE],g_infc.angle_outer_error.yaw,deltaT);
    s_angle_contorl_out.pitch         = PID_GetPID(&g_infc.pid[PITCH_ANGLE],g_infc.angle_outer_error.pitch,deltaT);
    
    //����޷�
    s_angle_contorl_out.shoot         = ConstrainFloat(s_angle_contorl_out.shoot,-10000,10000);
    s_angle_contorl_out.yaw           = ConstrainFloat(s_angle_contorl_out.yaw,-5000,5000);
    s_angle_contorl_out.pitch         = ConstrainFloat(s_angle_contorl_out.pitch,-10000,10000);
    
    //���Ƕ��⻷��������Ϊ�ٶ��ڻ��Ŀ���Ŀ��
    SET_Speed_Target(s_angle_contorl_out); 
}


/**********************************************************************************************************
*�� �� ��: SET_Speed_Target
*����˵��: ���ýǶ��⻷����Ŀ����
*��    ��: ����Ŀ��ֵ
*�� �� ֵ: ��
**********************************************************************************************************/
void SET_Angle_Target(object_t target)
{
    g_infc.angle_outer_target=target;
}

/**********************************************************************************************************
*�� �� ��: GET_Angle_Measure
*����˵��: ���ýǶ��⻷���Ʋ�����
*��    ��: ��
*�� �� ֵ: ����ֵ
**********************************************************************************************************/
object_t GET_Angle_Measure(void)
{
    object_t measure;
    
    measure.shoot=g_data_2006.angle;
    measure.yaw=g_data_6623.angle[YAW];
    measure.pitch=g_data_6623.angle[PITCH];
    
    return measure;
}






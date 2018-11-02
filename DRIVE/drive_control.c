#include <drive_control.h>
#include <motor_chassis.h>
#include <motor_cradle_head.h>

infantry_control_t g_infc;

object_t g_speed_target;
object_t g_angle_target;

static void PID_Reset(void)
{
    
    PID_SetParam(&g_infc.pid[SHOOT_SPEED],5.0,0.1,0,5000,0);
  //  PID_SetParam(&g_infc.pid[SHOOT_ANGLE],180,0,0,5000,0);
    
    PID_SetParam(&g_infc.pid[LF_SPEED],3.5,1.26,0,5000,35);
    
    
    
    
}
void Infan_Control_Init(void)
{
     //g_angle_target.shoot=60;
   // SET_Angle_Target(g_angle_target);
   // g_speed_target.lf=5000;
    g_speed_target.shoot=5000;
      SET_Speed_Target(g_speed_target);
    PID_Reset();
   
}
/******************************************************************
*�� �� ��: Speed_In_Control
*����˵��: �ٶ��ڻ�����
*��    ��: �ٶȲ���ֵ ����ʱ����
*�� �� ֵ: �ٶ��ڻ�������
***********************************************************************/

void Speed_In_Control(object_t measure,float deltaT)
{
    object_t speed_contorl_out;
    
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
    speed_contorl_out.shoot         = PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
    speed_contorl_out.yaw           = PID_GetPID(&g_infc.pid[YAW_SPEED],g_infc.speed_inner_error.yaw,deltaT);
    speed_contorl_out.pitch         = PID_GetPID(&g_infc.pid[PITCH_SPEED],g_infc.speed_inner_error.pitch,deltaT);
    speed_contorl_out.lf            = PID_GetPID(&g_infc.pid[LF_SPEED],g_infc.speed_inner_error.lf,deltaT);
    
    //����޷�
    speed_contorl_out.shoot         = ConstrainFloat(speed_contorl_out.shoot,-10000,10000);
    speed_contorl_out.yaw           = ConstrainFloat(speed_contorl_out.yaw,-10000,10000);
    speed_contorl_out.pitch         = ConstrainFloat(speed_contorl_out.pitch,-10000,10000);
    
    speed_contorl_out.lf            = ConstrainFloat(speed_contorl_out.lf,-8000,8000);
    
  
    
    Cmd_2006_ESC(speed_contorl_out.shoot);
   // Cmd_6623_ESC(speed_contorl_out.yaw,speed_contorl_out.pitch);
    Cmd_3510_ESC(speed_contorl_out.lf,speed_contorl_out.la,speed_contorl_out.rf,speed_contorl_out.ra);
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
    measure.shoot=g_data_2006.speed;
    
    measure.lf=g_data_3510.speed[0];
    
  
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
    
    object_t angle_contorl_out;
    
    //����Ƕȿ������
    g_infc.angle_outer_error.shoot  = g_infc.angle_outer_target.shoot-measure.shoot;
    g_infc.angle_outer_error.yaw    = g_infc.angle_outer_target.yaw-measure.yaw;
    g_infc.angle_outer_error.pitch  = g_infc.angle_outer_target.pitch-measure.pitch;
    
    //��������
    g_infc.angle_outer_error.shoot  = ApplyDeadbandFloat(g_infc.angle_outer_error.shoot,0.05);
    g_infc.angle_outer_error.yaw    = ApplyDeadbandFloat(g_infc.angle_outer_error.yaw,0.05);
    g_infc.angle_outer_error.pitch  = ApplyDeadbandFloat(g_infc.angle_outer_error.pitch,0.05);
    
    
    //PID�㷨��������ǶȻ��Ŀ�����
    angle_contorl_out.shoot         = PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,deltaT);
    angle_contorl_out.yaw           = PID_GetPID(&g_infc.pid[YAW_ANGLE],g_infc.angle_outer_error.yaw,deltaT);
    angle_contorl_out.pitch         = PID_GetPID(&g_infc.pid[PITCH_ANGLE],g_infc.angle_outer_error.pitch,deltaT);
    
    //����޷�
    angle_contorl_out.shoot         = ConstrainFloat(angle_contorl_out.shoot,-10000,10000);
    angle_contorl_out.yaw           = ConstrainFloat(angle_contorl_out.yaw,-10000,10000);
    angle_contorl_out.pitch         = ConstrainFloat(angle_contorl_out.pitch,-10000,10000);
    
    //���Ƕ��⻷��������Ϊ�ٶ��ڻ��Ŀ���Ŀ��
    SET_Speed_Target(angle_contorl_out); 
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






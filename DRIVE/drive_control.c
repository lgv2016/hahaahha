#include <drive_control.h>
#include <motor_chassis.h>
#include <motor_cradle_head.h>

infantry_control_t g_infc;

object_t g_speed_target;
object_t g_angle_target;

static void PID_Reset(void)
{
    PID_SetParam(&g_infc.pid[SHOOT_SPEED],5.0,0.1,0,5000,0);
    PID_SetParam(&g_infc.pid[SHOOT_ANGLE],180,0,0,5000,0);
}
void Infan_Control_Init(void)
{
    g_angle_target.shoot=60;
    PID_Reset();
    SET_Angle_Target(g_angle_target);
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
    g_infc.speed_inner_error.shoot=g_infc.speed_inner_target.shoot-measure.shoot;
    
    //��������
    g_infc.speed_inner_error.shoot=ApplyDeadbandFloat( g_infc.speed_inner_error.shoot,10);
    
    //PID�㷨��������ٶȻ��Ŀ�����
    speed_contorl_out.shoot=PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
    
    //����޷�
    speed_contorl_out.shoot=ConstrainFloat(speed_contorl_out.shoot,-10000,10000);
    
    Cmd_2006_ESC(speed_contorl_out.shoot);
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
    g_infc.angle_outer_error.shoot=g_infc.angle_outer_target.shoot-measure.shoot;
    //��������
    g_infc.angle_outer_error.shoot=ApplyDeadbandFloat(g_infc.angle_outer_error.shoot,0.05);
    //PID�㷨��������ǶȻ��Ŀ�����
    angle_contorl_out.shoot=PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,deltaT);
    
    //����޷�
    angle_contorl_out.shoot=ConstrainFloat(angle_contorl_out.shoot,-10000,10000);
    
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
    return measure;
}






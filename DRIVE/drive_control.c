#include <drive_control.h>
#include <motor_chassis.h>
#include <motor_cradle_head.h>


infantry_control_t g_infc;


object_t g_speed_target;
object_t g_angle_target;



static void PID_Reset(void)
{
    PID_SetParam(&g_infc.pid[SHOOT_SPEED],5.0,0.1,0,5000,0);
    
    PID_SetParam(&g_infc.pid[SHOOT_ANGLE],5.0,15.0,0.2,200,0);
    
    
}
void Infan_Control_Init(void)
{
    g_speed_target.shoot=6000;
    g_angle_target.shoot=50;
    
    PID_Reset();
    
    SET_Speed_Target(g_speed_target);
}
/******************************************************************
*函 数 名: Speed_In_Control
*功能说明: 速度内环控制
*形    参: 速度测量值 运行时间间隔
*返 回 值: 速度内环控制量
***********************************************************************/

void Speed_In_Control(object_t measure,float deltaT)
{
    object_t speed_contorl_out;
    
    
    //计算速度环控制误差
    g_infc.speed_inner_error.shoot=g_infc.speed_inner_target.shoot-measure.shoot;
    
    //死区控制
    g_infc.speed_inner_error.shoot=ApplyDeadbandFloat( g_infc.speed_inner_error.shoot,10);
    
    //PID算法，计算出速度环的控制量
    speed_contorl_out.shoot=PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
    

    //输出限幅
    speed_contorl_out.shoot=ConstrainFloat(speed_contorl_out.shoot,-10000,10000);
    
    Cmd_2006_ESC(speed_contorl_out.shoot);
    //return speed_contorl_out;
}
/**********************************************************************************************************
*函 数 名: SET_Speed_Target
*功能说明: 设置速度内环控制目标量
*形    参: 控制目标值
*返 回 值: 无
**********************************************************************************************************/
void SET_Speed_Target(object_t target)
{
    g_infc.speed_inner_target=target;
}

/**********************************************************************************************************
*函 数 名: GET_Speed_Measure
*功能说明: 设置速度内环控制测量量
*形    参: 无
*返 回 值: 测量值
**********************************************************************************************************/
object_t GET_Speed_Measure(void)
{
    object_t measure;
    measure.shoot=g_data_2006.speed;
    return measure;
}
/**********************************************************************************************************
*函 数 名: Angle_Out_Control
*功能说明: 电机角度外环控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Angle_Out_Control(object_t measure,float deltaT)
{
    
    object_t angle_contorl_out;
    
    //计算角度控制误差
    g_infc.angle_outer_error.shoot=g_infc.angle_outer_target.shoot-measure.shoot;
    //死区控制
    g_infc.angle_outer_error.shoot=ApplyDeadbandFloat(g_infc.angle_outer_error.shoot,0.2);
    //PID算法，计算出角度环的控制量
    angle_contorl_out.shoot=PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,deltaT);
    
    //输出限幅
    angle_contorl_out.shoot=ConstrainFloat(angle_contorl_out.shoot,-360,360);
    
    //将角度外环控制量作为速度内环的控制目标
    SET_Speed_Target(angle_contorl_out);
    
}


/**********************************************************************************************************
*函 数 名: SET_Speed_Target
*功能说明: 设置角度外环控制目标量
*形    参: 控制目标值
*返 回 值: 无
**********************************************************************************************************/
void SET_Angle_Target(object_t target)
{
    g_infc.angle_outer_target=target;
}

/**********************************************************************************************************
*函 数 名: GET_Angle_Measure
*功能说明: 设置角度外环控制测量量
*形    参: 无
*返 回 值: 测量值
**********************************************************************************************************/
object_t GET_Angle_Measure(void)
{
    object_t measure;
    measure.shoot=g_data_2006.angle;
    return measure;
    
    
}






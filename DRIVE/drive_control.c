#include <drive_control.h>
#include <motor_chassis.h>
#include <motor_cradle_head.h>
 
infantry_control_t g_infc;

object_t g_speed_target;
object_t g_angle_target;

static object_t s_angle_contorl_out;
static object_t s_speed_contorl_out;




static void PID_Reset(void)
{
    PID_SetParam(&g_infc.pid[SHOOT_SPEED],5.0, 0.1 ,  0,  5000,   0);
    PID_SetParam(&g_infc.pid[SHOOT_ANGLE],180, 0   ,  0,  5000,   0);
	
    PID_SetParam(&g_infc.pid[PITCH_ANGLE],5.0, 0.1 ,  0,  2000,   0);
	
	PID_SetParam(&g_infc.pid[LF_SPEED],   3.5, 1.26,  0,  5000,   0);
}
void Infan_Control_Init(void)
{
   g_speed_target.shoot=3500;
   g_speed_target.lf=3200;

   PID_Reset();
}
void Angle_6623_Control(object_t target,float deltaT)
{
	
}



void Speed_3510_Control(object_t target,float deltaT)
{
	
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
	g_infc.speed_inner_error.lf     = ApplyDeadbandFloat( g_infc.speed_inner_error.lf,10);
	g_infc.speed_inner_error.la     = ApplyDeadbandFloat( g_infc.speed_inner_error.la,10);
	g_infc.speed_inner_error.rf     = ApplyDeadbandFloat( g_infc.speed_inner_error.rf,10);
	g_infc.speed_inner_error.ra     = ApplyDeadbandFloat( g_infc.speed_inner_error.ra,10);
	
	//PID算法，计算出速度环的控制量
	s_speed_contorl_out.lf          = PID_GetPID(&g_infc.pid[LF_SPEED],g_infc.speed_inner_error.lf,deltaT);
	s_speed_contorl_out.la          = PID_GetPID(&g_infc.pid[LA_SPEED],g_infc.speed_inner_error.la,deltaT);
	s_speed_contorl_out.rf          = PID_GetPID(&g_infc.pid[RF_SPEED],g_infc.speed_inner_error.rf,deltaT);
	s_speed_contorl_out.ra          = PID_GetPID(&g_infc.pid[RA_SPEED],g_infc.speed_inner_error.ra,deltaT);
	
	//输出限幅
	s_speed_contorl_out.lf          = ConstrainFloat(s_speed_contorl_out.lf,-8000,8000);
	s_speed_contorl_out.la          = ConstrainFloat(s_speed_contorl_out.la,-8000,8000);
	s_speed_contorl_out.rf          = ConstrainFloat(s_speed_contorl_out.rf,-8000,8000);
	s_speed_contorl_out.ra          = ConstrainFloat(s_speed_contorl_out.ra,-8000,8000);
	
	//can发送控制量
	Cmd_3510_ESC(s_speed_contorl_out.lf,s_speed_contorl_out.la,s_speed_contorl_out.rf,s_speed_contorl_out.ra);
}



void Speed_2006_Control(object_t target,float deltaT)
{
	
	//速度环目标赋值
    g_infc.speed_inner_target.shoot		= target.shoot;
	
	//计算速度环控制误差
    g_infc.speed_inner_error.shoot  	= g_infc.speed_inner_target.shoot - g_data_2006.speed;
	
	//死区控制
    g_infc.speed_inner_error.shoot  	= ApplyDeadbandFloat( g_infc.speed_inner_error.shoot,10);
	
	//PID算法，计算出速度环的控制量
    s_speed_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
	
	//输出限幅
    s_speed_contorl_out.shoot         	= ConstrainFloat(s_speed_contorl_out.shoot,-10000,10000);
	
	//can发送控制量
	//Cmd_2006_ESC(s_speed_contorl_out.shoot);
}

void Angle_2006_Control(object_t target,float deltaT)
{
	
	//角度环目标赋值
	g_infc.angle_outer_target.shoot		= target.shoot;
	
	
	//计算角度控制误差
	g_infc.angle_outer_error.shoot  	= g_infc.angle_outer_target.shoot-g_data_2006.angle;
	
	//死区控制
    g_infc.angle_outer_error.shoot  	= ApplyDeadbandFloat(g_infc.angle_outer_error.shoot,0.05);
	
	//PID算法，计算出角度环的控制量
    s_angle_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,deltaT);
	
	//输出限幅
    s_angle_contorl_out.shoot         	= ConstrainFloat(s_angle_contorl_out.shoot,-10000,10000);
	
	//将角度外环控制量作为速度内环的控制目标
	g_infc.speed_inner_target.shoot		= s_angle_contorl_out.shoot;
	
	//计算速度环控制误差
    g_infc.speed_inner_error.shoot  	= g_infc.speed_inner_target.shoot - g_data_2006.speed;
	
	//死区控制
    g_infc.speed_inner_error.shoot  	= ApplyDeadbandFloat( g_infc.speed_inner_error.shoot,10);
	
	 //PID算法，计算出速度环的控制量
    s_speed_contorl_out.shoot         	= PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
	
	 //输出限幅
    s_speed_contorl_out.shoot         	= ConstrainFloat(s_speed_contorl_out.shoot,-10000,10000);
	
	//can发送控制量
    Cmd_2006_ESC(s_speed_contorl_out.shoot);
}


/******************************************************************
*函 数 名: Speed_In_Control
*功能说明: 速度内环控制
*形    参: 速度测量值 运行时间间隔
*返 回 值: 速度内环控制量
***********************************************************************/

void Speed_In_Control(object_t measure,float deltaT)
{
    //object_t s_speed_contorl_out;
    
    //计算速度环控制误差
    g_infc.speed_inner_error.shoot  = g_infc.speed_inner_target.shoot   - measure.shoot;
    g_infc.speed_inner_error.yaw    = g_infc.speed_inner_target.yaw     - measure.yaw;
    g_infc.speed_inner_error.pitch  = g_infc.speed_inner_target.pitch   - measure.pitch;
    g_infc.speed_inner_error.lf     = g_infc.speed_inner_target.lf      - measure.lf;
    
    

    //死区控制
    g_infc.speed_inner_error.shoot  = ApplyDeadbandFloat( g_infc.speed_inner_error.shoot,10);
    g_infc.speed_inner_error.yaw    = ApplyDeadbandFloat( g_infc.speed_inner_error.yaw,0.1);
    g_infc.speed_inner_error.pitch  = ApplyDeadbandFloat( g_infc.speed_inner_error.pitch,0.1);
    g_infc.speed_inner_error.lf     = ApplyDeadbandFloat( g_infc.speed_inner_error.lf,10);

    
    //PID算法，计算出速度环的控制量
    s_speed_contorl_out.shoot         = PID_GetPID(&g_infc.pid[SHOOT_SPEED],g_infc.speed_inner_error.shoot,deltaT);
    s_speed_contorl_out.yaw           = PID_GetPID(&g_infc.pid[YAW_SPEED],g_infc.speed_inner_error.yaw,deltaT);
    s_speed_contorl_out.pitch         = PID_GetPID(&g_infc.pid[PITCH_SPEED],g_infc.speed_inner_error.pitch,deltaT);
    s_speed_contorl_out.lf            = PID_GetPID(&g_infc.pid[LF_SPEED],g_infc.speed_inner_error.lf,deltaT);
    
    //输出限幅
    s_speed_contorl_out.shoot         = ConstrainFloat(s_speed_contorl_out.shoot,-10000,10000);
    s_speed_contorl_out.yaw           = ConstrainFloat(s_speed_contorl_out.yaw,-5000,5000);
    s_speed_contorl_out.pitch         = ConstrainFloat(s_speed_contorl_out.pitch,-5000,5000);
    s_speed_contorl_out.lf            = ConstrainFloat(s_speed_contorl_out.lf,-8000,8000);
    
  
    
    Cmd_2006_ESC(s_speed_contorl_out.shoot);
    Cmd_6623_ESC(s_speed_contorl_out.yaw,s_speed_contorl_out.pitch);
    Cmd_3510_ESC(s_speed_contorl_out.lf,s_speed_contorl_out.la,s_speed_contorl_out.rf,s_speed_contorl_out.ra);
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
    measure.shoot = g_data_2006.speed;
    measure.yaw   = g_data_6623.speed[YAW];
    measure.lf    = g_data_3510.speed[LF];
    
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
    
   // object_t s_angle_contorl_out;
    
    //计算角度控制误差
    g_infc.angle_outer_error.shoot  = g_infc.angle_outer_target.shoot-measure.shoot;
    g_infc.angle_outer_error.yaw    = g_infc.angle_outer_target.yaw-measure.yaw;
    g_infc.angle_outer_error.pitch  = g_infc.angle_outer_target.pitch-measure.pitch;
    
    //死区控制
    g_infc.angle_outer_error.shoot  = ApplyDeadbandFloat(g_infc.angle_outer_error.shoot,0.05);
    g_infc.angle_outer_error.yaw    = ApplyDeadbandFloat(g_infc.angle_outer_error.yaw,0.05);
    g_infc.angle_outer_error.pitch  = ApplyDeadbandFloat(g_infc.angle_outer_error.pitch,0.05);
    
    
    //PID算法，计算出角度环的控制量
    s_angle_contorl_out.shoot         = PID_GetPID(&g_infc.pid[SHOOT_ANGLE],g_infc.angle_outer_error.shoot,deltaT);
    s_angle_contorl_out.yaw           = PID_GetPID(&g_infc.pid[YAW_ANGLE],g_infc.angle_outer_error.yaw,deltaT);
    s_angle_contorl_out.pitch         = PID_GetPID(&g_infc.pid[PITCH_ANGLE],g_infc.angle_outer_error.pitch,deltaT);
    
    //输出限幅
    s_angle_contorl_out.shoot         = ConstrainFloat(s_angle_contorl_out.shoot,-10000,10000);
    s_angle_contorl_out.yaw           = ConstrainFloat(s_angle_contorl_out.yaw,-5000,5000);
    s_angle_contorl_out.pitch         = ConstrainFloat(s_angle_contorl_out.pitch,-10000,10000);
    
    //将角度外环控制量作为速度内环的控制目标
    SET_Speed_Target(s_angle_contorl_out); 
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
    measure.yaw=g_data_6623.angle[YAW];
    measure.pitch=g_data_6623.angle[PITCH];
    
    return measure;
}






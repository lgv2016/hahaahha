#include <drive_chassis.h>
#include <drive_control.h>
#include <drive_delay.h> 
#include <drive_imu.h>
#include <motor_chassis.h>
#include <motor_cradle_head.h>
#include <robotstatus.h>
#include <drive_rc.h>


#include <drive_gimble.h>
#ifdef  RED1


#define M3510_3508_RATE 1.0f

#define NORMAL_MAX_CHASSIS_SPEED_X  6500*M3510_3508_RATE
#define NORMAL_MAX_CHASSIS_SPEED_Y  6500*M3510_3508_RATE

#define CHASSIS_MAX_FOLLOW_SPEED_Z  120.0f*M3510_3508_RATE
#define CHASSIS_FOLLOW_LIMIT        20.0f

#define CHASSIS_ROTATE_SPEED_Z        150.0f*M3510_3508_RATE

#define CHASSIS_ROTATE_RUN_SPEED_Z    120.0f*M3510_3508_RATE
#define CHASSIS_ROTATE_RUN_SPEED_X    4000*M3510_3508_RATE
#define CHASSIS_ROTATE_RUN_SPEED_Y    4000*M3510_3508_RATE

#define ADD_CHASSIS_SPEED  -3000*M3510_3508_RATE

#endif

#ifdef  RED2
#define M3510_3508_RATE 0.704f

#define NORMAL_MAX_CHASSIS_SPEED_X  7000*M3510_3508_RATE
#define NORMAL_MAX_CHASSIS_SPEED_Y  7000*M3510_3508_RATE

#define CHASSIS_MAX_FOLLOW_SPEED_Z  130.0f*M3510_3508_RATE
#define CHASSIS_FOLLOW_LIMIT        20.0f

#define CHASSIS_ROTATE_SPEED_Z        180.0f*M3510_3508_RATE

#define CHASSIS_ROTATE_RUN_SPEED_Z    130.0f*M3510_3508_RATE
#define CHASSIS_ROTATE_RUN_SPEED_X    4000*M3510_3508_RATE
#define CHASSIS_ROTATE_RUN_SPEED_Y    4000*M3510_3508_RATE

#define ADD_CHASSIS_SPEED  -3000*M3510_3508_RATE
#endif




#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.1666666667f
#define CHASSIS_ACCEL_W_NUM 0.1666666667f

#define RC_SW_UP   ((uint16_t)1)
#define RC_SW_MID  ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)



#define CH_POWER 80
#define CH_POWER_T 0.02
#define CH_POWER_BUFFER_LIMIT 20


#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s)  (s == RC_SW_MID)
#define switch_is_up(s)   (s == RC_SW_UP)
chassis_move_t chassis_move;


static float motor_ecd_to_angle_change(float ecd, float offset_ecd);

void CHASSIS_Init()
{
	static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
	static float chassis_w_order_filter[1] = {CHASSIS_ACCEL_W_NUM};
	
	robot_status.chassis_mode = CHASSIS_STOP;
	
	//用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_CYCLE/1000.0f, chassis_x_order_filter);
    first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vy, CHASSIS_CONTROL_CYCLE/1000.0f, chassis_y_order_filter);
	first_order_filter_init(&chassis_move.chassis_cmd_slow_set_wz, CHASSIS_CONTROL_CYCLE/1000.0f, chassis_w_order_filter);

	    //最大 最小速度
    chassis_move.vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move.vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
}


void CHASSIS_Set_Mode()
{	
    if(g_rc_control.key.k[W]||g_rc_control.key.k[A]||g_rc_control.key.k[S]||g_rc_control.key.k[D])
    {
        robot_status.chassis_mode=CHASSIS_FOLLOW_GIMBLE;
    }
	

	if(switch_is_mid(g_rc_control.rc.s1))
	{
		robot_status.chassis_mode=CHASSIS_FOLLOW_GIMBLE;
	}


//	if(!g_rc_control.key.k[W]&&!g_rc_control.key.k[A]&&!g_rc_control.key.k[S]&&!g_rc_control.key.k[D])
//	{
//		robot_status.chassis_mode=CHASSIS_NO_FOLLOW_GIMBLE;
//	}
	

	
    if(g_rc_control.key.k[CTRL])
    {
        robot_status.chassis_mode=CHASSIS_ROTATE;
    }
	
	
	if(g_rc_control.key.k[CTRL]&&(g_rc_control.key.k[W]||g_rc_control.key.k[A]||g_rc_control.key.k[S]||g_rc_control.key.k[D]))
	{
		robot_status.chassis_mode=CHASSIS_ROTATE_RUN;
	}
	


	
	if(switch_is_up(g_rc_control.rc.s1))
	{
		robot_status.chassis_mode=CHASSIS_FOLLOW_GIMBLE;
	}
	
	if(switch_is_down(g_rc_control.rc.s1))
	{
		robot_status.chassis_mode=CHASSIS_NO_FOLLOW_GIMBLE;
	}
	
	    //如果云台没有校准完成
    if(robot_status.gimbal_data!=GIMBAL_MOTOR_GYRO)
	{
		  robot_status.chassis_mode = CHASSIS_STOP;
	}
}

//遥控器的数据处理成底盘的前进vx速度，vy速度
void CHASSIS_RC_Control_Value(float *vx_set, float *vy_set)
{

    //遥控器原始通道值
    float vx_set_channel, vy_set_channel;
	
	if(g_rc_control.rc.ch3&&g_rc_control.rc.ch2)
	{
//		vx_set_channel = (chassis_move.vx_max_speed/(660.0f))*(g_rc_control.rc.ch3-1024);
//		vy_set_channel= -(chassis_move.vy_max_speed/(660.0f))*(g_rc_control.rc.ch2-1024);
		
		vx_set_channel = (4000/(660.0f))*(g_rc_control.rc.ch3-1024);
		vy_set_channel= -(4000/(660.0f))*(g_rc_control.rc.ch2-1024);
	}
   
	
    if(g_rc_control.key.k[SHIFT])
	{
		chassis_move.chassis_speed_add=ADD_CHASSIS_SPEED;
	}
	else
	{
		chassis_move.chassis_speed_add=0;
	}
	
    if (g_rc_control.key.k[W])
    {
		if(robot_status.chassis_mode==CHASSIS_ROTATE_RUN)
		{
			vx_set_channel=CHASSIS_ROTATE_RUN_SPEED_X;
		}
		else
		{
			vx_set_channel = chassis_move.vx_max_speed+chassis_move.chassis_speed_add;
		}

    }
    else if (g_rc_control.key.k[S])
    {
		if(robot_status.chassis_mode==CHASSIS_ROTATE_RUN)
		{
			vx_set_channel=-CHASSIS_ROTATE_RUN_SPEED_X;
		}
		else
		{
			vx_set_channel = chassis_move.vx_min_speed-chassis_move.chassis_speed_add;
		}
    }

    if (g_rc_control.key.k[A])
    {
		if(robot_status.chassis_mode==CHASSIS_ROTATE_RUN)
		{
			vy_set_channel=CHASSIS_ROTATE_RUN_SPEED_Y;
		}
		else
		{
			vy_set_channel = chassis_move.vy_max_speed+chassis_move.chassis_speed_add;
		}
    }
    else if (g_rc_control.key.k[D])
    {
		if(robot_status.chassis_mode==CHASSIS_ROTATE_RUN)
		{
			vy_set_channel=-CHASSIS_ROTATE_RUN_SPEED_Y;
		}
		else
		{
			vy_set_channel = chassis_move.vy_min_speed-chassis_move.chassis_speed_add;
		}
    }

	if(robot_status.chassis_mode==CHASSIS_ROTATE_RUN)
	{
		chassis_move.chassis_rotate_speed_set=CHASSIS_ROTATE_RUN_SPEED_Z;
	}
	
	else
	{
		chassis_move.chassis_rotate_speed_set=0.0f;
	}
	
	if(robot_status.chassis_mode==CHASSIS_ROTATE)
	{
	    chassis_move.chassis_rotate_speed_set=CHASSIS_ROTATE_SPEED_Z;
	}

	
    //一阶低通滤波代替斜波作为底盘速度输入
	first_order_filter_cali(&chassis_move.chassis_cmd_slow_set_wz,chassis_move.chassis_rotate_speed_set);
    first_order_filter_cali(&chassis_move.chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move.chassis_cmd_slow_set_vy, vy_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
	
    if (abs(vx_set_channel) < 100)
    {
        chassis_move.chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (abs(vy_set_channel) < 100)
    {
        chassis_move.chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move.chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move.chassis_cmd_slow_set_vy.out;
}
void CHASSIS_Rotate_Control(float *vx_set, float *vy_set, float *wz_set)
{	
	CHASSIS_RC_Control_Value(vx_set,vy_set);
	
    *wz_set =chassis_move.chassis_cmd_slow_set_wz.out;
}

void CHASSIS_Stop_Control(float *vx_set, float *vy_set, float *wz_set)
{
	*vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

void CHASSIS_Follow_Gimble_Control(float *vx_set, float *vy_set, float *angle_set)
{
	CHASSIS_RC_Control_Value(vx_set,vy_set);
	*angle_set =180.0f;
}

void CHASSIS_NO_Follow_Gimble_Control(float *vx_set, float *vy_set, float *wz_set)
{
	CHASSIS_RC_Control_Value(vx_set,vy_set);
}
void CHASSIS_Rotate_Run_Control(float *vx_set, float *vy_set, float *wz_set)
{
	CHASSIS_RC_Control_Value(vx_set,vy_set);
    *wz_set =chassis_move.chassis_cmd_slow_set_wz.out;
}

void CHASSIS_Mode_Control_Set(float *vx_set, float *vy_set, float *angle_set)
{
	if(robot_status.chassis_mode==CHASSIS_FOLLOW_GIMBLE)
	{
		CHASSIS_Follow_Gimble_Control(vx_set,vy_set,angle_set);
	}
	
	else if(robot_status.chassis_mode==CHASSIS_NO_FOLLOW_GIMBLE)
	{
		CHASSIS_NO_Follow_Gimble_Control(vx_set,vy_set,angle_set);
	}
	
	else if(robot_status.chassis_mode==CHASSIS_ROTATE)
	{
		CHASSIS_Rotate_Control(vx_set,vy_set,angle_set);
	}
	
	else if(robot_status.chassis_mode==CHASSIS_STOP)
	{
		CHASSIS_Stop_Control(vx_set,vy_set,angle_set);
	}
	
	else if(robot_status.chassis_mode==CHASSIS_ROTATE_RUN)
	{
		CHASSIS_Rotate_Run_Control(vx_set,vy_set,angle_set);
	}
}


//////设置遥控器输入控制量
static void CHASSIS_Set_Control()
{
    //设置速度
    float vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
	float sin_yaw=0.0f,cos_yaw=0.0f;
    CHASSIS_Mode_Control_Set(&vx_set, &vy_set, &angle_set);
	
	if(robot_status.chassis_mode==CHASSIS_FOLLOW_GIMBLE)
	{
		float relative_angle=-motor_ecd_to_angle_change(g_data_6623.angle[YAW],180.0f);
	    chassis_move.chassis_relative_angle=Radians(relative_angle);
		
        sin_yaw = arm_sin_f32(chassis_move.chassis_relative_angle);
        cos_yaw = arm_cos_f32(chassis_move.chassis_relative_angle);
		
		
		g_angle_target.ch_rotate=angle_set;
		chassis_move.wz_set =-Angle_Rotate_Control(g_angle_target);
		chassis_move.wz_set=Constrainfloat(chassis_move.wz_set,-CHASSIS_MAX_FOLLOW_SPEED_Z,CHASSIS_MAX_FOLLOW_SPEED_Z);
		if(abs(g_infc.angle_outer_error.ch_rotate)<CHASSIS_FOLLOW_LIMIT)
		{
			chassis_move.vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
			chassis_move.vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
		}
		
		else
		{
			chassis_move.vx_set=0.0f;
			chassis_move.vy_set=0.0f;
		}
	}
	
	else if(robot_status.chassis_mode==CHASSIS_NO_FOLLOW_GIMBLE)
	{
		chassis_move.wz_set = angle_set;
		chassis_move.vx_set = vx_set;
        chassis_move.vy_set = vy_set;
	}
	else if((robot_status.chassis_mode==CHASSIS_ROTATE)||(robot_status.chassis_mode==CHASSIS_ROTATE_RUN))
	{
		float relative_angle=-motor_ecd_to_angle_change(g_data_6623.angle[YAW],180.0f);
	    chassis_move.chassis_relative_angle=Radians(relative_angle);
	
        sin_yaw = arm_sin_f32(chassis_move.chassis_relative_angle);
        cos_yaw = arm_cos_f32(chassis_move.chassis_relative_angle);
		
		chassis_move.wz_set = angle_set;
		chassis_move.vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move.vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
	}
	
	else if(robot_status.chassis_mode==CHASSIS_STOP)
	{
		chassis_move.wz_set = angle_set;
		chassis_move.vx_set = vx_set;
        chassis_move.vy_set = vy_set;
	}
}
		
void CHASSIS_Loop_Control()
{
	CHASSIS_Set_Mode();
	CHASSIS_Set_Control();
	Speed_Chassis_Control(chassis_move.vx_set,chassis_move.vy_set,chassis_move.wz_set);
}


//计算相对角度
static float motor_ecd_to_angle_change(float ecd, float offset_ecd)
{
    float relative_ecd = ecd - offset_ecd;
    if (relative_ecd > 180.0f)
    {
        relative_ecd -= 360.0f;
    }
    else if (relative_ecd < -180.0f)
    {
        relative_ecd += 360.0f;
    }

    return relative_ecd;
}


//最大功率计算

//void CHASSIS_Max_Power()
//{
//	chassis_move.max_power=((judge_data.power_buffer-CH_POWER_BUFFER_LIMIT)/CH_POWER_T)+CH_POWER;
//	
//	if(chassis_move.max_power<CH_POWER)
//	{
//		chassis_move.max_power=CH_POWER;
//	}
//}


//void CHASSIS_Speed_Set(float speed_set)
//{
//	
//}

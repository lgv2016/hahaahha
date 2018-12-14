#ifndef __DRIVE_CONTROL_H
#define __DRIVE_CONTROL_H

#include <math_pid.h>
#include <math_tool.h>


enum
{
    YAW_ANGLE,
    YAW_SPEED,
    PITCH_ANGLE,
    PITCH_SPEED,
    SHOOT_ANGLE,
    SHOOT_SPEED,
    
    LF_SPEED,    //左前
    LA_SPEED,
    RF_SPEED,
    RA_SPEED,    //右后
	
	CH_ROTATE_SPEED,  //自旋速度控制
	CH_ROTATE_ANGLE,  //自旋角度控制
	
    PIDNUM
};

typedef struct
{
    float yaw;
    float pitch;
    float shoot;
    float lf;
    float la;
    float rf;
    float ra;
	
	float ch_rotate;

} object_t;

typedef struct
{
    PID_t      pid[PIDNUM];         //PID参数结构体
    object_t speed_inner_target;
    object_t angle_outer_target;
    object_t speed_inner_error;
    object_t angle_outer_error;

} infantry_control_t;

extern infantry_control_t infc;
extern object_t g_speed_target;
extern object_t g_angle_target;

extern void Infan_Control_Init(void);
extern void Speed_3510_Control(object_t target);
extern void Angle_6623_Control(object_t target);
extern void Speed_2006_Control(object_t target);
extern void Angle_2006_Control(object_t target);

extern void Speed_Chassis_Control(float xspeed,float yspeed,float yawspeed);
extern void Speed_Rotate_Control(object_t target);
extern void Angle_Rotate_Control(object_t target);



#endif

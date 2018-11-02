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
extern void Infan_Control_Init(void);
extern void SET_Speed_Target(object_t target);
extern object_t GET_Speed_Measure(void);
extern void Speed_In_Control(object_t measure,float deltaT);

extern void Angle_Out_Control(object_t measure,float deltaT);
extern void SET_Angle_Target(object_t target);
extern object_t GET_Angle_Measure(void);

#endif
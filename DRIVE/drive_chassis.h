#ifndef __DRIVE_CHASSIS_H
#define __DRIVE_CHASSIS_H


#include <stm32f4xx.h>
#include <math_tool.h>

#define CHASSIS_CONTROL_CYCLE 2


typedef struct
{
  first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;
  first_order_filter_type_t chassis_cmd_slow_set_wz;

  float vx;                         //底盘速度 前进方向 前为正，单位 m/s
  float vy;                         //底盘速度 左右方向 左为正  单位 m/s
  float wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
  float vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
  float vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
  float wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
	
  float chassis_rotate_angle_set;
  float chassis_rotate_speed_set;
	
   float chassis_speed_add;
   float chassis_relative_angle;     //底盘与云台的相对角度，单位 rad/s
   float chassis_absolute_angle;

 

  float vx_max_speed;  //前进方向最大速度 单位m/s
  float vx_min_speed;  //前进方向最小速度 单位m/s
  float vy_max_speed;  //左右方向最大速度 单位m/s
  float vy_min_speed;  //左右方向最小速度 单位m/s
  float chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
  float chassis_yaw_set;

} chassis_move_t;


extern void CHASSIS_Loop_Control(void);
extern void CHASSIS_Init(void);
extern void Get_CHASSIS_data(CanRxMsg rx_message);
#endif



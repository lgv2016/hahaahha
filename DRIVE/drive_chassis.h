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

  float vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  float vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  float wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  float vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  float wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
	
  float chassis_rotate_angle_set;
  float chassis_rotate_speed_set;
	
   float chassis_speed_add;
   float chassis_relative_angle;     //��������̨����ԽǶȣ���λ rad/s
   float chassis_absolute_angle;

 

  float vx_max_speed;  //ǰ����������ٶ� ��λm/s
  float vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
  float vy_max_speed;  //���ҷ�������ٶ� ��λm/s
  float vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
  float chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
  float chassis_yaw_set;

} chassis_move_t;


extern void CHASSIS_Loop_Control(void);
extern void CHASSIS_Init(void);
extern void Get_CHASSIS_data(CanRxMsg rx_message);
#endif



#ifndef __MATH_TOOL_H
#define __MATH_TOOL_H

#include <stm32f4xx.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>


#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))

#define M_PI                 3.141592653f	//Բ����
#define DEG_TO_RAD           0.01745329f	//�Ƕ�ת����
#define RAD_TO_DEG           57.29577951f	//����ת�Ƕ�

#define EARTH_RADIUS         6371.004f      //km
#define GRAVITY_ACCEL        9.8f          //�������ٶ� ��λ��m/s^2

#define HALF_SQRT_2          0.70710678118654757f	//����2��ֵ

typedef __packed struct
{
    float input;        //��������
    float out;          //�˲����������
    float num[1];       //�˲�����
    float frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;

float Constrainfloat(float amt, float low, float high);

int32_t ApplyDeadbandInt(int32_t value, int32_t deadband);
float ApplyDeadbandfloat(float value, float deadband);

float Radians(float deg);
float Degrees(float rad);

extern u8 Get_Check_SUM(u8* pchMessage, u16 dwLength);
extern u8 Verify_Check_SUM(u8* pchMessage, u16 dwLength);
extern void Append_Check_SUM(u8* pchMessage, u16 dwLength);


extern float HEX_TO_float(u8 * pchMessage);
extern void float_TO_Hex(u8 * pchMessage,float fdata);

extern void FILTER_Limit(u16 lastdata,u16 predata,u16 limit);

extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);


#endif  

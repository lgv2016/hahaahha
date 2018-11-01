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


float ConstrainFloat(float amt, float low, float high);

int32_t ApplyDeadbandInt(int32_t value, int32_t deadband);
float ApplyDeadbandFloat(float value, float deadband);

float Radians(float deg);
float Degrees(float rad);



#endif  
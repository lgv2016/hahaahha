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

#define M_PI                 3.141592653f	//圆周率
#define DEG_TO_RAD           0.01745329f	//角度转弧度
#define RAD_TO_DEG           57.29577951f	//弧度转角度

#define EARTH_RADIUS         6371.004f      //km
#define GRAVITY_ACCEL        9.8f          //重力加速度 单位：m/s^2

#define HALF_SQRT_2          0.70710678118654757f	//根号2的值


float ConstrainFloat(float amt, float low, float high);

int32_t ApplyDeadbandInt(int32_t value, int32_t deadband);
float ApplyDeadbandFloat(float value, float deadband);

float Radians(float deg);
float Degrees(float rad);



#endif  
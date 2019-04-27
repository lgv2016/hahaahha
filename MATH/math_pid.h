#ifndef __MATH_PID_H
#define __MATH_PID_H

#include <stm32f4xx.h>

typedef struct
{
	
	u8 mode;
	
    float kP;
    float kI;
    float kD;

    float imax;            //抗积分饱和阈值
    float integrator;
    float lastError;
    float lastDerivative;
    float dFilter;         //低通滤波截至频率
    
	float lineval;         //积分分离阈值
	
    float target;
    float error;
    
    float maxout;       
    
} PID_t;

float PID_GetP(PID_t* pid, float error);
float PID_GetI(PID_t* pid, float error, float dt);
float PID_GetD(PID_t* pid, float error, float dt);

float PID_GetPID(PID_t* pid, float error, float dt);
float PID_GetPIDD(PID_t* pid, float error,float error_delta,float dt);

void PID_SetParam(PID_t* pid, float p, float i, float d, float ilineval,float imaxval,float dCutFreq);



void PID_ResetParam(PID_t* pid);


#endif  

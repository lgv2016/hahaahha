#ifndef __MATH_PID_H
#define __MATH_PID_H

#include <stm32f4xx.h>

typedef struct
{
    float kP;
    float kI;
    float kD;

    float imax;
    float integrator;
    float lastError;
    float lastDerivative;
    float dFilter;         //RC
    
    float target;
    float error;
    
    float maxout;       
    
} PID_t;

float PID_GetP(PID_t* pid, float error);
float PID_GetI(PID_t* pid, float error, float dt);
void  PID_ResetI(PID_t* pid);
float PID_GetD(PID_t* pid, float error, float dt);
float PID_GetPI(PID_t* pid, float error, float dt);
float PID_GetPID(PID_t* pid, float error, float dt);
void  PID_SetParam(PID_t* pid, float p, float i, float d, float imaxval, float dCutFreq) ;


#endif  





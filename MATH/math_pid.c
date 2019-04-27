#include <math_pid.h>
#include <math_tool.h>

/**********************************************************************************************************
*�� �� ��: PID_GetP
*����˵��: ����������
*��    ��: PID�ṹ�� �������
*�� �� ֵ: �����������
**********************************************************************************************************/
float PID_GetP(PID_t* pid, float error)
{
    return (float)error * pid->kP;
}

/**********************************************************************************************************
*�� �� ��: PID_GetI
*����˵��: ���ֿ�����
*��    ��: PID�ṹ�� ������� ����ʱ����
*�� �� ֵ: ������ֿ���
**********************************************************************************************************/
float PID_GetI(PID_t* pid, float error, float dt)
{
    if((pid->kI != 0) && (dt != 0))
    {
        pid->integrator += ((float)error * pid->kI) * dt;
		
        pid->integrator = ConstrainFloat(pid->integrator, -pid->imax, +pid->imax);
        return pid->integrator;
    }
    return 0;
}
/**********************************************************************************************************
*�� �� ��: PID_GetD
*����˵��: ��һ��RC��ͨ�˲���΢�ֿ�����������΢������������
*��    ��: PID�ṹ�� ������� ����ʱ����
*�� �� ֵ: ���΢�ֿ���
**********************************************************************************************************/
float PID_GetD(PID_t* pid, float error, float dt)
{
    if ((pid->kD != 0) && (dt != 0))
    {
        float derivative;

        derivative = (error - pid->lastError) / dt;

        derivative = pid->lastDerivative + (dt / ( pid->dFilter + dt)) * (derivative - pid->lastDerivative);

        pid->lastError	= error;
        pid->lastDerivative = derivative;

        return pid->kD * derivative;
    }
    return 0;
}

float PID_GetDD(PID_t* pid,float error_delta)
{
	return pid->kD * error_delta;
}
/**********************************************************************************************************
*�� �� ��: PID_GetPID
*����˵��: ����+����+΢�ֿ�����
*��    ��: PID�ṹ�� ������� ����ʱ����
*�� �� ֵ: ���PID����
**********************************************************************************************************/
float PID_GetPID(PID_t* pid, float error, float dt)
{
	if(abs(error)<(pid->lineval))
		return PID_GetP(pid, error) + PID_GetI(pid, error, dt) + PID_GetD(pid, error, dt);
	else
		return PID_GetP(pid, error)+ PID_GetD(pid, error, dt);
}


float PID_GetPIDD(PID_t* pid, float error,float error_delta,float dt)
{
	if(abs(error)<(pid->lineval))
		return PID_GetP(pid, error) + PID_GetI(pid, error, dt) + PID_GetDD(pid,error_delta);
	else
		return PID_GetP(pid, error)+ PID_GetDD(pid,error_delta);
}


/**********************************************************************************************************
*�� �� ��: PID_SetParam
*����˵��: PID��������
*��    ��: PID�ṹ�� �������� ���ֲ��� ΢�ֲ��� ���ַ�����ֵ �������� ΢�����ͨ��ֹƵ��
*�� �� ֵ: ��
**********************************************************************************************************/
void PID_SetParam(PID_t* pid, float p, float i, float d, float ilineval,float imaxval,float dCutFreq)
{
    pid->kP = p;
    pid->kI = i;
    pid->kD = d;
	pid->lineval=ilineval;
    pid->imax = imaxval;
	
    if(dCutFreq != 0)
        pid->dFilter = 1 / (2 * M_PI * dCutFreq);
    else
        pid->dFilter = 0;
}




void PID_ResetParam(PID_t* pid)
{
	pid->kP=0;
	pid->kI=0;
	pid->kD=0;
	pid->integrator=0;
	pid->dFilter=0;
	pid->error=0;
	pid->imax=0;
	pid->lastDerivative=0;
	pid->lastError=0;
	pid->lineval=0;
	pid->maxout=0;
	pid->target=0;
}

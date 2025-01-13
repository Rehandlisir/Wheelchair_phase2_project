/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/R9CODE/R9_V2_2ºÅ»ú×îÐÂ/R9_V2/R9_407F_num_2/Drivers/BSP/PID/attitude_control.c
 * @Description  :  
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-09-29 09:46:11
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
 * ref :https://blog.csdn.net/qq_30267617/article/details/113541033
**/
#include <stdbool.h>
#include "pid.h"

void abs_outlimit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

void pidInit(PidObject* pid, const float desired, const pidInit_t pidParam, const float dt)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
	pid->desired = desired;
	pid->kp = pidParam.kp;
	pid->ki = pidParam.ki;
	pid->kd = pidParam.kd;
	pid->iLimit    = 1.0;//DEFAULT_PID_INTEGRATION_LIMIT;
	pid->iLimitLow = 1.0;//-DEFAULT_PID_INTEGRATION_LIMIT;
	pid->dt        = dt;
}

float pidUpdate(PidObject* pid, const float error)
{
	float output;

	pid->error = error;   
	pid->integ += pid->error * pid->dt;
	pid->deriv = (pid->error - pid->prevError) / pid->dt;

	pid->outP = pid->kp * pid->error;
	pid->outI = pid->ki * pid->integ;
	pid->outD = pid->kd * pid->deriv;

	abs_outlimit(&(pid->integ), pid->iLimit);
	output = pid->outP + pid->outI + pid->outD;
	abs_outlimit(&(output), pid->maxOutput);
	pid->prevError = pid->error;

	return output;
}

void pidSetIntegralLimit(PidObject* pid, const float limit) 
{
    pid->iLimit = limit;
}

void pidSetIntegralLimitLow(PidObject* pid, const float limitLow) 
{
    pid->iLimitLow = limitLow;
}

void pidSetOutLimit(PidObject* pid, const float maxoutput) 
{
    pid->maxOutput = maxoutput;
}

void pidReset(PidObject* pid)
{
	pid->error     = 0;
	pid->prevError = 0;
	pid->integ     = 0;
	pid->deriv     = 0;
}

void pidSetError(PidObject* pid, const float error)
{
	pid->error = error;
}

void pidSetDesired(PidObject* pid, const float desired)
{
	pid->desired = desired;
}

float pidGetDesired(PidObject* pid)
{
	return pid->desired;
}

bool pidIsActive(PidObject* pid)
{
	bool isActive = true;

	if (pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f)
	{
		isActive = false;
	}

	return isActive;
}

void pidSetKp(PidObject* pid, const float kp)
{
	pid->kp = kp;
}

void pidSetKi(PidObject* pid, const float ki)
{
	pid->ki = ki;
}

void pidSetKd(PidObject* pid, const float kd)
{
	pid->kd = kd;
}

void pidSetPID(PidObject* pid, const float kp,const float ki,const float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}
void pidSetDt(PidObject* pid, const float dt) 
{
    pid->dt = dt;
}


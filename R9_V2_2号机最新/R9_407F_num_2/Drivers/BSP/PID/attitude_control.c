/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/R9CODE/R9_V2_2号机最新/R9_V2/R9_407F_num_2/Drivers/BSP/PID/attitude_control.c
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
#include "sensor.h"
#include "attitude_pid.h"

//pid参数
configParam_t configParamCar =
{
	.pidAngle=	/*角度PID*/
	{	
		.roll=
		{
			.kp=5.0,
			.ki=0.0,
			.kd=0.0,
		},
		.pitch=
		{
			.kp=5.0,
			.ki=0.0,
			.kd=0.0,
		},
		.yaw=
		{
			.kp=5.0,
			.ki=0.0,
			.kd=0.0,
		},
	},	
	.pidRate=	/*角速度PID*/
	{	
		.roll=
		{
			.kp=320.0,
			.ki=0.0,
			.kd=5.0,
		},
		.pitch=
		{
			.kp=320.0,
			.ki=0.0,
			.kd=5.0,
		},
		.yaw=
		{
			.kp=18.0,
			.ki=0.2,
			.kd=0.0,
		},
	},	
	.pidPos=	/*位置PID*/
	{	
		.vx=
		{
			.kp=0.0,
			.ki=0.0,
			.kd=0.0,
		},
		.vy=
		{
			.kp=0.0,
			.ki=0.0,
			.kd=0.0,
		},
		.vz=
		{
			.kp=21.0,
			.ki=0.0,
			.kd=60.0,
		},
	},
	
};


PidObject pidAngleRoll;
PidObject pidAnglePitch;
PidObject pidAngleYaw;
PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;
PidObject pidDepth;

static inline int16_t pidOutLimit(float in)
{
	if (in > INT16_MAX)
		return INT16_MAX;
	else if (in < -INT16_MAX)
		return -INT16_MAX;
	else
		return (int16_t)in;
}


void attitudeControlInit()
{

	//pidInit(&pidAngleRoll, 0, configParamCar.pidAngle.roll, ATTITUDE_UPDATE_DT);   /*roll  角度PID初始化*/
	//pidInit(&pidAnglePitch, 0, configParamCar.pidAngle.pitch, ATTITUDE_UPDATE_DT); /*pitch 角度PID初始化*/
	pidInit(&pidAngleYaw, 0, configParamCar.pidAngle.yaw, ATTITUDE_UPDATE_DT);	   /*yaw   角度PID初始化*/
	//pidSetIntegralLimit(&pidAngleRoll, PID_ANGLE_ROLL_INTEGRATION_LIMIT);		   /*roll  角度积分限幅设置*/
	//pidSetIntegralLimit(&pidAnglePitch, PID_ANGLE_PITCH_INTEGRATION_LIMIT);		   /*pitch 角度积分限幅设置*/
	pidSetIntegralLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);			   /*yaw   角度积分限幅设置*/
	pidSetOutLimit(&pidAngleYaw, PID_ANGLE_YAW_INTEGRATION_LIMIT);

	//pidInit(&pidRateRoll, 0, configParamCar.pidRate.roll, ATTITUDE_UPDATE_DT);	 /*roll  角速度PID初始化*/
	//pidInit(&pidRatePitch, 0, configParamCar.pidRate.pitch, ATTITUDE_UPDATE_DT); /*pitch 角速度PID初始化*/
	pidInit(&pidRateYaw, 0, configParamCar.pidRate.yaw, ATTITUDE_UPDATE_DT);	 /*yaw   角速度PID初始化*/
	//pidSetIntegralLimit(&pidRateRoll, PID_RATE_ROLL_INTEGRATION_LIMIT);			 /*roll  角速度积分限幅设置*/
	//pidSetIntegralLimit(&pidRatePitch, PID_RATE_PITCH_INTEGRATION_LIMIT);		 /*pitch 角速度积分限幅设置*/
	pidSetIntegralLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);			 /*yaw   角速度积分限幅设置*/
	pidSetOutLimit(&pidRateYaw, PID_RATE_YAW_INTEGRATION_LIMIT);
}

void attitudeRatePID(attitude_t *actualRate, attitude_t *desiredRate, attitude_t *output) /* 角速度环PID */
{
	//output->roll = pidOutLimit(pidUpdate(&pidRateRoll, desiredRate->roll - actualRate->roll));
	//output->pitch = pidOutLimit(pidUpdate(&pidRatePitch, desiredRate->pitch - actualRate->pitch));
	output->yaw = pidOutLimit(pidUpdate(&pidRateYaw, desiredRate->yaw - actualRate->yaw));
}

void attitudeAnglePID(attitude_t *actualAngle, attitude_t *desiredAngle, attitude_t *outDesiredRate) /* 角度环PID */
{
	//outDesiredRate->roll = pidUpdate(&pidAngleRoll, desiredAngle->roll - actualAngle->roll);
	//outDesiredRate->pitch = pidUpdate(&pidAnglePitch, desiredAngle->pitch - actualAngle->pitch);

	float yawError = desiredAngle->yaw - actualAngle->yaw;
	if (yawError > 180.0f)
		yawError -= 360.0f;
	else if (yawError < -180.0)
		yawError += 360.0f;
	outDesiredRate->yaw = pidUpdate(&pidAngleYaw, yawError);
}

void attitudeResetAllPID(void) /*复位PID*/
{
	pidReset(&pidAngleRoll);
	pidReset(&pidAnglePitch);
	pidReset(&pidAngleYaw);
	pidReset(&pidRateRoll);
	pidReset(&pidRatePitch);
	pidReset(&pidRateYaw);
}


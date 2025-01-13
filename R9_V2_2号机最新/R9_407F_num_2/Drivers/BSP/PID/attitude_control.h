/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/R9CODE/R9_V2_2号机最新/R9_V2/R9_407F_num_2/Drivers/BSP/PID/attitude_control.h
 * @Description  :  
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-09-27 15:50:03
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
#ifndef __ATTITUDE_PID_H
#define __ATTITUDE_PID_H
#include <stdbool.h>
#include "pid.h"

#define ATTITUDE_UPDATE_RATE 	500  //更新频率100hz
#define ATTITUDE_UPDATE_DT 		(1.0f / ATTITUDE_UPDATE_RATE)

typedef struct 
{
	float x;
	float y;
	float z;
} Axis3f;

//姿态集
typedef struct
{
	float roll;
	float pitch;
	float yaw;
} attitude_t;

extern PidObject pidAngleRoll;
extern PidObject pidAnglePitch;
extern PidObject pidAngleYaw;
extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;
extern PidObject pidDepth;
extern configParam_t configParamCar;

void attitudeControlInit(void);
bool attitudeControlTest(void);

void attitudeRatePID(attitude_t *actualRate, attitude_t *desiredRate,attitude_t *output);	/* 角速度环PID */
void attitudeAnglePID(attitude_t *actualAngle,attitude_t *desiredAngle,attitude_t *outDesiredRate);	/* 角度环PID */
void attitudeResetAllPID(void);		/*复位PID*/
void attitudePIDwriteToConfigParam(void);

#endif /* __ATTITUDE_PID_H */

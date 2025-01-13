#ifndef __PID_H
#define __PID_H
#include "./SYSTEM/sys/sys.h"
#include "stdbool.h"

typedef struct
{
	float kp;
	float ki;
	float kd;
} pidInit_t;

typedef struct
{
	pidInit_t roll;
	pidInit_t pitch;
	pidInit_t yaw;
} pidParam_t;

typedef struct
{
	pidInit_t vx;
	pidInit_t vy;
	pidInit_t vz;
} pidParamPos_t;

typedef struct
{
	pidParam_t pidAngle;  /*�Ƕ�PID*/
	pidParam_t pidRate;	  /*���ٶ�PID*/
	pidParamPos_t pidPos; /*λ��PID*/
	float thrustBase;		  /*���Ż���ֵ*/
	uint8_t cksum;
} configParam_t;

typedef struct
{
	float desired;	 //< set point
	float error;	 //< error
	float prevError; //< previous error
	float integ;	 //< integral
	float deriv;	 //< derivative
	float kp;		 //< proportional gain
	float ki;		 //< integral gain
	float kd;		 //< derivative gain
	float outP;		 //< proportional output (debugging)
	float outI;		 //< integral output (debugging)
	float outD;		 //< derivative output (debugging)
	float iLimit;	 //< integral limit
	float iLimitLow; //< integral limit
	float maxOutput;
	float dt; //< delta-time dt
} PidObject;

/*pid�ṹ���ʼ��*/
void pidInit(PidObject *pid, const float desired, const pidInit_t pidParam, const float dt);
void pidParaInit(PidObject *pid, float maxOutput, float iLimit, const pidInit_t pidParam);
void pidSetIntegralLimit(PidObject *pid, const float limit); /*pid�����޷�����*/
void pidSetOutLimit(PidObject *pid, const float maxoutput);	 /*pid����޷�����*/
void pidSetDesired(PidObject *pid, const float desired);	 /*pid��������ֵ*/
float pidUpdate(PidObject *pid, const float error);			 /*pid����*/
float pidGetDesired(PidObject *pid);						 /*pid��ȡ����ֵ*/
bool pidIsActive(PidObject *pid);							 /*pid״̬*/
void pidReset(PidObject *pid);								 /*pid�ṹ�帴λ*/
void pidSetError(PidObject *pid, const float error);		 /*pidƫ������*/
void pidSetKp(PidObject *pid, const float kp);				 /*pid Kp����*/
void pidSetKi(PidObject *pid, const float ki);				 /*pid Ki����*/
void pidSetKd(PidObject *pid, const float kd);				 /*pid Kd����*/
void pidSetPID(PidObject *pid, const float kp, const float ki, const float kd);
void pidSetDt(PidObject *pid, const float dt); /*pid dt����*/

#endif /* __PID_H */

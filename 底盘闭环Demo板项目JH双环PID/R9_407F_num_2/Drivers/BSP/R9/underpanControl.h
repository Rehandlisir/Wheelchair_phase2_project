#ifndef __UNDERPANCONTROL_H
#define __UNDERPANCONTROL_H

#include "./SYSTEM/sys/sys.h"
#include "math.h"
#include "stdio.h"
#include "./BSP/R9/brake.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/Common/common.h"
#include "./BSP/R9/Slavemodbus.h"
#include "./BSP/R9/getadcdata.h"
#include "./BSP/R9/moterdriver.h"
#include "./BSP/R9/mpu6050.h"
#include "./BSP/R9/inv_mpu.h"
#include "./BSP/R9/inv_mpu_dmp_motion_driver.h" 
#include "./BSP/R9/mlx90393.h"

/* 捷和电机参数 */
#define underpan_H 0.55 /**/

#define pi 3.1415926 /*PI*/
#define KMPH_TO_RPM   15  /*1km/h 约15RPM*/
#define JH_BEMF   0.137  /*电机输出轴反电动势 0.1025V/RPM */

/*  MAXON电机参数 */
#define Maxon_BEMF  0.1025    /*电机输出轴反电动势 0.1025V/RPM */

/*车体运行状态结构体*/
typedef enum
{
	idle = 0,
	forward,
	backward,
	front_left,
	front_right,
	back_left,
	back_right,
	turnself_left,
	turnself_right
} RunState;
/*速度规划输入参数结构体*/
typedef struct /**/
{
	int16_t adcx;
	int16_t adcy;
	float set_Maximum_Strspeed;
	float k_backpra ;  /*倒车最大速度比率*/
	float k_forwardpra ;  /*前进最大速度比率*/
	float K_revise;        /*转速修正系数*/
	float Steering_Angle;  /*转向角设定默认 15.0 范围哦 0~89度 值越大转弯半径越小*/
	float K_tran2RPM  ;   /*即 RPM/KM/H*/
} VELOCITY_PIn;
extern VELOCITY_PIn  Struc_ActuPra_Int;
/*速度规划输出参数结构体*/
typedef struct /*??????????*/
{
	float underpanVelocity;	  
	float presentation_velocity; 		  
	float acceleration_coeff;	  
	float steering_angle;	/*实时转向角*/	  
	float L_Velocity;	/*规划的目标线速速度*/		  
	float R_Velocity;	/*规划的目标线速速度*/	
	float LN_Velocity;	/*规划的电机目标RPM*/		  
	float RN_Velocity;	/*规划的电机目标RPM*/			  				 	   
	RunState runstate; 
} VELOCITY_POUT;
extern VELOCITY_POUT Struc_ActuPra_Out;

/* 电机参数结构体 */
typedef struct 
{
  uint8_t state;          /*电机状态*/
  float current;          /*电机电流*/
  float volatage;         /*电机电压*/
  float power;            /*电机功率*/
  float speed;            /*电机实际速度*/
  uint8_t brake_state;    /*电机抱闸器动作状态 0:松开 1：锁住*/
  uint16_t init_current_adc_val; /*电流初始ADC值大小 */
  float pwm;
  float pidspeed_out;
  float theory_pwm;
  float battary_v;
  float test_data;
} Motor_TypeDef;

typedef struct 
{
  float r9_battary_v;
  float r9_10v;
  float r9_15v;
} R9SYSTEM_TypeDef;

extern Motor_TypeDef gl_motor_data;  /*电机参数变量*/
extern Motor_TypeDef gr_motor_data;  /*电机参数变量*/
extern R9SYSTEM_TypeDef g_r9sys_data;
extern uint16_t brakeflage;
void velocity_maping(VELOCITY_PIn velPlanIn);
void brake_excute(void);
void VelocityLevelSet(void);
void underpanExcute(void);
void MPU6050Excute(void);

#endif

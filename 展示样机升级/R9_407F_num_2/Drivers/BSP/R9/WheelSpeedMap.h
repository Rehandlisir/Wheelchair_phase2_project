#ifndef __WHEELSPEEDMAP_H
#define __WHEELSPEEDMAP_H

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

/* ???? */
#define underpan_H 0.55 /*???*/

#define pi 3.1415926 /*????PI*/

#define GEAR_RATIO 29.5		 /* 减速比 */
#define Diameter 0.354		 /* 轮子直径354mm  */
#define MoterMaxrN 175.0	 /*输出轴额定转速 175PRM */
#define KMPH_TO_RPM    15.0  /*1km/h 约 15RPM*/
#define VelocityConst  7.3   /*电机速率常数 单位 RPM/V*/
#define KMPH_TO_Voltage 2.055 /*V/KMPH*/
#define KMPH_TO_Duty   0.087  /*1km/h 占空比约 8.56%*/

/*   */

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
	/*逆运动学规划变量*/
	float set_Max_Forward;
	float set_Min_Forward;
	float set_Max_Reverse;
	float set_Min_Reverse;
	float set_Max_Turn;
	float set_Min_Turn;
	uint8_t set_maxvelocitylevel;
	uint8_t set_minvelocitylevel;
	float setTurnAtMaxspeed;
	float setMaxspeedInTurn;
	float set_forwardAct;
	float set_reverseAct;
	float set_turnAct;
	uint8_t set_velocitylevelAct;
} VELOCITY_PIn;


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


/* 电机参数结构体 */
typedef struct 
{
  uint8_t state;          /*电机状态*/
  float current;          /*电机电流*/
  float volatage;         /*电机电压*/
  float power;            /*电机功率*/
  float speed;            /*电机实际速度*/
  float pwm;
  float volatage_Ra;   /*内阻分压*/
} Motor_TypeDef;

typedef struct 
{
  float r9_battary_v;
  float r9_10v;
  float r9_15v;
  uint8_t r9pid_start;
  uint8_t r9pid_stop;

} R9SYSTEM_TypeDef;

extern VELOCITY_PIn  Struc_ActuPra_Int;

extern VELOCITY_POUT Struc_ActuPra_Out; 

extern Motor_TypeDef gl_motor_data;  /*电机参数变量*/
extern Motor_TypeDef gr_motor_data;  /*电机参数变量*/
extern R9SYSTEM_TypeDef g_r9sys_data;

void joystic_data_handle(void);
void velocity_maping(VELOCITY_PIn velPlanIn);
void brake_excute(void);
void VelocityLevelSet(void);
void moter_run(void);
void mapingExcute(void);
int8_t sign(float x);
#endif

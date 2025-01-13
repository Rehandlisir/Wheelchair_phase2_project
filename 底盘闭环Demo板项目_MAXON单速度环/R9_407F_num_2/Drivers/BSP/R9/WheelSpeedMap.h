#ifndef __WHEELSPEEDMAPING_H
#define __WHEELSPEEDMAPING_H

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
#include "./BSP/Curve_planing/curve.h"
/* �ݺ͵������ */
#define underpan_H 0.55 /**/

#define PI 3.1415926 /*PI*/
#define KMPH_TO_RPM   15  /*1km/h Լ15RPM*/
#define JH_BEMF   0.137  /*�������ᷴ�綯�� 0.1025V/RPM */

/*  MAXON������� */
#define Maxon_BEMF  0.1025    /*�������ᷴ�綯�� 0.1025V/RPM */

#define MAX_XDATA 3500.0
#define MIN_XDATA 3500.0
#define MAX_YDATA 3500.0
#define MIN_YDATA 3500.0
#define YADC_DIM_MAX 250.0  
#define YADC_DIM_MIN -250.0  
#define XADC_DIM_MAX 250.0  
#define XADC_DIM_MIN -250.0  

/*��������״̬�ṹ��*/
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
/*�ٶȹ滮��������ṹ��*/
typedef struct /**/
{
	int16_t adcx;
	int16_t adcy;
	float set_Maximum_Strspeed;
	float k_backpra ;  /*��������ٶȱ���*/
	float k_forwardpra ;  /*ǰ������ٶȱ���*/
	float K_revise;        /*ת������ϵ��*/
	float Steering_Angle;  /*ת����趨Ĭ�� 15.0 ��ΧŶ 0~89�� ֵԽ��ת��뾶ԽС*/
	float K_tran2RPM  ;   /*�� RPM/KM/H*/
	/*���˶�ѧ�滮����*/
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
extern VELOCITY_PIn  Struc_ActuPra_Int;
/*�ٶȹ滮��������ṹ��*/
typedef struct /*??????????*/
{
	float underpanVelocity;	  
	float presentation_velocity; 		  
	float acceleration_coeff;	  
	float steering_angle;	/*ʵʱת���*/	  
	float L_Velocity;	/*�滮��Ŀ�������ٶ�*/		  
	float R_Velocity;	/*�滮��Ŀ�������ٶ�*/	
	float LN_Velocity;	/*�滮�ĵ��Ŀ��RPM*/		  
	float RN_Velocity;	/*�滮�ĵ��Ŀ��RPM*/			  				 	   
	RunState runstate; 
	/*���˶�ѧ�滮����*/
	// float forward_velocity;
	// float turn_velocity;
	// float reverse_velocity;
} VELOCITY_POUT;
extern VELOCITY_POUT Struc_ActuPra_Out;

/* ��������ṹ�� */
typedef struct 
{
  uint8_t state;          /*���״̬*/
  float current;          /*�������*/
  float volatage;         /*�����ѹ*/
  float power;            /*�������*/
  float speed;            /*���ʵ���ٶ�*/
  uint8_t brake_state;    /*�����բ������״̬ 0:�ɿ� 1����ס*/
  uint16_t init_current_adc_val; /*������ʼADCֵ��С */
  float pwm;
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

extern Motor_TypeDef gl_motor_data;  /*�����������*/
extern Motor_TypeDef gr_motor_data;  /*�����������*/
extern R9SYSTEM_TypeDef g_r9sys_data;
extern uint16_t brakeflage;
extern CurveObjectType lcurve; //�����������
void velocity_maping(VELOCITY_PIn velPlanIn);
void brake_excute(void);
void VelocityLevelSet(void);
void underpanExcute(void);
// void MPU6050Excute(void);
/*�������˶�ѧ���*/
void Reverse_Kinemaping(VELOCITY_PIn velPlanIn);
void Move_parameter_set(void);
#endif

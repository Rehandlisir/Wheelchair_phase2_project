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

typedef struct /**/
{
	int16_t adcx;
	int16_t adcy;
	double set_Maximum_Strspeed;
} VELOCITY_PIn;
extern VELOCITY_PIn  Struc_ActuPra_Int;
typedef struct /*??????????*/
{
	double xy_module; 	   // x、y 坐标系下的 摇杆AD 矢量模长
	double presentation_velocity; // 显示速度	    
	double steering_angle;	//转向角	  
	double L_Velocity;	 	// 左轮线速		  
	double R_Velocity;		// 右轮线速		
	double L_rpm;	 	// 左轮线速		  
	double R_rpm;		// 右轮线速		  		
	double L_Dutycycle;		// 左轮占空比			  
	double R_Dutycycle;		// 右轮占空比		 		 	   
	RunState runstate; 
} VELOCITY_POUT;


extern VELOCITY_POUT Struc_ActuPra_Out; 
void velocity_maping(VELOCITY_PIn velPlanIn);
void brake_excute(void);
void VelocityLevelSet(void);
void underpanExcute(void);
void MPU6050Excute(void);

#endif

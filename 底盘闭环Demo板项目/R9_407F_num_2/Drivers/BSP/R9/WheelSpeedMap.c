/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/第二阶段新项目/底盘闭环Demo板项目/R9_407F_num_2/Drivers/BSP/R9/WheelSpeedMap.c
 * @Description  :  
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2024-12-09 20:33:04
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/
#include "./BSP/R9/underpanControl.h"
#include "./BSP/R9/moterdriver.h"
#include "./BSP/LEG_ KINEMATICS/LegRestKinematics.h"
#include "./BSP/PID/pid.h"
//本地 or 远程数据实参
VELOCITY_POUT Struc_ActuPra_Out;
VELOCITY_PIn  Struc_ActuPra_Int;

Motor_TypeDef gl_motor_data;  /*电机参数变量*/
Motor_TypeDef gr_motor_data;  /*电机参数变量*/
R9SYSTEM_TypeDef g_r9sys_data;


uint16_t brakeflage;
RunState drivestate;
RunState e_lastdrivestate;
static float pitch,roll,yaw;
// #define JOYSTIC_AI 
#define JOYSTIC_DI 
#define REMOTE_DI

/**
 * @description: 本地摇杆数据映射左右轮速方法
 * @param  set_Maximum_Strspeed ：最大直行速度  
 * @return {*}
**/
void velocity_maping(VELOCITY_PIn velPlanIn)
{
	/*转向角计算*/
    Struc_ActuPra_Out.steering_angle = atan2((double)velPlanIn.adcy, (double)velPlanIn.adcx);
    if (velPlanIn.adcy < 0)
	{
		Struc_ActuPra_Out.steering_angle = Struc_ActuPra_Out.steering_angle +2*pi;
	}
  /*速度模长*/
    Struc_ActuPra_Out.underpanVelocity = sqrt(pow(velPlanIn.adcx,2.0)+pow(velPlanIn.adcy,2.0));
	
	/*速度模长约束*/
	
	Struc_ActuPra_Out.underpanVelocity = Value_limitf(0,Struc_ActuPra_Out.underpanVelocity,MAX_YDATA);
	if ((Struc_ActuPra_Out.steering_angle>=0.0 && Struc_ActuPra_Out.steering_angle<(pi-Struc_ActuPra_Int.Steering_Angle/180.0 *pi))||
	(Struc_ActuPra_Out.steering_angle>=(2.0*pi-Struc_ActuPra_Int.Steering_Angle/180.0 *pi) && Struc_ActuPra_Out.steering_angle<=2*pi))
	{
		Struc_ActuPra_Out.L_Velocity = velPlanIn.k_forwardpra *velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA  * \
		(sin(Struc_ActuPra_Out.steering_angle-(pi/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *pi)) + \
		cos(Struc_ActuPra_Out.steering_angle-(pi/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *pi))) /Struc_ActuPra_Int.K_revise ;
	}/*左轮前进速度规划*/
	else
	{
		Struc_ActuPra_Out.L_Velocity = velPlanIn.k_backpra *velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA  * \
		(sin(Struc_ActuPra_Out.steering_angle-(pi/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *pi)) + \
		cos(Struc_ActuPra_Out.steering_angle-(pi/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *pi))) /Struc_ActuPra_Int.K_revise ;
	}/*左轮后退进速度规划*/

    if((Struc_ActuPra_Out.steering_angle>=0 && Struc_ActuPra_Out.steering_angle<=Struc_ActuPra_Int.Steering_Angle/180.0 *pi)|| \
	(Struc_ActuPra_Out.steering_angle>=pi+Struc_ActuPra_Int.Steering_Angle/180.0 *pi && Struc_ActuPra_Out.steering_angle<=2.0 *pi) )
	{
		Struc_ActuPra_Out.R_Velocity = velPlanIn.k_backpra*velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA * \
		(sin(Struc_ActuPra_Out.steering_angle+(pi/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *pi)) - \
		cos(Struc_ActuPra_Out.steering_angle+(pi/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *pi))) /Struc_ActuPra_Int.K_revise ;
	}/*右轮后退速度规划*/
	else
	{
		Struc_ActuPra_Out.R_Velocity = velPlanIn.k_forwardpra*velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA * \
		(sin(Struc_ActuPra_Out.steering_angle+(pi/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *pi)) - \
		cos(Struc_ActuPra_Out.steering_angle+(pi/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *pi))) /Struc_ActuPra_Int.K_revise;
	}/*右轮前进速度规划*/
 
	Struc_ActuPra_Out.presentation_velocity = (fabs(Struc_ActuPra_Out.L_Velocity) + fabs(Struc_ActuPra_Out.R_Velocity))/2.0;
	Struc_ActuPra_Out.presentation_velocity = Value_limitf (0,Struc_ActuPra_Out.presentation_velocity ,velPlanIn.set_Maximum_Strspeed);
	// /*KM/h —— RPM—— Voltage - Duty cycle*/
	/*左右目标轮线速度 转换为 电机目标转速*/
	Struc_ActuPra_Out.LN_Velocity = Struc_ActuPra_Out.L_Velocity * Struc_ActuPra_Int.K_tran2RPM;
	Struc_ActuPra_Out.RN_Velocity = Struc_ActuPra_Out.R_Velocity * Struc_ActuPra_Int.K_tran2RPM;
	/*电机目标转速滤波处理*/
	Struc_ActuPra_Out.LN_Velocity = filterValue_float(&filter_LN,Struc_ActuPra_Out.LN_Velocity);
	Struc_ActuPra_Out.RN_Velocity= filterValue_float(&filter_RN,Struc_ActuPra_Out.RN_Velocity);
    /********************左右轮目标速度PID 给定***********************************************/
	gl_speed_pid.SetPoint = Struc_ActuPra_Out.LN_Velocity;
	gr_speed_pid.SetPoint = Struc_ActuPra_Out.RN_Velocity;

    // gl_motor_data.pwm  =(Struc_ActuPra_Out.L_Velocity *0.083);
    // gr_motor_data.pwm = (Struc_ActuPra_Out.R_Velocity *0.083);
	// /*算术平均滤波占空比滤波处理*/
    // gl_motor_data.pwm = filterValue_float(&filter_Lpwm,gl_motor_data.pwm);
	// gr_motor_data.pwm = filterValue_float(&filter_Rpwm,gr_motor_data.pwm);
	// /* 占空比约束*/
	// gl_motor_data.pwm = Value_limitf(-0.90, gl_motor_data.pwm, 0.90);
	// gr_motor_data.pwm = Value_limitf(-0.90, gr_motor_data.pwm, 0.90);	
 
	LeftMoterMove(gl_motor_data.pwm,0);
	RightMoterMove(gr_motor_data.pwm ,1);
	

}

/**
 * @brief         : Brake Control Excute
 * @param        ： 摇杆归位后延时100 ms 抱住抱闸器
 * @return        {*} 
**/
void brake_excute(void)
{
/*松开抱闸*/
	if (Struc_ActuPra_Int.adcx < -200 || Struc_ActuPra_Int.adcx > 200 || Struc_ActuPra_Int.adcy > 200 || Struc_ActuPra_Int.adcy < -200)
	{
			brake(0);	
			brakeflage = 0;	
			gl_motor_data.brake_state = 0;
			gr_motor_data.brake_state = 0;
	}	
	else
/*锁住抱闸*/
	{	
		brakeflage++;
		if (brakeflage > 25)
		{ 
			brake(1);
			gl_motor_data.brake_state = 1;
			gr_motor_data.brake_state = 1;
			brakeflage = 0;
		}
	}	
}

/**
 * @brief        : Velocity Level Set
 * @return        {*}None
**/
void VelocityLevelSet(void)
{ 
	
	Struc_ActuPra_Int.k_forwardpra = 1.0;
	Struc_ActuPra_Int.Steering_Angle = 30.0;
	Struc_ActuPra_Int.K_revise = sin(0.5*pi-(pi/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *pi))
	+cos(0.5*pi-(pi/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *pi));
	Struc_ActuPra_Int.K_tran2RPM = 15.0;
	Struc_ActuPra_Int.set_Maximum_Strspeed =2.0 ;
	if(Struc_ActuPra_Int.set_Maximum_Strspeed ==1.0 )
	{
		Struc_ActuPra_Int.k_backpra =1.0;
	}
	else
	{
		Struc_ActuPra_Int.k_backpra =0.5;

	}
}


/**
 * @description: 底盘驱动程序代码 主要用到定时器 3 和 定时器 9 控制左右电机占空比
 * @return 无
 */
void underpanExcute(void)
{
	brake_excute();
	VelocityLevelSet();
	/*摇杆数据给定-已进行了限制幅度和滤波处理*/
	Struc_ActuPra_Int.adcx =mlxdata.xdata;// local_slopelimitx(mlxdata.xdata,75,150);  
	Struc_ActuPra_Int.adcy =mlxdata.ydata;// local_slopelimity(mlxdata.ydata,75,150);  
    /*调试参数*/
	// Struc_ActuPra_Int.adcx =0.0;
	// Struc_ActuPra_Int.adcy=3500;	 
    /*摇杆数据增量约束*/
	Struc_ActuPra_Int.adcx = local_slopelimitx(Struc_ActuPra_Int.adcx,50,100);  
	Struc_ActuPra_Int.adcy = local_slopelimity(Struc_ActuPra_Int.adcy,50,100);


	velocity_maping(Struc_ActuPra_Int); /*速度规划 */
}
/**
 * @FilePath     : /R9_407F_num_2/Drivers/BSP/R9/WheelSpeedMap.c
 * @Description  :  
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-03-05 10:32:33
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/
#include "./BSP/R9/WheelSpeedMap.h"
#include "./BSP/R9/moterdriver.h"
#include "./BSP/LEG_ KINEMATICS/LegRestKinematics.h"
#include "./BSP/PID/pid.h"
#include "./BSP/Common/common.h"
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
#define JOYSTICDATA_AS5013
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
		Struc_ActuPra_Out.steering_angle = Struc_ActuPra_Out.steering_angle +2*PI;
	}
  /*速度模长*/
    Struc_ActuPra_Out.underpanVelocity = sqrt(pow(velPlanIn.adcx,2.0)+pow(velPlanIn.adcy,2.0));
	
	/*速度模长约束*/
	
	Struc_ActuPra_Out.underpanVelocity = Value_limitf(0,Struc_ActuPra_Out.underpanVelocity,MAX_YDATA);
	if ((Struc_ActuPra_Out.steering_angle>=0.0 && Struc_ActuPra_Out.steering_angle<(PI-Struc_ActuPra_Int.Steering_Angle/180.0 *PI))||
	(Struc_ActuPra_Out.steering_angle>=(2.0*PI-Struc_ActuPra_Int.Steering_Angle/180.0 *PI) && Struc_ActuPra_Out.steering_angle<=2*PI))
	{
		Struc_ActuPra_Out.L_Velocity = velPlanIn.k_forwardpra *velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA  * \
		(sin(Struc_ActuPra_Out.steering_angle-(PI/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *PI)) + \
		cos(Struc_ActuPra_Out.steering_angle-(PI/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *PI))) /Struc_ActuPra_Int.K_revise ;
	}/*左轮前进速度规划*/
	else
	{
		Struc_ActuPra_Out.L_Velocity = velPlanIn.k_backpra *velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA  * \
		(sin(Struc_ActuPra_Out.steering_angle-(PI/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *PI)) + \
		cos(Struc_ActuPra_Out.steering_angle-(PI/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *PI))) /Struc_ActuPra_Int.K_revise ;
	}/*左轮后退进速度规划*/

    if((Struc_ActuPra_Out.steering_angle>=0 && Struc_ActuPra_Out.steering_angle<=Struc_ActuPra_Int.Steering_Angle/180.0 *PI)|| \
	(Struc_ActuPra_Out.steering_angle>=PI+Struc_ActuPra_Int.Steering_Angle/180.0 *PI && Struc_ActuPra_Out.steering_angle<=2.0 *PI) )
	{
		Struc_ActuPra_Out.R_Velocity = velPlanIn.k_backpra*velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA * \
		(sin(Struc_ActuPra_Out.steering_angle+(PI/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *PI)) - \
		cos(Struc_ActuPra_Out.steering_angle+(PI/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *PI))) /Struc_ActuPra_Int.K_revise ;
	}/*右轮后退速度规划*/
	else
	{
		Struc_ActuPra_Out.R_Velocity = velPlanIn.k_forwardpra*velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA * \
		(sin(Struc_ActuPra_Out.steering_angle+(PI/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *PI)) - \
		cos(Struc_ActuPra_Out.steering_angle+(PI/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *PI))) /Struc_ActuPra_Int.K_revise;
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
	
}

void joystic_data_handle(void)
{
	
	
	#ifdef JOYSTICDATA_MLX90393
	if (sqrt((mlxdata.xdata) * (mlxdata.xdata) + (mlxdata.ydata) * (mlxdata.ydata)) < DEAD_ZONE_R)
	{
		mlxdata.xdata = 0;
		mlxdata.ydata = 0;
		g_r9sys_data.r9pid_start =0;
		g_r9sys_data.r9pid_stop =1;

	}
	else
	{
		g_r9sys_data.r9pid_start =1;
		g_r9sys_data.r9pid_stop =0;

		// 计算摇杆的距离
		double distance = sqrt((mlxdata.xdata) * (mlxdata.xdata) + (mlxdata.ydata) * (mlxdata.ydata));
		if (distance > JOYSTIC_R)
		{
			// 计算缩放因子，将点映射到圆周上
			double scale = JOYSTIC_R / distance;
			mlxdata.xdata = (int16_t)(mlxdata.xdata * scale);
			mlxdata.ydata = (int16_t)(mlxdata.ydata * scale);
		}
	}
	/*原始数据均值滤波处理*/
	// mlxdata.xdata = filterValue_int16(&filter_ADCX, mlxdata.xdata);
	// mlxdata.ydata = filterValue_int16(&filter_ADCY, mlxdata.ydata);
	// if (mlxdata.ydata > 2810)
	// {
	// 	mlxdata.ydata = 2810;
	// }
	// if (mlxdata.ydata < -2810)
	// {
	// 	mlxdata.ydata = -2810;
	// }
	// if (mlxdata.xdata > 2810)
	// {
	// 	mlxdata.xdata = 2810;
	// }
	// if (mlxdata.xdata < -2810)
	// {
	// 	mlxdata.xdata = -2810;
	// }
	// Struc_ActuPra_Int.adcx = mlxdata.xdata;
	// Struc_ActuPra_Int.adcy = mlxdata.ydata;

	Struc_ActuPra_Int.adcx = lowPassFilter(&lowpassx_ADC, mlxdata.xdata);
	Struc_ActuPra_Int.adcy = lowPassFilter(&lowpassy_ADC, mlxdata.ydata);

	// printf("%d,%d\n\t",Struc_ActuPra_Int.adcx,Struc_ActuPra_Int.adcy);
	#endif
	#ifdef JOYSTICDATA_AS5013
	double distance = sqrt((as5013_data.x_raw ) * (as5013_data.x_raw ) + (as5013_data.y_raw ) * (as5013_data.y_raw ));
	if (distance < DEAD_ZONE_R)
	{
		as5013_data.x_raw = 0;
		as5013_data.y_raw = 0;
		g_r9sys_data.r9pid_start =0;
		g_r9sys_data.r9pid_stop =1;

	}
	else
	{
		g_r9sys_data.r9pid_start =1;
		g_r9sys_data.r9pid_stop =0;
		// 计算摇杆的距离
		if (distance > JOYSTIC_R)
		{
			// 计算缩放因子，将点映射到圆周上
			double scale = JOYSTIC_R / distance;
			as5013_data.x_raw = (int8_t)(as5013_data.x_raw * scale);
			as5013_data.y_raw = (int8_t)(as5013_data.y_raw * scale);
		}
	}

	// as5013_data.x_raw= filterValue_int16(&filter_ADCX,  as5013_data.x_raw);
	// as5013_data.y_raw = filterValue_int16(&filter_ADCY, as5013_data.y_raw);
	Struc_ActuPra_Int.adcx = lowPassFilter(&lowpassx_ADC, as5013_data.x_raw);
	Struc_ActuPra_Int.adcy = lowPassFilter(&lowpassy_ADC, as5013_data.y_raw);
	Struc_ActuPra_Int.adcx = - Struc_ActuPra_Int.adcx;
	Struc_ActuPra_Int.adcy = - Struc_ActuPra_Int.adcy;
	// Struc_ActuPra_Int.adcx = filterValue_int16(&filter_ADCX, Struc_ActuPra_Int.adcx);
	// Struc_ActuPra_Int.adcy = filterValue_int16(&filter_ADCY, Struc_ActuPra_Int.adcy);
	// printf("%d,%d\n\t",Struc_ActuPra_Int.adcx,Struc_ActuPra_Int.adcy);
	#endif
}
void Reverse_Kinemaping(VELOCITY_PIn velPlanIn)
{
	float lineVelocity ,omega_velocity;
	float set_turn_handlmax,set_forwardActmax,set_reverseActmax;
	if (velPlanIn.adcy>0)
	{
		set_forwardActmax =velPlanIn.set_forwardAct/JOYSTIC_R *velPlanIn.adcy;
		lineVelocity =  set_forwardActmax*sin(velPlanIn.adcy/JOYSTIC_R*PI/2.0);
	}
	else
	{
		set_reverseActmax =velPlanIn.set_reverseAct/JOYSTIC_R *abs(velPlanIn.adcy);
		lineVelocity =  set_reverseActmax*sin(velPlanIn.adcy/JOYSTIC_R*PI/2.0);	
	}
	set_turn_handlmax =velPlanIn.set_turnAct/JOYSTIC_R *abs(velPlanIn.adcx) ;
	omega_velocity =  set_turn_handlmax*sin(velPlanIn.adcx/JOYSTIC_R*PI/2.0);
	/*带入底盘逆运动学公式解算*/
	Struc_ActuPra_Out.L_Velocity = lineVelocity + omega_velocity;
	Struc_ActuPra_Out.R_Velocity = lineVelocity -omega_velocity;
	/*左右目标轮线速度 转换为 电机目标转速*/
	Struc_ActuPra_Out.LN_Velocity = Struc_ActuPra_Out.L_Velocity * velPlanIn.K_tran2RPM;
	Struc_ActuPra_Out.RN_Velocity = Struc_ActuPra_Out.R_Velocity * velPlanIn.K_tran2RPM;
	gl_speed_pid.SetPoint = lowPassFilter(&lowpass_lspeedTarget, Struc_ActuPra_Out.LN_Velocity);
	gr_speed_pid.SetPoint = lowPassFilter(&lowpass_rspeedTarget, Struc_ActuPra_Out.RN_Velocity);
	/*目标速度约束*/
	gl_speed_pid.SetPoint = Value_limitf(-100.0,gl_speed_pid.SetPoint,100.0);
	gr_speed_pid.SetPoint = Value_limitf(-100.0,gr_speed_pid.SetPoint,100.0);

	// gl_motor_data.pwm = Struc_ActuPra_Out.L_Velocity*KMPH_TO_Duty;
	// gr_motor_data.pwm = Struc_ActuPra_Out.R_Velocity*KMPH_TO_Duty;
}

void Move_parameter_set(void)
{
	Struc_ActuPra_Int.K_tran2RPM = 15.0;
	/*挡位等级范围*/
	Struc_ActuPra_Int.set_maxvelocitylevel = 5.0;
	Struc_ActuPra_Int.set_minvelocitylevel = 1.0;
	/*各方向最值速度*/
	Struc_ActuPra_Int.set_Max_Forward = 6.0;
	Struc_ActuPra_Int.set_Min_Forward = 1.5; // 1挡位对应的最大速度
	Struc_ActuPra_Int.set_Max_Reverse =2.0;
	Struc_ActuPra_Int.set_Min_Reverse=1.0; // 1挡位对应的最大速度
	Struc_ActuPra_Int.set_Max_Turn =1.2;
	Struc_ActuPra_Int.set_Min_Turn =1.0;// 1挡位对应的最大速度	
	Struc_ActuPra_Int.setMaxspeedInTurn = 0.15;
	
	/*实际设定挡位*/
	Struc_ActuPra_Int.set_velocitylevelAct = 3.0;
	/*实际设定挡位下的最大前行速度*/
	Struc_ActuPra_Int.set_forwardAct = (Struc_ActuPra_Int.set_Max_Forward-Struc_ActuPra_Int.set_Min_Forward)/(Struc_ActuPra_Int.set_maxvelocitylevel-Struc_ActuPra_Int.set_minvelocitylevel)*
	(Struc_ActuPra_Int.set_velocitylevelAct-Struc_ActuPra_Int.set_minvelocitylevel) + Struc_ActuPra_Int.set_Min_Forward;
	/*实际设定挡位下的最大倒车速度*/
	Struc_ActuPra_Int.set_reverseAct=(Struc_ActuPra_Int.set_Max_Reverse-Struc_ActuPra_Int.set_Min_Reverse)/(Struc_ActuPra_Int.set_maxvelocitylevel-Struc_ActuPra_Int.set_minvelocitylevel)*
	(Struc_ActuPra_Int.set_velocitylevelAct-Struc_ActuPra_Int.set_minvelocitylevel) + Struc_ActuPra_Int.set_Min_Reverse;
	/*实际设定挡位下的最大转向速度*/
	Struc_ActuPra_Int.set_turnAct = (Struc_ActuPra_Int.set_Max_Turn - Struc_ActuPra_Int.set_Min_Turn)/(Struc_ActuPra_Int.set_maxvelocitylevel-Struc_ActuPra_Int.set_minvelocitylevel)*
	(Struc_ActuPra_Int.set_velocitylevelAct-Struc_ActuPra_Int.set_minvelocitylevel) + Struc_ActuPra_Int.set_Min_Turn;
	
}

void moter_run(void)
{
	car_move(gl_motor_data.pwm, gr_motor_data.pwm);
}

/**
 * @brief         : Brake Control Excute
 * @param        ： 摇杆归位后延时100 ms 抱住抱闸器
 * @return        {*} 
**/
void brake_excute(void)
{
/*松开抱闸*/
if (g_r9sys_data.r9pid_start) //摇杆在死区之外打开抱闸器
	{
		brake(0);	
		brakeflage = 0;	
		gl_motor_data.brake_state = 0;
		gr_motor_data.brake_state = 0;
	}	
	else
/*锁住抱闸*/
	{	
		// brakeflage++;
		if(fabs(gl_motor_data.speed) <=1.0 && fabs(gr_motor_data.speed)<=1.0) /*等待轮子完全停下来后 启动锁定抱闸器*/
		{ 
				pid_init();
				brake(1);
				gl_motor_data.pwm=0;
				gr_motor_data.pwm=0;	
				Struc_ActuPra_Out.LN_Velocity=0;
				Struc_ActuPra_Out.RN_Velocity=0;
				
				gl_motor_data.brake_state = 1;
				gr_motor_data.brake_state = 1;
				brakeflage =0;
				
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
	Struc_ActuPra_Int.Steering_Angle = 20.0;
	Struc_ActuPra_Int.K_revise = sin(0.5*PI-(PI/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *PI))
	+cos(0.5*PI-(PI/4.0-Struc_ActuPra_Int.Steering_Angle/180.0 *PI));
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
void car_maping(void)
{
	Move_parameter_set();
	joystic_data_handle();															
	brake_excute();
	Reverse_Kinemaping(Struc_ActuPra_Int);
}
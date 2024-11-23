/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/闭环电调Demo/R9_407F_MoterDriveDemo/Drivers/BSP/R9/WheelSpeedMap.c
 * @Description  :  
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-10-30 16:00:25
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/
#include "./BSP/R9/underpanControl.h"
#include "./BSP/R9/moterdriver.h"
#include "./BSP/PID/pid.h"
//本地 or 远程数据实参
VELOCITY_POUT Struc_ActuPra_Out;
VELOCITY_PIn  Struc_ActuPra_Int;

RunState drivestate;
RunState e_lastdrivestate;

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
    Struc_ActuPra_Out.xy_module = sqrt(pow(velPlanIn.adcx,2.0)+pow(velPlanIn.adcy,2.0));
	
	#if defined JOYSTIC_DI
		/*速度模长约束*/
		Struc_ActuPra_Out.xy_module = Value_limitf(0,Struc_ActuPra_Out.xy_module,MAX_YDATA);//(MAX_YDATA - YADC_DIM));
		// printf("Struc_ActuPra_Out.xy_module: %f\n",Struc_ActuPra_Out.xy_module);
		/*左右轮目标线速度 Km/h*/

			Struc_ActuPra_Out.L_Velocity = velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.xy_module/MAX_YDATA  * \
			(sin(Struc_ActuPra_Out.steering_angle-pi/12.0) + cos(Struc_ActuPra_Out.steering_angle-pi/12.0)) /1.30 ;

			Struc_ActuPra_Out.R_Velocity = velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.xy_module/MAX_YDATA * \
			(sin(Struc_ActuPra_Out.steering_angle+pi/12.0) - cos(Struc_ActuPra_Out.steering_angle+pi/12.0)) /1.30 ;

	#endif 
	/*模拟型摇杆*/
	#if defined JOYSTIC_AI
		/*速度模长约束*/
		Struc_ActuPra_Out.xy_module = Value_limitf(0,Struc_ActuPra_Out.xy_module,(yadc_max - yadc_Dim));
		Struc_ActuPra_Out.xy_module = Value_limitf(0,Struc_ActuPra_Out.xy_module,(yadc_max - yadc_Dim));
		// printf("Struc_ActuPra_Out.xy_module: %f\n",Struc_ActuPra_Out.xy_module);
		/*左右轮目标线速度 Km/h*/
		Struc_ActuPra_Out.L_Velocity = velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.xy_module/ (yadc_max - yadc_Dim) * \
		(sin(Struc_ActuPra_Out.steering_angle-pi/6.0) + cos(Struc_ActuPra_Out.steering_angle-pi/6.0)) / 1.0 ;

		Struc_ActuPra_Out.R_Velocity = velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.xy_module/ (yadc_max - yadc_Dim) * \
		(sin(Struc_ActuPra_Out.steering_angle+pi/6.0) - cos(Struc_ActuPra_Out.steering_angle+pi/6.0)) / 1.0 ;
	#endif	
	Struc_ActuPra_Out.presentation_velocity = (fabs(Struc_ActuPra_Out.L_Velocity) + fabs(Struc_ActuPra_Out.R_Velocity))/1.8;
	Struc_ActuPra_Out.presentation_velocity = Value_limitf (0,Struc_ActuPra_Out.presentation_velocity ,velPlanIn.set_Maximum_Strspeed);

	/*KM/h —— RPM—— Voltage - Duty cycle*/
	/*左右目标轮线速度 转换为 占空比根据实际情况进行修正，常数为修正系数*/
	// Struc_ActuPra_Out.L_Dutycycle = fabs(Struc_ActuPra_Out.L_Velocity) * KMPH_TO_Duty;
	// Struc_ActuPra_Out.R_Dutycycle = fabs(Struc_ActuPra_Out.R_Velocity) * KMPH_TO_Duty;
	/*算术平均滤波占空比滤波处理*/
    // Struc_ActuPra_Out.L_Velocity = filterValue_float(&filter_L,Struc_ActuPra_Out.L_Velocity);
	// Struc_ActuPra_Out.R_Velocity = filterValue_float(&filter_R,Struc_ActuPra_Out.R_Velocity);
	/* 占空比约束*/
	// Struc_ActuPra_Out.L_Dutycycle = slopelimitLDuty(Struc_ActuPra_Out.L_Dutycycle,0.1,0.1);
	// Struc_ActuPra_Out.R_Dutycycle = slopelimitRDuty(Struc_ActuPra_Out.R_Dutycycle,0.1,0.1);
	
	// Struc_ActuPra_Out.L_Dutycycle = Value_limitf(0.0, Struc_ActuPra_Out.L_Dutycycle, 0.99);
	// Struc_ActuPra_Out.R_Dutycycle = Value_limitf(0.0, Struc_ActuPra_Out.R_Dutycycle, 0.99);	

	/*闭环调节 目标线速度需要转换为电机输出轴 目标转速 1km/h 约 15rpm/min*/
	Struc_ActuPra_Out.L_rpm =fabs(Struc_ActuPra_Out.L_Velocity) * 15.0;

	Struc_ActuPra_Out.R_rpm =fabs(Struc_ActuPra_Out.R_Velocity) * 15.0;
	// Struc_ActuPra_Out.L_Dutycycle = fabs(Struc_ActuPra_Out.L_Velocity) * KMPH_TO_Duty;
	// Struc_ActuPra_Out.L_Dutycycle = Value_limitf(0.0, Struc_ActuPra_Out.L_Dutycycle, 0.99);
	// // if (g_speed_pid.detect_flage) 
	// // {
	// 	if(Struc_ActuPra_Out.runstate!=idle)
	// 	{
	// 		if (g_speed_pid.detect_flage)
	// 		g_speed_pid.SetPoint = Struc_ActuPra_Out.L_rpm;
	// 		g_speed_pid.SetPoint = Value_limitf(0.0, g_speed_pid.SetPoint, 175); // 输入约束
	// 		/* PID计算，输出占空比 */
	// 		Struc_ActuPra_Out.L_Dutycycle = increment_pid_ctrl(&g_speed_pid, fabs(g_motor_data.speed));
	// 		Struc_ActuPra_Out.L_Dutycycle = Value_limitf(0.0, Struc_ActuPra_Out.L_Dutycycle, 0.999); // PID输出约束
	// 	}
	// 	else
	// 	{
	// 		g_speed_pid.SetPoint =0;
	// 		Struc_ActuPra_Out.L_Dutycycle = 0;
	// 	}

	// }
  

	/* 静止  */
	if (velPlanIn.adcx == 0 && velPlanIn.adcy  == 0)
	{
		Struc_ActuPra_Out.runstate = idle;
		drivestate = idle;
                                                                                                                                                                                  
	}
	/*前直行 */
	if (velPlanIn.adcx == 0 && velPlanIn.adcy > 0)
	{

		Struc_ActuPra_Out.runstate = forward;
		drivestate = forward;
	}
	/*向后直行 */
	if (velPlanIn.adcx == 0 && velPlanIn.adcy < 0)
	{
		Struc_ActuPra_Out.runstate = backward;
		drivestate = backward;
	}
	
	/*向左前转向 */
//	if (velPlanIn.adcx < 0 && velPlanIn.adcy > 0)
	if (Struc_ActuPra_Out.steering_angle > pi/2 && Struc_ActuPra_Out.steering_angle <(5/6.0)*pi && velPlanIn.adcx!=0 )
	{
		Struc_ActuPra_Out.runstate = front_left;
		drivestate = front_left;
	}
	/*向右前转向 */
	if (Struc_ActuPra_Out.steering_angle > pi*1.0/6.0 && Struc_ActuPra_Out.steering_angle <1/2.0 *pi && velPlanIn.adcx!=0)
	{
		Struc_ActuPra_Out.runstate = front_right;
		drivestate = front_right;
	}
	/*向左后转向 */
	if (Struc_ActuPra_Out.steering_angle > 1.5*pi  && Struc_ActuPra_Out.steering_angle <11/6.0 *pi && velPlanIn.adcx!=0)
	{
		Struc_ActuPra_Out.runstate = back_left;
		drivestate = back_left;
	}
	/*向右后转向 */
	if (Struc_ActuPra_Out.steering_angle > 7/6.0 *pi  && Struc_ActuPra_Out.steering_angle <1.5 *pi && velPlanIn.adcx!=0 )
	{
		Struc_ActuPra_Out.runstate = back_right;
		drivestate = back_right;
	}
	
	/*原地右转 */
	if ((Struc_ActuPra_Out.steering_angle >=0 &&  velPlanIn.adcx>0 && Struc_ActuPra_Out.steering_angle < 1/6.0 *pi) \
	|| (Struc_ActuPra_Out.steering_angle >11/6.0*pi  && Struc_ActuPra_Out.steering_angle <2 *pi))	
	{
		Struc_ActuPra_Out.runstate = turnself_right;
		drivestate = turnself_right;
	}

	/*原地左转 */
	if (Struc_ActuPra_Out.steering_angle >5/6.0*pi && Struc_ActuPra_Out.steering_angle < 7/6.0 *pi)	
	{
		Struc_ActuPra_Out.runstate = turnself_left;
		drivestate = turnself_left;
	}
	// printf("L:%lf,R:%lf,ADCX:%d,ADCY:%d\r\n",Struc_ActuPra_Out.L_Dutycycle,Struc_ActuPra_Out.R_Dutycycle,velPlanIn.adcx,velPlanIn.adcy);	
	switch (drivestate)
	{
		case idle: 
			// dcmotor_stop();
			dcmotor_stop();
			Struc_ActuPra_Out.L_Dutycycle=0;
			break;
		case forward:
			motor_pwm_set(0,Struc_ActuPra_Out.L_Dutycycle);
			//RightMoterMove(1,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case front_right:
			motor_pwm_set(0,Struc_ActuPra_Out.L_Dutycycle);
			//RightMoterMove(1,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case front_left:
			motor_pwm_set(0,Struc_ActuPra_Out.L_Dutycycle);
			//RightMoterMove(1,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case backward : /*backward velocity is half of set_Maximum_Strspeed*/
			motor_pwm_set(1,Struc_ActuPra_Out.L_Dutycycle);
			//RightMoterMove(0,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case back_left:
			motor_pwm_set(1,Struc_ActuPra_Out.L_Dutycycle);
			//RightMoterMove(0,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case back_right:
			motor_pwm_set(1,Struc_ActuPra_Out.L_Dutycycle);
			//RightMoterMove(0,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case turnself_right:
			motor_pwm_set(0,Struc_ActuPra_Out.L_Dutycycle);
			//RightMoterMove(0,Struc_ActuPra_Out.R_Dutycycle);		
			break;
		case turnself_left:
			motor_pwm_set(1,Struc_ActuPra_Out.L_Dutycycle);
			//RightMoterMove(1,Struc_ActuPra_Out.R_Dutycycle);		
			break;
		default:
			break;
	}
}

/**
 * @brief         : Brake Control Excute
 * @param        ： 摇杆归位后延时100 ms 抱住抱闸器
 * @return        {*} 
**/


/**
 * @brief        : Velocity Level Set
 * @return        {*}None
**/
void VelocityLevelSet(void)
{ 

	Struc_ActuPra_Int.set_Maximum_Strspeed = 10.0 ; 
}

/**
 * @description: 底盘驱动程序代码 主要用到定时器 3 和 定时器 9 控制左右电机占空比
 * @return 无
 */
void underpanExcute(void)
{
		VelocityLevelSet();
		#if defined JOYSTIC_DI	
		Struc_ActuPra_Int.adcx =mlxdata.xdata;// local_slopelimitx(mlxdata.xdata,75,150);  
		Struc_ActuPra_Int.adcy =mlxdata.ydata;// local_slopelimity(mlxdata.ydata,75,150);  	 
		#endif

    /*摇杆数据增量约束*/
		Struc_ActuPra_Int.adcx = local_slopelimitx(Struc_ActuPra_Int.adcx,50,150);  
		Struc_ActuPra_Int.adcy = local_slopelimity(Struc_ActuPra_Int.adcy,50,150);

        // printf("Struc_ActuPra_Int.adcx:%d,Struc_ActuPra_Int.adcy:%d\n",Struc_ActuPra_Int.adcx,Struc_ActuPra_Int.adcy);
		velocity_maping(Struc_ActuPra_Int); /*速度规划 */
}



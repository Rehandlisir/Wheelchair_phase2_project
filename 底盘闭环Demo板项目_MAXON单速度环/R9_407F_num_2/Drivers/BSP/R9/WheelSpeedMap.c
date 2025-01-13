/**
 * @FilePath     : /R9_407F_num_2/Drivers/BSP/R9/WheelSpeedMap.c
 * @Description  :  
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-01-13 10:44:58
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
	car_move(gl_motor_data.pwm,gr_motor_data.pwm,1,0,0);
}


void Reverse_Kinemaping(VELOCITY_PIn velPlanIn)
{
  static float y_port,x_port,x_portlimit;
  static float limitKp;
  if (velPlanIn.adcy>0)
  {
	y_port = velPlanIn.set_forwardAct*sin(velPlanIn.adcy/MAX_YDATA*PI/2.0);
	
  }
  else
  {
	y_port = velPlanIn.set_reverseAct*sin(velPlanIn.adcy/MIN_YDATA*PI/2.0);	
  }
  x_port =  velPlanIn.set_turnAct*sin(velPlanIn.adcx/MAX_XDATA*PI/2.0);
  
//   // x_port  转向约束
//   	if(Struc_ActuPra_Int.set_velocitylevelAct ==1)
// 	{
// 		limitKp = 0.19;
// 	}
// 	else if(Struc_ActuPra_Int.set_velocitylevelAct ==2)
// 	{
// 		limitKp = 0.38;
// 	}
// 	else if(Struc_ActuPra_Int.set_velocitylevelAct ==3)
// 	{
// 		limitKp = 0.57;
// 	}
// 	else if(Struc_ActuPra_Int.set_velocitylevelAct ==4)
// 	{
// 		limitKp = 0.76;
// 	}
// 	else if(Struc_ActuPra_Int.set_velocitylevelAct ==5)
// 	{
// 		limitKp = 0.85;
// 	}
	// x_port =lowPassFilter(&lowpassx_port,x_port);
	// y_port = lowPassFilter(&lowpassy_port,y_port);

  	// x_port = Value_limitf(limitKp*velPlanIn.set_turnAct/velPlanIn.set_forwardAct*y_port -velPlanIn.set_turnAct,x_port,-limitKp*velPlanIn.set_turnAct/velPlanIn.set_forwardAct*y_port +velPlanIn.set_turnAct);
 	// y_port = Value_limitf(limitKp*velPlanIn.set_forwardAct/velPlanIn.set_turnAct *x_port-velPlanIn.set_forwardAct,y_port,-limitKp*velPlanIn.set_forwardAct/velPlanIn.set_turnAct *x_port+velPlanIn.set_forwardAct);
 	

 
  /*寻找摇杆稳定性的两个点对应的*/
  /*逆运动学映射计算*/
	Struc_ActuPra_Out.L_Velocity = y_port + x_port;
    Struc_ActuPra_Out.R_Velocity = y_port - x_port;
 	/*左右目标轮线速度 转换为 电机目标转速*/
	Struc_ActuPra_Out.LN_Velocity = Struc_ActuPra_Out.L_Velocity*Struc_ActuPra_Int.K_tran2RPM;
	Struc_ActuPra_Out.RN_Velocity = Struc_ActuPra_Out.R_Velocity*Struc_ActuPra_Int.K_tran2RPM;
	/*电机目标转速滤波处理*/
	Struc_ActuPra_Out.LN_Velocity = filterValue_float(&filter_LN,Struc_ActuPra_Out.LN_Velocity);
	Struc_ActuPra_Out.RN_Velocity= filterValue_float(&filter_RN,Struc_ActuPra_Out.RN_Velocity);
	gl_speed_pid.SetPoint = Struc_ActuPra_Out.LN_Velocity;
	gr_speed_pid.SetPoint = Struc_ActuPra_Out.RN_Velocity;
	car_move(gl_motor_data.pwm,gr_motor_data.pwm,1,0,0);
	// printf("%f,%f\n\t",y_port,x_port);
	
}

void Move_parameter_set(void)
{
	Struc_ActuPra_Int.K_tran2RPM = 15.0;
	/*挡位等级范围*/
	Struc_ActuPra_Int.set_maxvelocitylevel = 5.0;
	Struc_ActuPra_Int.set_minvelocitylevel = 1.0;
	/*各方向最值速度*/
	Struc_ActuPra_Int.set_Max_Forward = 6.0;
	Struc_ActuPra_Int.set_Min_Forward = 1.0; // 1挡位对应的最大速度
	Struc_ActuPra_Int.set_Max_Reverse =2.0;
	Struc_ActuPra_Int.set_Min_Reverse=1.0; // 1挡位对应的最大速度
	Struc_ActuPra_Int.set_Max_Turn =1.5;
	Struc_ActuPra_Int.set_Min_Turn =1.0;// 1挡位对应的最大速度	
	Struc_ActuPra_Int.setMaxspeedInTurn = 0.15;
	
	/*实际设定挡位*/
	Struc_ActuPra_Int.set_velocitylevelAct = 4.0;
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

/**
 * @brief         : Brake Control Excute
 * @param        ： 摇杆归位后延时100 ms 抱住抱闸器
 * @return        {*} 
**/
void brake_excute(void)
{
/*松开抱闸*/
	if (Struc_ActuPra_Int.adcx!=0 ||Struc_ActuPra_Int.adcy!=0) //(Struc_ActuPra_Int.adcx < -300 || Struc_ActuPra_Int.adcx > 300 || Struc_ActuPra_Int.adcy > 300 || Struc_ActuPra_Int.adcy < -300)
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
		if ((fabs(gl_motor_data.speed)<5.0 && fabs(gr_motor_data.speed<5.0))|| brakeflage>250 ) 
		{ 
				pid_init();
				brake(1);
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
void underpanExcute(void)
{
	static float limitKp;
	
//  VelocityLevelSet();
	Move_parameter_set();
	/*将数据约束在一个正方形中*/
	mlxdata.xdata = Value_limit(-MIN_XDATA, mlxdata.xdata, MAX_XDATA);
	mlxdata.ydata = Value_limit(-MIN_YDATA, mlxdata.ydata, MAX_YDATA); 
	/*死区区域划分*/
	if(mlxdata.xdata>=-350&&mlxdata.xdata<=350&&mlxdata.ydata>=-350&&mlxdata.ydata<=350)
	{
		mlxdata.xdata =0;
		mlxdata.ydata =0;
	}
	/*摇杆数据限幅度处理*/
	Struc_ActuPra_Int.adcx =mlxdata.xdata;
	Struc_ActuPra_Int.adcy =mlxdata.ydata;
	
	if(Struc_ActuPra_Int.set_velocitylevelAct ==1)
	{
		limitKp = 0.19;
	}
	else if(Struc_ActuPra_Int.set_velocitylevelAct ==2)
	{
		limitKp = 0.38;
	}
	else if(Struc_ActuPra_Int.set_velocitylevelAct ==3)
	{
		limitKp = 0.57;
	}
	else if(Struc_ActuPra_Int.set_velocitylevelAct ==4)
	{
		limitKp = 0.76;
	}
	else if(Struc_ActuPra_Int.set_velocitylevelAct ==5)
	{
		limitKp = 0.85;
	}
	// Struc_ActuPra_Int.adcx = local_slopelimitx(Struc_ActuPra_Int.adcx,25,25);  
	// Struc_ActuPra_Int.adcy = local_slopelimity(Struc_ActuPra_Int.adcy,25,25);

	Struc_ActuPra_Int.adcx = Value_limit(limitKp*Struc_ActuPra_Int.adcy-3500,Struc_ActuPra_Int.adcx,-limitKp*Struc_ActuPra_Int.adcy+3500);
	Struc_ActuPra_Int.adcy = Value_limit(limitKp*Struc_ActuPra_Int.adcx-3500,Struc_ActuPra_Int.adcy,-limitKp*Struc_ActuPra_Int.adcx+3500);
	// Struc_ActuPra_Int.adcx  =lowPassFilter(&lowpassx_port,Struc_ActuPra_Int.adcx);
	Struc_ActuPra_Int.adcy = lowPassFilter(&lowpassy_port,Struc_ActuPra_Int.adcy);
	brake_excute();
//	velocity_maping(Struc_ActuPra_Int); /*速度规划 */
	Reverse_Kinemaping(Struc_ActuPra_Int);
	// printf("%f,%f,%d,%d\n\t",Struc_ActuPra_Out.LN_Velocity,Struc_ActuPra_Out.RN_Velocity,Struc_ActuPra_Int.adcx,Struc_ActuPra_Int.adcy);
}
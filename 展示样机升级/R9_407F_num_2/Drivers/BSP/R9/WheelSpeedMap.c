/**
 * @FilePath     : /展示样机升级/R9_407F_num_2/Drivers/BSP/R9/WheelSpeedMap.c
 * @Description  :  
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-03-04 11:43:34
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/
#include "./BSP/R9/WheelSpeedMap.h"
#include "./BSP/R9/moterdriver.h"
#include "./BSP/LEG_ KINEMATICS/LegRestKinematics.h"
#include "./BSP/PID/pid.h"
#include "./BSP/CAN/can.h"
#include "./BSP/ADC/adc.h"
//本地 or 远程数据实参
VELOCITY_POUT Struc_ActuPra_Out;
VELOCITY_PIn  Struc_ActuPra_Int;
Motor_TypeDef gl_motor_data; /*电机参数变量*/
Motor_TypeDef gr_motor_data; /*电机参数变量*/
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
 * @brief        : 摇杆数据预处理
 * @return        {*}无
 **/
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

	as5013_data.x_raw= filterValue_int16(&filter_ADCX,  as5013_data.x_raw);
	as5013_data.y_raw = filterValue_int16(&filter_ADCY, as5013_data.y_raw);
	Struc_ActuPra_Int.adcx = lowPassFilter(&lowpassx_ADC, as5013_data.x_raw);
	Struc_ActuPra_Int.adcy = lowPassFilter(&lowpassy_ADC, as5013_data.y_raw);
	Struc_ActuPra_Int.adcx = -Struc_ActuPra_Int.adcx;
	Struc_ActuPra_Int.adcy = -Struc_ActuPra_Int.adcy;
	// Struc_ActuPra_Int.adcx = filterValue_int16(&filter_ADCX, Struc_ActuPra_Int.adcx);
	// Struc_ActuPra_Int.adcy = filterValue_int16(&filter_ADCY, Struc_ActuPra_Int.adcy);
	// printf("%d,%d\n\t",Struc_ActuPra_Int.adcx ,Struc_ActuPra_Int.adcy);
	#endif
	
}
/**
 * @description: 本地摇杆数据映射左右轮速方法
 * @param  set_Maximum_Strspeed ：最大直行速度  
 * @return {*}
**/
void velocity_maping(VELOCITY_PIn velPlanIn)
{
    /*本地摇杆操控轮椅状态*/
	if (velPlanIn.adcx!=0 || velPlanIn.adcy!=0)
	{
		g_slaveReg[22] = 1;
	}
	else
	{
		g_slaveReg[22] = 0;
	}
	
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
	set_turn_handlmax =0.7*velPlanIn.set_turnAct/JOYSTIC_R *abs(velPlanIn.adcx) ;
	omega_velocity =  set_turn_handlmax*sin(velPlanIn.adcx/JOYSTIC_R*PI/2.0);
	/*带入底盘逆运动学公式解算*/
	Struc_ActuPra_Out.L_Velocity = lineVelocity + omega_velocity;
	Struc_ActuPra_Out.R_Velocity = lineVelocity -omega_velocity;
	/*测试代码*/
	// gl_motor_data.pwm = (Struc_ActuPra_Out.L_Velocity) * KMPH_TO_Duty;
	// gr_motor_data.pwm = (Struc_ActuPra_Out.R_Velocity) * KMPH_TO_Duty;
	
	/*左右目标轮线速度 转换为 电机目标转速*/
	Struc_ActuPra_Out.LN_Velocity = Struc_ActuPra_Out.L_Velocity * velPlanIn.K_tran2RPM;
	Struc_ActuPra_Out.RN_Velocity = Struc_ActuPra_Out.R_Velocity * velPlanIn.K_tran2RPM;
	
	// gl_speed_pid.SetPoint = lowPassFilter(&lowpass_lspeedTarget, Struc_ActuPra_Out.LN_Velocity);//Struc_ActuPra_Out.LN_Velocity;
	// gr_speed_pid.SetPoint = lowPassFilter(&lowpass_rspeedTarget, Struc_ActuPra_Out.RN_Velocity);//Struc_ActuPra_Out.RN_Velocity;
	// /*速度显示*/
	// Struc_ActuPra_Out.presentation_velocity = (fabs(Struc_ActuPra_Out.L_Velocity) + fabs(Struc_ActuPra_Out.R_Velocity))/2.0;
	// Struc_ActuPra_Out.presentation_velocity = Value_limitf (0,Struc_ActuPra_Out.presentation_velocity ,velPlanIn.set_Maximum_Strspeed);
	// g_slaveReg[3] = (uint16_t)(Struc_ActuPra_Out.presentation_velocity * 100); // RK3588 接受车速信息KM/H
	// if (g_slaveReg[2] == 1) //充电时速度清0
	// {
	// 	g_slaveReg[3] =0;	
	// }
	moter_run();
}

/**
 * @brief        : 电机运行
 * @return        {*}无
 **/
void moter_run(void)
{
	
	car_move(gl_motor_data.pwm, gr_motor_data.pwm);//, 1, 0, 0);
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
		BRAKE(1);
		brakeflage = 0;
	}
	else /* 如果摇杆落在死区之内 启动锁抱闸的逻辑*/
	{	
		brakeflage++;
    	if(brakeflage>100)//((fabs(gl_motor_data.speed) <=1.0 &&fabs(gr_motor_data.speed)<=1.0)||brakeflage>100) /*等待轮子完全停下来后 启动锁定抱闸器*/
		{
			Struc_ActuPra_Out.LN_Velocity=0;
			Struc_ActuPra_Out.RN_Velocity=0;
			gl_motor_data.pwm=0;
			gr_motor_data.pwm=0;
			BRAKE(0);
			brakeflage=0;
		}
	}
}

/**
 * @brief        : Velocity Level Set
 * @return        {*}None
**/
void VelocityLevelSet(void)
{ 
		Struc_ActuPra_Int.K_tran2RPM =15.0;// 15.0;
		/*挡位等级范围*/
		Struc_ActuPra_Int.set_maxvelocitylevel = 5.0;
		Struc_ActuPra_Int.set_minvelocitylevel = 1.0;
  switch (g_slaveReg[64])
	{
		case 1: /*normal mode*/
					Struc_ActuPra_Int.set_Max_Forward = 6.0;
					Struc_ActuPra_Int.set_Min_Forward = 1.0; // 1挡位对应的最大速度
					Struc_ActuPra_Int.set_Max_Reverse = 2.0;
					Struc_ActuPra_Int.set_Min_Reverse = 1.0; // 1挡位对应的最大速度
					Struc_ActuPra_Int.set_Max_Turn = 1.5;
					Struc_ActuPra_Int.set_Min_Turn = 1.0; // 1挡位对应的最大速度
					Struc_ActuPra_Int.setMaxspeedInTurn = 0.15;
					Struc_ActuPra_Int.setTurnAtMaxspeed = 0.15;
					break;
		case 2 :/*indoor mode*/
					Struc_ActuPra_Int.set_Max_Forward = 6.0;
					Struc_ActuPra_Int.set_Min_Forward = 1.0; // 1挡位对应的最大速度
					Struc_ActuPra_Int.set_Max_Reverse = 2.0;
					Struc_ActuPra_Int.set_Min_Reverse = 1.0; // 1挡位对应的最大速度
					Struc_ActuPra_Int.set_Max_Turn = 1.5;
					Struc_ActuPra_Int.set_Min_Turn = 1.0; // 1挡位对应的最大速度
					Struc_ActuPra_Int.setMaxspeedInTurn = 0.15;
					Struc_ActuPra_Int.setTurnAtMaxspeed = 0.15;	
					break;		
		case 3 :/*outdoor mode*/
					Struc_ActuPra_Int.set_Max_Forward = 6.0;
					Struc_ActuPra_Int.set_Min_Forward = 1.0; // 1挡位对应的最大速度
					Struc_ActuPra_Int.set_Max_Reverse = 2.0;
					Struc_ActuPra_Int.set_Min_Reverse = 1.0; // 1挡位对应的最大速度
					Struc_ActuPra_Int.set_Max_Turn = 1.5;
					Struc_ActuPra_Int.set_Min_Turn = 1.0; // 1挡位对应的最大速度
					Struc_ActuPra_Int.setMaxspeedInTurn = 0.15;
					Struc_ActuPra_Int.setTurnAtMaxspeed = 0.15;	
					break;
		default:
					Struc_ActuPra_Int.set_Max_Forward = 5.0;
					Struc_ActuPra_Int.set_Min_Forward = 1.5; // 1挡位对应的最大速度
					Struc_ActuPra_Int.set_Max_Reverse = 2.0;
					Struc_ActuPra_Int.set_Min_Reverse = 1.0; // 1挡位对应的最大速度
					Struc_ActuPra_Int.set_Max_Turn = 2.0;
					Struc_ActuPra_Int.set_Min_Turn = 1.5; // 1挡位对应的最大速度
					Struc_ActuPra_Int.setMaxspeedInTurn = 0.15;
					Struc_ActuPra_Int.setTurnAtMaxspeed = 0.15;	
			break;
	}
	  /*实际设定挡位*/
		Struc_ActuPra_Int.set_velocitylevelAct = 4.0;//g_slaveReg[73];
		// 远程连接 或者通讯失败 默认2档位
		if(g_slaveReg[78]||comheartstate.com_state == Fail)
		{
			Struc_ActuPra_Int.set_velocitylevelAct = 4.0; 
		}
			/*低速限制*/
		if ((chairkinematicspra.y_o>50.8 && chairkinematicspra.y_o<152.4) ||
		adcdata.backboard_pos<1300 ||
		(fabs(chairkinematicspra.theta_x2X)>6.0 && fabs(chairkinematicspra.theta_x2X)<10.0) ||
		adcdata.legangle_pos>1800)
		 {
			Struc_ActuPra_Int.set_velocitylevelAct = 4.0; 	
		}
		/*超低速限制*/
		if (chairkinematicspra.y_o>152.4 ||
		fabs(chairkinematicspra.theta_x2X)>10.0||
		(adcdata.legangle_pos>1800 && fabs(chairkinematicspra.theta_x2X)>6.0 && fabs(chairkinematicspra.theta_x2X)<10.0)||
		(chairkinematicspra.y_o>50.8 && fabs(chairkinematicspra.theta_x2X)>6.0 && fabs(chairkinematicspra.theta_x2X)<10.0)
		)
		{
			Struc_ActuPra_Int.set_velocitylevelAct = 4.0; 
		}	
		/*实际设定挡位下的最大前行速度*/
		Struc_ActuPra_Int.set_forwardAct = (Struc_ActuPra_Int.set_Max_Forward - Struc_ActuPra_Int.set_Min_Forward) / (Struc_ActuPra_Int.set_maxvelocitylevel - Struc_ActuPra_Int.set_minvelocitylevel) *
												 (Struc_ActuPra_Int.set_velocitylevelAct - Struc_ActuPra_Int.set_minvelocitylevel) +
											 Struc_ActuPra_Int.set_Min_Forward;
		/*实际设定挡位下的最大倒车速度*/
		Struc_ActuPra_Int.set_reverseAct = (Struc_ActuPra_Int.set_Max_Reverse - Struc_ActuPra_Int.set_Min_Reverse) / (Struc_ActuPra_Int.set_maxvelocitylevel - Struc_ActuPra_Int.set_minvelocitylevel) *
												 (Struc_ActuPra_Int.set_velocitylevelAct - Struc_ActuPra_Int.set_minvelocitylevel) +
											 Struc_ActuPra_Int.set_Min_Reverse;
		/*实际设定挡位下的最大转向速度*/
		Struc_ActuPra_Int.set_turnAct = (Struc_ActuPra_Int.set_Max_Turn - Struc_ActuPra_Int.set_Min_Turn) / (Struc_ActuPra_Int.set_maxvelocitylevel - Struc_ActuPra_Int.set_minvelocitylevel) *
											(Struc_ActuPra_Int.set_velocitylevelAct - Struc_ActuPra_Int.set_minvelocitylevel) +
										Struc_ActuPra_Int.set_Min_Turn;
}

/**
 * @description: 底盘驱动程序代码 主要用到定时器 3 和 定时器 9 控制左右电机占空比
 * @return 无
 */
void mapingExcute(void)
{
	#ifdef OLD_
	static uint8_t LocalOpflage=0,RemoteOpfalge=0;
	/*本地操作 可操控条件同时满足 ：(1)摇杆有数据  （2）未在充电 （3）未操作座椅*/
	if((mlxdata.xdata || mlxdata.ydata) &&g_slaveReg[2]==0 && linerun_cmd==iddle_cmd) 
	{
		LocalOpflage =1;
		RemoteOpfalge =0;
		// 本地打断远程操作的同时需要将远程摇杆数据清除
		g_slaveReg[79] = 0;
		g_slaveReg[80]  =0;

	}	
	else // 从本地退出
	{
		LocalOpflage =0;	
	}
	/*远程可操控条件同时满足:(1)本地未操作 (2)蓝牙处于配对状态 g_slaveReg[78] (3)上下位机通讯OK (4)未在充电 （5）远程端有摇杆数据  （6）未操作座椅*/
	if (LocalOpflage ==0 &&g_slaveReg[78] && comheartstate.com_state && g_slaveReg[2]==0&&(g_slaveReg[79] || g_slaveReg[80])&& linerun_cmd==iddle_cmd)
	{
		RemoteOpfalge =1;
	}
	else // 从远程退出
	{
		RemoteOpfalge =0;
	}
    /*速度设定*/
	VelocityLevelSet();
	
	/*摇杆数据输入*/
	if (LocalOpflage)
	{
		joystic_data_handle();	 
	}
	else if(LocalOpflage == 0 && RemoteOpfalge)
	{
		Struc_ActuPra_Int.adcx = g_slaveReg[79];//remote_slopelimitx(g_slaveReg[79],35,100);  
		Struc_ActuPra_Int.adcy = g_slaveReg[80];//remote_slopelimity(g_slaveReg[80],35,100); 
	}
	else
	{
		Struc_ActuPra_Int.adcx = 0;
		Struc_ActuPra_Int.adcy = 0;
	}

	/*摇杆数据上传*/
	g_slaveReg[10] =  Struc_ActuPra_Int.adcx;
	g_slaveReg[11] =  Struc_ActuPra_Int.adcy;
	
	/*抱闸控制*/
	if (g_slaveReg[2]==0 && linerun_cmd==iddle_cmd) 
	{
		brake_excute();
	}
	else /*充电过程及姿态调整中不可运行抱闸程序*/
	{
		BRAKE(0);
	}
	velocity_maping(Struc_ActuPra_Int); /*速度规划 */
	#else
	VelocityLevelSet();
	joystic_data_handle();
	brake_excute();
	velocity_maping(Struc_ActuPra_Int); /*速度规划 */
	#endif
}
int8_t sign(float x)
{
	if (x>0)
	{
		return 1;
	}
	else if (x<0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}


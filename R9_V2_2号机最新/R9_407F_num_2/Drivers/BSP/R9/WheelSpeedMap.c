/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/R9CODE/R9_V2_2号机最新/R9_V2/R9_407F_num_2/Drivers/BSP/R9/WheelSpeedMap.c
 * @Description  :  
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-09-27 13:43:24
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/
#include "./BSP/R9/underpanControl.h"
#include "./BSP/R9/moterdriver.h"
#include "./BSP/LEG_ KINEMATICS/LegRestKinematics.h"
//本地 or 远程数据实参
VELOCITY_POUT Struc_ActuPra_Out;
VELOCITY_PIn  Struc_ActuPra_Int;

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
    /*本地摇杆操控轮椅状态*/
	if (velPlanIn.adcx!=0 || velPlanIn.adcy!=0)
	{
		g_slaveReg[22] = 1;
	}
	else
	{
		g_slaveReg[22] = 0;
	}
	/*转向角计算*/
    Struc_ActuPra_Out.steering_angle = atan2((double)velPlanIn.adcy, (double)velPlanIn.adcx);
    if (velPlanIn.adcy < 0)
	{
		Struc_ActuPra_Out.steering_angle = Struc_ActuPra_Out.steering_angle +2*pi;
	}
  /*速度模长*/
    Struc_ActuPra_Out.underpanVelocity = sqrt(pow(velPlanIn.adcx,2.0)+pow(velPlanIn.adcy,2.0));
	
	#if defined JOYSTIC_DI
		/*速度模长约束*/
		Struc_ActuPra_Out.underpanVelocity = Value_limitf(0,Struc_ActuPra_Out.underpanVelocity,MAX_YDATA);//(MAX_YDATA - YADC_DIM));
		// printf("Struc_ActuPra_Out.underpanVelocity: %f\n",Struc_ActuPra_Out.underpanVelocity);
		/*左右轮目标线速度 Km/h*/
		if (g_slaveReg[73]==1)
		{
			Struc_ActuPra_Out.R_Velocity = velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA  * \
			(sin(Struc_ActuPra_Out.steering_angle-pi/12.0) + cos(Struc_ActuPra_Out.steering_angle-pi/12.0+10.0)) /1.35 ;

			Struc_ActuPra_Out.L_Velocity = velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA * \
			(sin(Struc_ActuPra_Out.steering_angle+pi/12.0) - cos(Struc_ActuPra_Out.steering_angle+pi/12.0-10.0)) /1.35 ;
		}
		else
		{
			Struc_ActuPra_Out.L_Velocity = velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA  * \
			(sin(Struc_ActuPra_Out.steering_angle-pi/12.0) + cos(Struc_ActuPra_Out.steering_angle-pi/12.0)) /1.30 ;

			Struc_ActuPra_Out.R_Velocity = velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/MAX_YDATA * \
			(sin(Struc_ActuPra_Out.steering_angle+pi/12.0) - cos(Struc_ActuPra_Out.steering_angle+pi/12.0)) /1.30 ;
		}

	#endif 
	/*模拟型摇杆*/
	#if defined JOYSTIC_AI
		/*速度模长约束*/
		Struc_ActuPra_Out.underpanVelocity = Value_limitf(0,Struc_ActuPra_Out.underpanVelocity,(yadc_max - yadc_Dim));
		Struc_ActuPra_Out.underpanVelocity = Value_limitf(0,Struc_ActuPra_Out.underpanVelocity,(yadc_max - yadc_Dim));
		// printf("Struc_ActuPra_Out.underpanVelocity: %f\n",Struc_ActuPra_Out.underpanVelocity);
		/*左右轮目标线速度 Km/h*/
		Struc_ActuPra_Out.L_Velocity = velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/ (yadc_max - yadc_Dim) * \
		(sin(Struc_ActuPra_Out.steering_angle-pi/6.0) + cos(Struc_ActuPra_Out.steering_angle-pi/6.0)) / 1.0 ;

		Struc_ActuPra_Out.R_Velocity = velPlanIn.set_Maximum_Strspeed * Struc_ActuPra_Out.underpanVelocity/ (yadc_max - yadc_Dim) * \
		(sin(Struc_ActuPra_Out.steering_angle+pi/6.0) - cos(Struc_ActuPra_Out.steering_angle+pi/6.0)) / 1.0 ;
	#endif	
	Struc_ActuPra_Out.presentation_velocity = (fabs(Struc_ActuPra_Out.L_Velocity) + fabs(Struc_ActuPra_Out.R_Velocity))/1.8;
	Struc_ActuPra_Out.presentation_velocity = Value_limitf (0,Struc_ActuPra_Out.presentation_velocity ,velPlanIn.set_Maximum_Strspeed);
	g_slaveReg[3] = (uint16_t)(Struc_ActuPra_Out.presentation_velocity * 100); // RK3588 接受车速信息KM/H
	if (g_slaveReg[2] == 1) //充电时速度清0
	{
		g_slaveReg[3] =0;	
	}
	/*KM/h —— RPM—— Voltage - Duty cycle*/
	/*左右目标轮线速度 转换为 占空比根据实际情况进行修正，常数为修正系数*/
	Struc_ActuPra_Out.L_Dutycycle = fabs(Struc_ActuPra_Out.L_Velocity) * KMPH_TO_Duty;
	Struc_ActuPra_Out.R_Dutycycle = fabs(Struc_ActuPra_Out.R_Velocity) * KMPH_TO_Duty;
	/*算术平均滤波占空比滤波处理*/
    Struc_ActuPra_Out.L_Dutycycle = filterValue_float(&filter_L,Struc_ActuPra_Out.L_Dutycycle);
	Struc_ActuPra_Out.R_Dutycycle = filterValue_float(&filter_R,Struc_ActuPra_Out.R_Dutycycle);
	/* 占空比约束*/
	Struc_ActuPra_Out.L_Dutycycle = slopelimitLDuty(Struc_ActuPra_Out.L_Dutycycle,0.1,0.1);
	Struc_ActuPra_Out.R_Dutycycle = slopelimitRDuty(Struc_ActuPra_Out.R_Dutycycle,0.1,0.1);
	
	Struc_ActuPra_Out.L_Dutycycle = Value_limitf(0.0, Struc_ActuPra_Out.L_Dutycycle, 0.99);
	Struc_ActuPra_Out.R_Dutycycle = Value_limitf(0.0, Struc_ActuPra_Out.R_Dutycycle, 0.99);	

	/* 静止  */
	if (velPlanIn.adcx == 0 && velPlanIn.adcy  == 0)
	{
		Struc_ActuPra_Out.runstate = idle;
		drivestate = idle;
		g_slaveReg[5] = 1 ;

	}
	/*前直行 */
	if (velPlanIn.adcx == 0 && velPlanIn.adcy > 0)
	{

		Struc_ActuPra_Out.runstate = forward;
		drivestate = forward;
		g_slaveReg[5] = 2 ;
	}
	/*向后直行 */
	if (velPlanIn.adcx == 0 && velPlanIn.adcy < 0)
	{
		Struc_ActuPra_Out.runstate = backward;
		drivestate = backward;
		g_slaveReg[5] = 3 ;
	}
	
	/*向左前转向 */
//	if (velPlanIn.adcx < 0 && velPlanIn.adcy > 0)
	if (Struc_ActuPra_Out.steering_angle > pi/2 && Struc_ActuPra_Out.steering_angle <(5/6.0)*pi && velPlanIn.adcx!=0 )
	{
		Struc_ActuPra_Out.runstate = front_left;
		drivestate = front_left;
		g_slaveReg[5] = 6;
	}
	/*向右前转向 */
	if (Struc_ActuPra_Out.steering_angle > pi*1.0/6.0 && Struc_ActuPra_Out.steering_angle <1/2.0 *pi && velPlanIn.adcx!=0)
	{
		Struc_ActuPra_Out.runstate = front_right;
		drivestate = front_right;
		g_slaveReg[5] = 7;
	}
	/*向左后转向 */
	if (Struc_ActuPra_Out.steering_angle > 1.5*pi  && Struc_ActuPra_Out.steering_angle <11/6.0 *pi && velPlanIn.adcx!=0)
	{
		Struc_ActuPra_Out.runstate = back_left;
		drivestate = back_left;
		g_slaveReg[5] = 8;
	}
	/*向右后转向 */
	if (Struc_ActuPra_Out.steering_angle > 7/6.0 *pi  && Struc_ActuPra_Out.steering_angle <1.5 *pi && velPlanIn.adcx!=0 )
	{
		Struc_ActuPra_Out.runstate = back_right;
		drivestate = back_right;
		g_slaveReg[5] = 9;
	}
	
	/*原地右转 */
	if ((Struc_ActuPra_Out.steering_angle >=0 &&  velPlanIn.adcx>0 && Struc_ActuPra_Out.steering_angle < 1/6.0 *pi) \
	|| (Struc_ActuPra_Out.steering_angle >11/6.0*pi  && Struc_ActuPra_Out.steering_angle <2 *pi))	
	{
		Struc_ActuPra_Out.runstate = turnself_right;
		drivestate = turnself_right;
		g_slaveReg[5] = 5;
	}

	/*原地左转 */
	if (Struc_ActuPra_Out.steering_angle >5/6.0*pi && Struc_ActuPra_Out.steering_angle < 7/6.0 *pi)	
	{
		Struc_ActuPra_Out.runstate = turnself_left;
		drivestate = turnself_left;
		g_slaveReg[5] = 4;
	}
	// printf("L:%lf,R:%lf,ADCX:%d,ADCY:%d\r\n",Struc_ActuPra_Out.L_Dutycycle,Struc_ActuPra_Out.R_Dutycycle,velPlanIn.adcx,velPlanIn.adcy);	
	switch (drivestate)
	{
		case idle: 
			LeftMoterStop();
			RightMoterStop();
			break;
		case forward:
			LeftMoterMove(0,Struc_ActuPra_Out.L_Dutycycle);
			RightMoterMove(1,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case front_right:
			LeftMoterMove(0,Struc_ActuPra_Out.L_Dutycycle);
			RightMoterMove(1,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case front_left:
			LeftMoterMove(0,Struc_ActuPra_Out.L_Dutycycle);
			RightMoterMove(1,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case backward : /*backward velocity is half of set_Maximum_Strspeed*/
			LeftMoterMove(1,Struc_ActuPra_Out.L_Dutycycle);
			RightMoterMove(0,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case back_left:
			LeftMoterMove(1,Struc_ActuPra_Out.L_Dutycycle);
			RightMoterMove(0,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case back_right:
			LeftMoterMove(1,Struc_ActuPra_Out.L_Dutycycle);
			RightMoterMove(0,Struc_ActuPra_Out.R_Dutycycle);
			break;
		case turnself_right:
			LeftMoterMove(0,Struc_ActuPra_Out.L_Dutycycle);
			RightMoterMove(0,Struc_ActuPra_Out.R_Dutycycle);		
			break;
		case turnself_left:
			LeftMoterMove(1,Struc_ActuPra_Out.L_Dutycycle);
			RightMoterMove(1,Struc_ActuPra_Out.R_Dutycycle);		
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
void brake_excute(void)
{
/*松开抱闸*/
	if (Struc_ActuPra_Int.adcx < -400 || Struc_ActuPra_Int.adcx > 400 || Struc_ActuPra_Int.adcy > 400 || Struc_ActuPra_Int.adcy < -400)
	{
			brake(0);	
			brakeflage = 0;	
	}	
	else
/*锁住抱闸*/
	{	
		brakeflage++;
		if (brakeflage > 25)
		{ 
			brake(1);
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

  switch (g_slaveReg[64])
	{
		case 1: /*normal mode*/
			if (g_slaveReg[73] == 1)/*one level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 1.5 ;
			}
			else if (g_slaveReg[73] == 2)/*two level*/
	   	    {
				Struc_ActuPra_Int.set_Maximum_Strspeed =2.5;// 4.0 ;
			}
			else if (g_slaveReg[73] == 3)/*three level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed =3.5;// 6.6 ;

			}
			else if (g_slaveReg[73] == 4)/*four level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed =4.0;// 9.3 ;

			}
			else if (g_slaveReg[73] == 5)/*five level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 5.0;//12.0 ;

			}
			else
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 1.5 ;

			}
			break;
		case 2 :/*indoor mode*/
			if (g_slaveReg[73] == 1)/*one level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 1.5 ;

			}
			else if (g_slaveReg[73] == 2)/*two level*/
	   	    {
				Struc_ActuPra_Int.set_Maximum_Strspeed = 2.5 ;

			}
			else if (g_slaveReg[73] == 3)/*three level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 3.5 ;

			}
			else if (g_slaveReg[73] == 4)/*four level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 4.0 ;

			}
			else if (g_slaveReg[73] == 5)/*five level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 5.0;//5.0 ;

			}
			else
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 1.5 ;

			}
			break;			
		case 3 :/*outdoor mode*/
			if (g_slaveReg[73] == 1)/*one level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 1.5 ;

			}
			else if (g_slaveReg[73] == 2)/*two level*/
	   	    {
				Struc_ActuPra_Int.set_Maximum_Strspeed = 2.5 ;

			}
			else if (g_slaveReg[73] == 3)/*three level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 3.5;//7.0 ;

			}
			else if (g_slaveReg[73] == 4)/*four level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed =4.0;// 10.0 ;

			}
			else if (g_slaveReg[73] == 5)/*five level*/
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed =5.0;// 12.0 ;

			}
			else
			{
				Struc_ActuPra_Int.set_Maximum_Strspeed = 1.5 ;
			}
			break;
		default:
			Struc_ActuPra_Int.set_Maximum_Strspeed = 2.0 ;
			break;
	}
    // 远程连接 或者通讯失败 默认2档速度
	if(g_slaveReg[78]||comheartstate.com_state == Fail)
	{
		Struc_ActuPra_Int.set_Maximum_Strspeed = 2.0 ; 
	}
	// /*低速限制*/
	// if ((chairkinematicspra.y_o>50.8 && chairkinematicspra.y_o<152.4) ||
	// 	adcdata.backboard_pos<1300 ||
	// 	(fabs(chairkinematicspra.theta_x2X)>6.0 && fabs(chairkinematicspra.theta_x2X)<10.0) ||
	// 	adcdata.legangle_pos>1800)
	// {
	// 	Struc_ActuPra_Int.set_Maximum_Strspeed =Value_limitf(0,Struc_ActuPra_Int.set_Maximum_Strspeed,3.0);	
	// }
	// /*超低速限制*/
	// if (chairkinematicspra.y_o>152.4 ||
	// fabs(chairkinematicspra.theta_x2X)>10.0||
	// (adcdata.legangle_pos>1800 && fabs(chairkinematicspra.theta_x2X)>6.0 && fabs(chairkinematicspra.theta_x2X)<10.0)||
	// (chairkinematicspra.y_o>50.8 && fabs(chairkinematicspra.theta_x2X)>6.0 && fabs(chairkinematicspra.theta_x2X)<10.0)
	// )
	// {
	// 	Struc_ActuPra_Int.set_Maximum_Strspeed =Value_limitf(0,Struc_ActuPra_Int.set_Maximum_Strspeed,1.5);	
	// }	
}

/**
 * @description: 底盘驱动程序代码 主要用到定时器 3 和 定时器 9 控制左右电机占空比
 * @return 无
 */
void underpanExcute(void)
{
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
		#if defined JOYSTIC_AI
		/* X 数据清偏 */
		if (adcdata.adc_x > 0)
		{
			Struc_ActuPra_Int.adcx = adcdata.adc_x - xadc_Dim;
		}
		else if (adcdata.adc_x < 0)
		{
			Struc_ActuPra_Int.adcx = adcdata.adc_x + xadc_Dim;
		}
		else
		{
			Struc_ActuPra_Int.adcx = 0;
		}
		
		Struc_ActuPra_Int.adcx = slopelimitx( Struc_ActuPra_Int.adcx,25);  
		
		/* Y 数据清偏移*/
		if (adcdata.adc_y > 0)	
		{
			Struc_ActuPra_Int.adcy = adcdata.adc_y - yadc_Dim;
		}
		 else if (adcdata.adc_y < 0)
		{
			Struc_ActuPra_Int.adcy = adcdata.adc_y+ yadc_Dim;
		}	
		else
		{
			Struc_ActuPra_Int.adcy = 0 ;
		}
		Struc_ActuPra_Int.adcy = slopelimity( Struc_ActuPra_Int.adcy,25); 	
		#endif
		#if defined JOYSTIC_DI	
		Struc_ActuPra_Int.adcx =mlxdata.xdata;// local_slopelimitx(mlxdata.xdata,75,150);  
		Struc_ActuPra_Int.adcy =mlxdata.ydata;// local_slopelimity(mlxdata.ydata,75,150);  	 
		#endif
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

    /*摇杆数据增量约束*/
	Struc_ActuPra_Int.adcx = local_slopelimitx(Struc_ActuPra_Int.adcx,75,150);  
	Struc_ActuPra_Int.adcy = local_slopelimity(Struc_ActuPra_Int.adcy,75,150);
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
		brake(1);
	}
	velocity_maping(Struc_ActuPra_Int); /*速度规划 */
}

/**
 * @description: 获取陀螺仪 的pry欧拉角
 * @return 空 rpy欧拉角数值放置静态变量 pitch roll yaw 中
 */
void MPU6050Excute(void)
{
	static short res,gyrox,gyroy,gyroz,aacx,aacy,aacz,Temperature;
	Temperature =  MPU_Get_Temperature();
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	// printf("%d,%d,%d,%d,%d,%d,%d\n",Temperature,gyrox,gyroy,gyroz,aacx,aacy,aacz);
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	mpu_dmp_get_data(&pitch,&roll,&yaw);       
	g_slaveReg[24] = (int16_t)(pitch*100);
	g_slaveReg[25] = (int16_t)(roll*100);
	g_slaveReg[26] = (int16_t)(yaw*100); 
	// printf("pitch:%d,roll:%d,yaw:%d\n",g_slaveReg[24],g_slaveReg[25],g_slaveReg[26]);
}


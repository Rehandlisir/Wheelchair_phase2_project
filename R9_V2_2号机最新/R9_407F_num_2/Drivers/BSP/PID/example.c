/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/R9CODE/R9_V2_2号机最新/R9_V2/R9_407F_num_2/Drivers/BSP/PID/example.c
 * @Description  :  
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-09-27 16:11:32
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
//stabilization姿态控制任务
// void stabilization_task(void *p_arg)
// {
// 	OS_ERR err;
// 	CPU_SR_ALLOC();

// 	int dt_ms = 1000 / ATTITUDE_UPDATE_RATE; //姿态数据采样周期,默认500Hz,2ms
// 	float ft = (float)(dt_ms) / 1000.0;		 //积分间隔，单位秒
// 	float throttle_base;					 //油门基础值，由油门通道决定
// 	float zoom_factor = 0.10f;				 //转弯角速度
// 	attitude_t realAngle, expectedAngle, expectedRate;
// 	attitude_t realRate, output;

// 	attitudeControlInit();
	
// 	while (1)
// 	{
// /********************************   航向角姿态控制  ****************************************/
// /********************************   油门 控制      ****************************************/
// 		//zoom_factor速度放大因子
// 		expectedAngle.yaw -= (float)(command[YAW]) * zoom_factor * ft;
// 		if (expectedAngle.yaw > 180.0f)
// 			expectedAngle.yaw -= 360.0f;
// 		if (expectedAngle.yaw < -180.0f)
// 			expectedAngle.yaw += 360.0f;

// 		//油门值,最高速9000，减速输出400rpm
// 		if (command[SPEED_MODE] == HIGH_SPEED)
// 			throttle_base = (float)(command[THROTTLE] * 8);
// 		else if (command[SPEED_MODE] == LOW_SPEED)
// 			throttle_base = (float)(command[THROTTLE] * 4);

// 		//没有油门输出，也没有转弯信号，此时机器人在静止状态
// 		//始终把当前姿态角作为期望姿态角
// 		//不使能PID计算，复位所有PID
// 		if (command[THROTTLE] == 0 && command[YAW] == 0)
// 		{
// 			expectedAngle.yaw = realAngle.yaw;
// 			attitudeResetAllPID(); //PID复位
// 			expectedRate.yaw = 0;
// 			output.yaw = 0;
// 		}
// 		//有油门输出，说明机器人在运动状态，此时应该做姿态控制
// 		else
// 		{
// 			//姿态角串级pid计算
// 			attitudeAnglePID(&realAngle, &expectedAngle, &expectedRate); /* 角度环PID */
// 			attitudeRatePID(&realRate, &expectedRate, &output);			 /* 角速度环PID */
// 		}

// 		//pid控制量分配到电机混控
// 		set_speed[1] = throttle_base - output.yaw;
// 		set_speed[0] = set_speed[1];
// 		set_speed[3] = -(throttle_base + output.yaw);
// 		set_speed[2] = set_speed[3];
	
// 		//延时采样
// 		delay_ms(dt_ms);
// 	}
// }

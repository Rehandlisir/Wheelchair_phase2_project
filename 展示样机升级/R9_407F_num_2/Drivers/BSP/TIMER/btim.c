/**
 ****************************************************************************************************
 * @file        btim.c
 * @author    lis
 * @version     V1.0
 * @date        2024
 * @brief       基本定时器 驱动代码
 * @license     复成医疗
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:rehand 探索者 F407开发板
 * 复成医疗
 * rehand
 *复成医疗
 * 复成医疗
 *
 * 修改说明
 * V1.0 20211015
 * 第一次发布
 *
 ****************************************************************************************************
 */


#include "./BSP/TIMER/btim.h"
#include "./BSP/LED/led.h"

TIM_HandleTypeDef g_timx_handler;         /* 定时器参数句柄 */

/**
 * @brief       基本定时器TIMX定时中断初始化函数
 * @note
 *              基本定时器的时钟来自APB1,当PPRE1 ≥ 2分频的时候
 *              基本定时器的时钟为APB1时钟的2倍, 而APB1为42M, 所以定时器时钟 = 84Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr : 自动重装值。
 * @param       psc : 时钟预分频数
 * @retval      无
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    g_timx_handler.Instance = BTIM_TIMX_INT;                      /* 定时器x */
    g_timx_handler.Init.Prescaler = psc;                          /* 分频 */
    g_timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;         /* 递增计数模式 */
    g_timx_handler.Init.Period = arr;                             /* 自动装载值 */
    HAL_TIM_Base_Init(&g_timx_handler);
    
    HAL_TIM_Base_Start_IT(&g_timx_handler);                       /* 使能定时器x和定时器更新中断 */
}

  


/**
 * @brief       定时器底层驱动，开启时钟，设置中断优先级
                此函数会被HAL_TIM_Base_Init()函数调用
 * @param       无
 * @retval      无
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();                     /* 使能TIMx时钟 */
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 1, 1); /* 抢占1，子优先级3 */
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);         /* 开启ITMx中断 */
    }
}

/**
 * @brief       基本定时器TIMX中断服务函数
 * @param       无
 * @retval      无
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_handler);  /* 定时器回调函数 */
}

/**
 * @brief       回调函数，定时器中断服务函数调用
 * @param       无
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
		OS_IT_RUN();			
//		Modbus 从机被RK3588读写
		if(slavemodbus.timrun != 0)//运行时间！=0表明
			{
			slavemodbus.timout++;
			if(slavemodbus.timout >=8)
			{
				slavemodbus.timrun = 0;
				slavemodbus.reflag = 1;//接收数据完毕
			}
				
			}

		if(modbus_dap21.timrun != 0)//运行时间！=0表明
			{
			modbus_dap21.timout++;
			if(modbus_dap21.timout >=8)
				{
					modbus_dap21.timrun = 0;
					modbus_dap21.reflag = 1;//接收数据完毕
				}
			
			}
		
			comheartstate.detect_time++; // 检测完上一次通讯状态后计数
			if (comheartstate.detect_time > 2000) // 距离上一次检测过去了1000ms
			{
				comheartstate.detect_falge = 1;
				comheartstate.detect_time = 0; 
			}
			// float templv;
			// float templI;
			// float temprv;
			// float temprI;
			/**************************************左路电机 电枢电压、电流、速度采集***************************************************/
			// templv = (adcdata.A2_ADC-adcdata.A1_ADC)*ADC2VOLATAGE;
			// gl_motor_data.volatage =lowPassFilter(&lowpassl_volatage,templv);//
			// if (fabs(gl_motor_data.volatage)<0.05)
			// {
			// 	gl_motor_data.volatage = 0.0 ;
			// }
			// /*电流采集速度计算*/
			// if (gl_motor_data.volatage>0.05)
			// {
			// 	gl_motor_data.current = (adcdata.ASH1_DIFF_AD/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);; 
			// 	gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
			// 	gl_motor_data.speed = -(fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
			// }
			// else if (gl_motor_data.volatage <-0.05)  
			// {
			// 	gl_motor_data.current =(adcdata.ASH2_DIFF_AD/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);  
			// 	gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
			// 	gl_motor_data.speed = (fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
			// }
			// else
			// {
			// 	gl_motor_data.current = 0;
			// 	gl_motor_data.speed = 0;  
			// }
			
	
	//    /**************************************右路电机 电枢电压、电流、速度采集***************************************************/
			/*右路电机 电枢电压、电流、速度采集*/
		// 	gr_motor_data.volatage= (adcdata.B2_ADC -adcdata.B1_ADC)*ADC2VOLATAGE;
		// 	gr_motor_data.volatage =lowPassFilter(&lowpassr_volatage,gr_motor_data.volatage);
		// 	if (fabs(gr_motor_data.volatage)<0.05)
		// 	{
		// 		gr_motor_data.volatage = 0.0;
		// 	}
		// 	/*电流采集速度计算*/
		// 	if (gr_motor_data.volatage>0.05)
		// 	{
		// 		gr_motor_data.current = (adcdata.BSH1_DIFF_AD /4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
		// 		gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
		// 		gr_motor_data.speed = -(fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;
		// 	}
		// 	else if (gr_motor_data.volatage<-0.05)
		// 	{
		// 		gr_motor_data.current = (adcdata.BSH2_DIFF_AD /4096.0 *3.3- 0.9892)/(2.86799*SAMP_RA);
		// 		gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
		// 		gr_motor_data.speed = (fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;

		// 	}
		// 	else
		// 	{
		// 		gr_motor_data.current = 0;
		// 		gr_motor_data.speed = 0;
		// 	}
		//  /**************左右电机 PWM PID闭环 计算****************************/	
		// 		gl_motor_data.pwm = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
		// 		gr_motor_data.pwm = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed);

		// 		/*电流保护*/
		// 		static uint16_t current_conttimes;
		// 		static uint8_t stopcarflage;
		// 		if (gl_motor_data.current > 70.0 || gr_motor_data.current > 70.0)
		// 		{
		// 			current_conttimes++;
		// 			if (current_conttimes > 3000)
		// 			{
		// 				stopcarflage = 1;
		// 				current_conttimes = 0;
		// 			}
		// 		}
		// 		if (stopcarflage)
		// 		{  
		// 			//电流超限紧急停机处理
		// 			gl_motor_data.pwm = 0;
		// 			gr_motor_data.pwm = 0;
		// 			LED1(0); // 亮起故障指示灯
		// 		}
		// 		/*算术平均滤波占空比滤波处理*/
		// 		// gl_motor_data.pwm = filterValue_float(&filter_Lpwm,gl_motor_data.pwm);
		// 		// gr_motor_data.pwm = filterValue_float(&filter_Rpwm,gr_motor_data.pwm);
		// 		/*占空比约束*/
		// 		gl_motor_data.pwm = Value_limitf(-0.8, gl_motor_data.pwm, 0.8);
		// 		gr_motor_data.pwm = Value_limitf(-0.8, gr_motor_data.pwm, 0.8);



		    float templv;
			float templI;
			float temprv;
			float temprI;
			/**************************************左路电机 电枢电压、电流、速度采集***************************************************/
			templv = (adcdata.A1_ADC-adcdata.A2_ADC)*ADC2VOLATAGE;
			gl_motor_data.volatage =lowPassFilter(&lowpassl_volatage,templv);
			// 去除电压波动
			if (fabs(gl_motor_data.volatage)<0.05)
			{
				gl_motor_data.volatage = 0.0 ;
			}
			/*电流采集及换算*/
			if (gl_motor_data.volatage<0)
			{
				gl_motor_data.current = sign(gl_motor_data.volatage)*(adcdata.ASH1_DIFF_AD/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);

				gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
			}
			else if(gl_motor_data.volatage>0)
			{
				gl_motor_data.current =sign(gl_motor_data.volatage)*(adcdata.ASH2_DIFF_AD/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);  
				gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
			}
			else
			{
				gl_motor_data.current = 0;
			}
			//去除电流波动
			if (fabs(gl_motor_data.current)<0.05)// || gl_motor_data.current<0.0)
			{
				gl_motor_data.current = 0.0 ;
			}
			/*内阻分压及速度计算*/
			gl_motor_data.volatage_Ra =gl_motor_data.current * MOTER_RA;
			gl_motor_data.speed = (gl_motor_data.volatage - gl_motor_data.volatage_Ra)/MOTER_CE;

			//去除速度波动
			if (fabs(gl_motor_data.speed)<0.5)
			{
				gl_motor_data.speed = 0.0 ;
			}

	//    /**************************************右路电机 电枢电压、电流、速度采集***************************************************/
			/*右路电机 电枢电压、电流、速度采集*/
			gr_motor_data.volatage= (adcdata.B1_ADC -adcdata.B2_ADC)*ADC2VOLATAGE;
			gr_motor_data.volatage =lowPassFilter(&lowpassr_volatage,gr_motor_data.volatage);
			if (fabs(gr_motor_data.volatage)<0.05)
			{
				gr_motor_data.volatage = 0.0;
			}
			/*电流采集及换算*/
			if (gr_motor_data.volatage<0)
			{
				gr_motor_data.current =sign(gr_motor_data.volatage)*(adcdata.BSH1_DIFF_AD /4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
				gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);

			}
			else if(gr_motor_data.volatage>0)
			{
				gr_motor_data.current =sign(gr_motor_data.volatage)*(adcdata.BSH2_DIFF_AD /4096.0 *3.3- 0.9892)/(2.86799*SAMP_RA);
				gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
			}
			else
			{
				gr_motor_data.current = 0;
			}
			//去除电流波动
			if (fabs(gr_motor_data.current)<0.05)//|| gr_motor_data.current<0.0)
			{
				gr_motor_data.current = 0.0 ;
			}
			/*内阻分压*/
			gr_motor_data.volatage_Ra = gr_motor_data.current * MOTER_RA;
			/*速度计算*/
			gr_motor_data.speed = (gr_motor_data.volatage - gr_motor_data.volatage_Ra)/MOTER_CE;
           //去除速度波动
			if (fabs(gr_motor_data.speed)<0.5)
			{
				gr_motor_data.speed = 0.0 ;
			}


		 /**************左右电机速度闭环PID 计算********************************************************************/	
				gl_motor_data.pwm = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
				gr_motor_data.pwm = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed);
				/*电流保护*/
				static uint16_t current_conttimes;
				static uint8_t stopcarflage;
				if (fabs(gl_motor_data.current > 60.0) || fabs(gr_motor_data.current > 60.0))
				{
					current_conttimes++;
					if (current_conttimes > 2000)
					{
						stopcarflage = 1;
						current_conttimes = 0;
					}
				}
				if (stopcarflage)
				{  
					//电流超限紧急停机处理
					gl_motor_data.pwm = 0;
					gr_motor_data.pwm = 0;
					LED1(0); // 亮起故障指示灯
				}
				/*算术平均滤波占空比滤波处理*/
				// gl_motor_data.pwm = filterValue_float(&filter_Lpwm,gl_motor_data.pwm);
				// gr_motor_data.pwm = filterValue_float(&filter_Rpwm,gr_motor_data.pwm);
				/*占空比约束*/
				gl_motor_data.pwm = Value_limitf(-0.8, gl_motor_data.pwm, 0.8);
				gr_motor_data.pwm = Value_limitf(-0.8, gr_motor_data.pwm, 0.8);

    }		
}

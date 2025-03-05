
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
#include "./BSP/R9/brake.h"
#include "./BSP/CAN/can.h"
#include "./BSP/ADC/adc.h"
// STRUCT_BRAKE struc_brake;
TIM_HandleTypeDef g_timx_handler;         /* 定时器参数句柄 */
ProtectionContext prote_Lmoter;  // 左电机限流器
IR_CompensationContext ircom_Lmoter;  // 左电机IR补偿器
ProtectionContext prote_Rmoter;  // 左电机限流器
IR_CompensationContext ircom_Rmoter;  // 左电机IR补偿器
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

/**
 * @brief       回调函数，定时器中断服务函数调用
 * @param       无
 * @retval      无
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    float templv;
    float templI;
    float temprv;
    float temprI;
    if (htim->Instance == BTIM_TIMX_INT)
    {
		OS_IT_RUN();
       /*摇杆can通讯心跳信号清0 操作*/
    //    static uint8_t joy_t;
    //    static uint8_t pid_t;       
    //     joy_t++;
    //     if (joy_t>200)
    //     {
    //         CanjoysticbufReceive[4] =0;
    //         joy_t=0;
    //     }
        /**************************************左路电机 电枢电压、电流、速度采集***************************************************/
        gl_motor_data.volatage = (g_adc_val[3]-g_adc_val[0])*ADC2VOLATAGE;
        gl_motor_data.volatage =lowPassFilter(&lowpassl_volatage,gl_motor_data.volatage);//
			// 去除电压波动
			if (fabs(gl_motor_data.volatage)<0.05)
			{
				gl_motor_data.volatage = 0.0 ;
			}
			/*电流采集及换算*/
			if(gl_motor_data.volatage<0)
			{
				gl_motor_data.current = sign(gl_motor_data.volatage)*(g_adc_val[2]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);

				gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
			}
			else if(gl_motor_data.volatage>0)
			{
				gl_motor_data.current =sign(gl_motor_data.volatage)*(g_adc_val[1]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);  
				gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
			}
			else
			{
				gl_motor_data.current = 0;
			}
			//去除电流波动
			if (fabs(gl_motor_data.current)<0.2)// || gl_motor_data.current<0.0)
			{
				gl_motor_data.current = 0.0 ;
			}
			/*内阻分压及速度计算*/
			gl_motor_data.speed = (gl_motor_data.volatage - gl_motor_data.current * MOTER_RA)/MOTER_CE;

			//去除速度波动
			if (fabs(gl_motor_data.speed)<0.5)
			{
				gl_motor_data.speed = 0.0 ;
			}
        

//    /**************************************右路电机 电枢电压、电流、速度采集***************************************************/
        /*右路电机 电枢电压、电流、速度采集*/
        gr_motor_data.volatage = (g_adc_val[9]-g_adc_val[10])*ADC2VOLATAGE;
        gr_motor_data.volatage =lowPassFilter(&lowpassr_volatage, gr_motor_data.volatage);
        if (fabs(gr_motor_data.volatage)<0.05)
        {
            gr_motor_data.volatage = 0.0;
        }
        /*电流采集及换算*/
        if(gr_motor_data.volatage<0)
        {
            gr_motor_data.current =sign(gr_motor_data.volatage)*(g_adc_val[8]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
            gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);

        }
        else if(gr_motor_data.volatage>0)
        {
            gr_motor_data.current =sign(gr_motor_data.volatage)*(g_adc_val[7] /4096.0 *3.3- 0.9892)/(2.86799*SAMP_RA);
            gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
        }
        else
        {
            gr_motor_data.current = 0;
        }
        //去除电流波动
        if (fabs(gr_motor_data.current)<0.2)//|| gr_motor_data.current<0.0)
        {
            gr_motor_data.current = 0.0 ;
        }

        /*速度计算*/
        gr_motor_data.speed = (gr_motor_data.volatage - gr_motor_data.current * MOTER_RA)/MOTER_CE;
       //去除速度波动
        if (fabs(gr_motor_data.speed)<0.5)
        {
            gr_motor_data.speed = 0.0 ;
        }

		//  /**************左右电机速度闭环PID 计算********************************************************************/	
		// 		gl_motor_data.pwm = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
		// 		gr_motor_data.pwm = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed);
				/*电流保护*/
				// static uint16_t current_conttimes;
				// static uint8_t stopcarflage;
				// if (fabs(gl_motor_data.current > 60.0) || fabs(gr_motor_data.current > 60.0))
				// {
				// 	current_conttimes++;
				// 	if (current_conttimes > 2000)
				// 	{
				// 		stopcarflage = 1;
				// 		current_conttimes = 0;
				// 	}
				// }
				// if (stopcarflage)
				// {  
				// 	//电流超限紧急停机处理
				// 	gl_motor_data.pwm = 0;
				// 	gr_motor_data.pwm = 0;
				// 	LED1(0); // 亮起故障指示灯
				// }
		// 		/*算术平均滤波占空比滤波处理*/
		// 		// gl_motor_data.pwm = filterValue_float(&filter_Lpwm,gl_motor_data.pwm);
		// 		// gr_motor_data.pwm = filterValue_float(&filter_Rpwm,gr_motor_data.pwm);
		// 		/*占空比约束*/
		// 		gl_motor_data.pwm  = Value_limitf(-0.8, gl_motor_data.pwm, 0.8);
		// 		gr_motor_data.pwm = Value_limitf(-0.8, gr_motor_data.pwm, 0.8);

		/************************开环 + IR 补偿**********************************/
		// 电流保护
		UpdateProtectionState(&prote_Lmoter, gl_motor_data.current);
		UpdateProtectionState(&prote_Rmoter, gr_motor_data.current);
		gl_motor_data.pwm=ApplyCurrentLimitedIRCompensation(&ircom_Lmoter, &prote_Lmoter, Struc_ActuPra_Out.LN_Velocity, gl_motor_data.current);
		gr_motor_data.pwm=ApplyCurrentLimitedIRCompensation(&ircom_Rmoter, &prote_Rmoter, Struc_ActuPra_Out.RN_Velocity, gr_motor_data.current);
	}
}

float min_float(float a, float b) 
{
    // 处理NaN（无效数值）的特殊情况 
    if (isnan(a)) return b;  // 若a非法则返回b 
    if (isnan(b)) return a;  // 若b非法则返回a 
    return (a < b) ? a : b;  // 核心比较逻辑 
}


float max_float(float a, float b) 
{
    // 处理NaN（无效数值）的特殊情况 
    if (isnan(a)) return b;  // 若a非法则返回b 
    if (isnan(b)) return a;  // 若b非法则返回a 
    return (a < b) ? b : a;  // 核心比较逻辑 
}

void UpdateProtectionState(ProtectionContext* ctx, float I_actual) 
{
    //------------ 全局折叠保护触发检测（优先级最高） -------------
    if (ctx->state != STATE_FOLDBACK) 
	{
        /* 折叠保护触发条件监测 */
        if (fabs(I_actual) >= CURRENT_FOLDBACK_TH) 
		{
            ctx->foldback_timer += CONTROL_CYCLE_MS;
            
            // 如果累计时间达到触发阈值，强制进入折叠保护
            if (ctx->foldback_timer >= CURRENT_FOLDBACK_TIME_MS) 
			{
                ctx->state          = STATE_FOLDBACK;
                ctx->I_limit_current= CURRENT_FOLDBACK;
                ctx->foldback_timer = COOL_TIME_SEC;    // 初始化冷却倒计时
                ctx->boost_timer    = 0;                       // 清除Boost计时器
                return; // 立即终止状态机，防止其他状态干扰
            }
        } 
		else 
		{
            // 电流低于保护阈值，复位累积计时
            ctx->foldback_timer = 0;
        }
    }

    //------------ 其他状态的常规切换逻辑 -------------
    switch(ctx->state) 
	{
        case STATE_NORMAL:
            /* Boost触发条件检测 */
            if (fabs(I_actual) > BOOST_DRIVE_CURRENT) {
                ctx->state          = STATE_BOOST;
                ctx->boost_timer    = 0;
                ctx->I_limit_current= BOOST_DRIVE_CURRENT;     // 激活Boost限流
            }
            break;

        case STATE_BOOST:
            /* Boost时间管理 */
            ctx->boost_timer += CONTROL_CYCLE_MS;
            if (ctx->boost_timer >= BOOST_DRIVE_TIME_MS) {
                ctx->state          = STATE_NORMAL; // Boost结束，回归正常
                ctx->I_limit_current= MAX_CURRENT_LIMIT;
            }
            break;

        case STATE_FOLDBACK:
            /* 冷却倒计时管理 */
            if (ctx->foldback_timer > 0) {
                ctx->foldback_timer -= CONTROL_CYCLE_MS;
            } else {
                ctx->state          = STATE_NORMAL; // 冷却完毕，恢复标准限流
                ctx->I_limit_current= MAX_CURRENT_LIMIT;
				ctx->boost_timer = 0; 
            }
            break;
    }
}

// 动态IR补偿 + 电流硬约束
float ApplyCurrentLimitedIRCompensation( IR_CompensationContext* irc,ProtectionContext* prot,float speed,float I_measured) 
{
	irc->V_battery = g_r9sys_data.r9_battary_v;
    //-- 阶段1：基于当前限流值实时约束反电动势需求 --
    // 无论系统处于任何状态（Normal/Boost/Foldback），均动态计算最大允许反电动势
    float I_max_current = prot->I_limit_current; // 当前状态的限流值（40A、50A或20A）
    float V_eff_max = sign(speed)*I_max_current * irc->R_int;
	float V_eff_clamped;
	float V_eff_desired = speed*MOTER_CE ;
	float I_applied ;
	if (V_eff_desired>0)
	{
		V_eff_clamped= min_float(V_eff_desired, V_eff_max)+0.5;
		if (I_measured > I_max_current)  // 应用电流约束
		{
			I_applied = I_max_current;
		}
		else
		{
			I_applied = I_measured;
		}
	}
	else if(V_eff_desired<0)
	{
		V_eff_clamped= max_float(V_eff_desired, V_eff_max)-0.5;	
		if (I_measured < -I_max_current)// 应用电流约束
		{
			I_applied = -I_max_current;
		}
		else				// 应用电流约束
		{
			I_applied = I_measured;
		}

	}
	else
	{
		V_eff_clamped = 0;
		I_applied = 0;
	}
    //-- 阶段2：执行IR补偿计算目标电圧 --
    float V_applied_target = V_eff_clamped + I_applied * irc->R_int;
	//目标电压约束
	
	V_applied_target = Value_limitf(-irc->V_battery*0.9, V_applied_target, irc->V_battery*0.9);
    //-- 阶段3：占空比实时二次限流保护 --
    float duty = V_applied_target / irc->V_battery;
    // if (fabs(I_measured) > I_max_current) 
	// {
    //     // 若实际电流超限，按比例降权占空比（强制约束）
    //     duty *= (I_max_current / I_measured);
    // }
	return duty;
}


void ir_compensation_init(void)
{
	prote_Lmoter.I_limit_current = MAX_CURRENT_LIMIT;
	prote_Lmoter.state = STATE_NORMAL;
	prote_Lmoter.boost_timer = 0;
	prote_Lmoter.foldback_timer = 0;
 
	prote_Rmoter.I_limit_current = MAX_CURRENT_LIMIT;
	prote_Rmoter.state = STATE_NORMAL;
	prote_Rmoter.boost_timer = 0;
	prote_Rmoter.foldback_timer = 0;

	ircom_Lmoter.voltage_moter = gl_motor_data.volatage;
	ircom_Lmoter.R_int = MOTER_RA;
	// ircom_Lmoter.V_battery = 24.0;

	ircom_Rmoter.voltage_moter = gr_motor_data.volatage;
	ircom_Rmoter.R_int = MOTER_RA;
	// ircom_Rmoter.V_battery = 24.0;

	Struc_ActuPra_Out.LN_Velocity=0;
	Struc_ActuPra_Out.RN_Velocity=0;
}

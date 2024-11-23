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
#include "./BSP/R9/moterdriver.h"
#include "./BSP/R9/underpanControl.h"
#include "./BSP/PID/pid.h"
#include "./BSP/ADC/adc.h"
// STRUCT_BRAKE struc_brake;


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

volatile int g_timx_encode_count = 0;    /* 溢出次数 */                      
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
    {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&g_timx_encode_chy_handle))   /* 判断CR1的DIR位 */
        {
            g_timx_encode_count--;                                      /* DIR位为1，也就是递减计数 */
        }
        else
        {
            g_timx_encode_count++;                                      /* DIR位为0，也就是递增计数 */
        }
    }
    else if (htim->Instance == BTIM_TIMX_INT)
    {
        double moter_pwm_temp;
        int Encode_now = gtim_get_encode();                             /* 获取编码器值，用于计算速度 */
        speed_computer(Encode_now, 5);                                 /* 中位平均值滤除编码器抖动数据，10ms计算一次速度*/	
        // g_speed_pid.smaplse_Pid_detec_time++; 
        // if (g_speed_pid.smaplse_Pid_detec_time >1) 
        // {
        //     g_speed_pid.detect_flage = 1;
        //     g_speed_pid.smaplse_Pid_detec_time = 0;

        // }	
        if(Struc_ActuPra_Out.runstate!=idle)
		{
			if (1)
            {
                /* PID-速度外环 PID控制*/
                // Struc_ActuPra_Out.L_rpm = Value_limitf(0.0, Struc_ActuPra_Out.L_rpm, 175); // 输入目标转速约束
                // g_speed_pid.SetPoint = Struc_ActuPra_Out.L_rpm;
                // Struc_ActuPra_Out.L_Dutycycle = increment_pid_ctrl(&g_speed_pid, fabs(g_motor_data.speed));
                // Struc_ActuPra_Out.L_Dutycycle = Value_limitf(0.0, Struc_ActuPra_Out.L_Dutycycle, 0.99); // PID输出约束
                // Struc_ActuPra_Out.L_Dutycycle = filterValue_float(&filter_L,Struc_ActuPra_Out.L_Dutycycle);
                /*PID -电流内环 PID 控制*/
                Struc_ActuPra_Out.L_rpm = Value_limitf(0.0, Struc_ActuPra_Out.L_rpm, 175); // 输入目标电流
                g_current_pid.SetPoint = Struc_ActuPra_Out.L_rpm;
                moter_pwm_temp = increment_pid_ctrl(&g_current_pid, fabs(g_motor_data.current));
                Struc_ActuPra_Out.L_Dutycycle = (double)((Struc_ActuPra_Out.L_Dutycycle * 0.3) + (moter_pwm_temp * 0.7));
                Struc_ActuPra_Out.L_Dutycycle = Value_limitf(0.0, Struc_ActuPra_Out.L_Dutycycle , 0.99); // PID输出约束

            }

		}
		else
		{
			g_speed_pid.SetPoint =0;
			Struc_ActuPra_Out.L_Dutycycle = 0;
		} 	
       
        OS_IT_RUN();			
    }		
}



int gtim_get_encode(void)
{
    return ( int32_t )__HAL_TIM_GET_COUNTER(&g_timx_encode_chy_handle) + g_timx_encode_count * 65536;       /* 当前计数值+之前累计编码器的值=总的编码器值 */
}


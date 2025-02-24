
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

int8_t sign(float a)
{
    if (a>0)
    {
        return 1;
    }
    else if (a<0)
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
        if (fabs(gl_motor_data.volatage)<0.2)
        {
            gl_motor_data.volatage = 0.0 ;
        }
        /*电流采集速度计算*/
        if (gl_motor_data.volatage<0)
        {
            gl_motor_data.current = (g_adc_val[2]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
            gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
            // if (fabs(gl_motor_data.volatage) <= gl_motor_data.current * MOTER_RA )
            // {
            //     gl_motor_data.speed = 0;
            // }
            // else
            {
                gl_motor_data.speed = -(fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else if (gl_motor_data.volatage >0)  
        {
            gl_motor_data.current  = (g_adc_val[1]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
            gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current );
            // if (fabs(gl_motor_data.volatage) <= gl_motor_data.current * MOTER_RA )
            // {
            //     gl_motor_data.speed = 0;
            // }
            // else
            {
                gl_motor_data.speed = (fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else
        {
            gl_motor_data.volatage = 0;
            gl_motor_data.current = 0;
            gl_motor_data.speed = 0;  

        }
        

//    /**************************************右路电机 电枢电压、电流、速度采集***************************************************/
        /*右路电机 电枢电压、电流、速度采集*/
        gr_motor_data.volatage = (g_adc_val[9]-g_adc_val[10])*ADC2VOLATAGE;
        gr_motor_data.volatage =lowPassFilter(&lowpassr_volatage, gr_motor_data.volatage);
        if (fabs(gr_motor_data.volatage)<0.2)
        {
            gr_motor_data.volatage = 0.0;
        }
        /*电流采集速度计算*/
        if (gr_motor_data.volatage<0)
        {
            gr_motor_data.current =(g_adc_val[8]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
            gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
            // if (fabs(gr_motor_data.volatage) <= gr_motor_data.current * MOTER_RA)
            // {
            //     gr_motor_data.speed = 0;
            // }
            // else
            {
                gr_motor_data.speed = -(fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else if (gr_motor_data.volatage >0)
        {
            gr_motor_data.current =(g_adc_val[7]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
            gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
            // if (fabs(gr_motor_data.volatage) <= gr_motor_data.current * MOTER_RA )
            // {
            //     gr_motor_data.speed = 0;
            // }
            // else
            {
                gr_motor_data.speed = (fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else
        {
            gr_motor_data.volatage = 0;
            gr_motor_data.current = 0;
            gr_motor_data.speed = 0;
        }
     /**************左右电机 PWM PID闭环 计算****************************/	
        // gl_motor_data.pwm = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
        // gr_motor_data.pwm = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed);
        // /*理论占空比*/
        // gl_motor_data.theory_pwm  =fabs(Struc_ActuPra_Out.L_Velocity *0.083);
        // gr_motor_data.theory_pwm = fabs(Struc_ActuPra_Out.R_Velocity *0.083);
        // /* 占空比约束*/
        // /*算术平均滤波占空比滤波处理*/
        // gl_motor_data.pwm = filterValue_float(&filter_Lpwm,gl_motor_data.pwm);
        // gr_motor_data.pwm = filterValue_float(&filter_Rpwm,gr_motor_data.pwm);
        // /*占空比约束*/
        // gl_motor_data.pwm = Value_limitf(-0.8, gl_motor_data.pwm, 0.8);
        // gr_motor_data.pwm = Value_limitf(-0.8, gr_motor_data.pwm, 0.8);
        /*电枢电压平衡方程 U= I*Ra + Ke*N*/

        /*左轮前馈电压*/
        gl_motor_data.volatageff_target =gl_speed_pid.SetPoint * MOTER_CE;//0.75*sign(gl_speed_pid.SetPoint)*gl_motor_data.current * MOTER_RA + gl_speed_pid.SetPoint * MOTER_CE;
       /*右轮前馈电压*/
        gr_motor_data.volatageff_target  =gr_speed_pid.SetPoint * MOTER_CE;//0.75*sign(gr_speed_pid.SetPoint)*gr_motor_data.current * MOTER_RA + gr_speed_pid.SetPoint * MOTER_CE;
       /*速度闭环电压调制*/
       gl_motor_data.volatagefb_target = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
       gr_motor_data.volatagefb_target = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed); 
       /*最终控制电压*/
        gl_motor_data.volatagecmd_target = gl_motor_data.volatagefb_target+gl_motor_data.volatageff_target;
        gr_motor_data.volatagecmd_target = gr_motor_data.volatagefb_target+gr_motor_data.volatageff_target;
        /*电压约束-电流预测法*/
        gl_motor_data.predict_current = (gl_motor_data.volatagecmd_target - gl_speed_pid.SetPoint * MOTER_CE)/MOTER_RA;
        gr_motor_data.predict_current = (gr_motor_data.volatagecmd_target - gr_speed_pid.SetPoint * MOTER_CE)/MOTER_RA;
        if (gl_motor_data.predict_current > 80)
        {
            gl_motor_data.volatagecmd_target = 80*MOTER_RA + gl_speed_pid.SetPoint * MOTER_CE;

        }
        if (gr_motor_data.predict_current > 80)
        {
            gr_motor_data.volatagecmd_target = 80*MOTER_RA + gr_speed_pid.SetPoint * MOTER_CE;
        }
        /*最终占空比输出*/
        gl_motor_data.pwm = gl_motor_data.volatagecmd_target/g_r9sys_data.r9_battary_v;
        gr_motor_data.pwm = gr_motor_data.volatagecmd_target/g_r9sys_data.r9_battary_v;
        /*占空比约束*/
        gl_motor_data.pwm = Value_limitf(-0.85, gl_motor_data.pwm, 0.85);
        gr_motor_data.pwm = Value_limitf(-0.85, gr_motor_data.pwm, 0.85);

    }
	
}

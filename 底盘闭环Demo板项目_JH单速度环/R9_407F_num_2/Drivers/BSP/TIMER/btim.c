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
       static uint8_t joy_t;
       static uint8_t pid_t;       
        joy_t++;
        if (joy_t>200)
        {
            CanjoysticbufReceive[4] =0;
            joy_t=0;
        }
        /**************************************左路电机 电枢电压、电流、速度采集***************************************************/
        templv = (g_adc_val[0]-g_adc_val[3])*ADC2VOLATAGE;
        gl_motor_data.volatage =lowPassFilter(&lowpassl_volatage,templv);//
        if (fabs(gl_motor_data.volatage)<0.02)
        {
            gl_motor_data.volatage = 0.0 ;
        }
        /*电流采集速度计算*/
        if (gl_motor_data.volatage>0)
        {
            templI =  pow(g_adc_val[2],2)*ADC2CURRENT_P1+g_adc_val[2]*ADC2CURRENT_P2+ADC2CURRENT_P3;
            gl_motor_data.current =lowPassFilter(&lowpassl_current,templI);
            if (fabs(gl_motor_data.volatage) <= gl_motor_data.current * MOTER_RA)
            {
                gl_motor_data.speed = 0;
            }
            else
            {
                gl_motor_data.speed = (fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else if (gl_motor_data.volatage <0)  
        {
            templI = pow(g_adc_val[1],2)*ADC2CURRENT_P1+g_adc_val[1]*ADC2CURRENT_P2+ADC2CURRENT_P3;
            gl_motor_data.current =lowPassFilter(&lowpassl_current,templI);
            if (fabs(gl_motor_data.volatage) <= gl_motor_data.current * MOTER_RA )
            {
                gl_motor_data.speed = 0;
            }
            else
            {
                gl_motor_data.speed = -(fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
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
        temprv = (g_adc_val[10]-g_adc_val[9])*ADC2VOLATAGE;
        gr_motor_data.volatage =lowPassFilter(&lowpassr_volatage,temprv);
        if (fabs(gr_motor_data.volatage)<0.02)
        {
            gr_motor_data.volatage = 0.0;
        }
        /*电流采集速度计算*/
        if (gr_motor_data.volatage>0)
        {
            temprI =g_adc_val[8]*g_adc_val[8]*ADC2CURRENT_P1+g_adc_val[8]*ADC2CURRENT_P2+ADC2CURRENT_P3;
            gr_motor_data.current = lowPassFilter(&lowpassr_current,temprI);
            if (fabs(gr_motor_data.volatage) <= gr_motor_data.current * MOTER_RA )
            {
                gr_motor_data.speed = 0;
            }
            else
            {
                gr_motor_data.speed = (fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else if (gr_motor_data.volatage <0)
        {
            gr_motor_data.current =g_adc_val[7]*g_adc_val[7]*ADC2CURRENT_P1+g_adc_val[7]*ADC2CURRENT_P2+ADC2CURRENT_P3;
            gr_motor_data.current = Value_limitf(0,gr_motor_data.current,(fabs(gr_motor_data.volatage)/MOTER_RA));
            gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
            if (fabs(gr_motor_data.volatage) <= gr_motor_data.current * MOTER_RA )
            {
                gr_motor_data.speed = 0;
            }
            else
            {
                gr_motor_data.speed = -(fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else
        {
            gr_motor_data.volatage = 0;
            gr_motor_data.current = 0;
            gr_motor_data.speed = 0;
        }
     /**************左右电机 PWM PID闭环 计算****************************/	
      static uint8_t tl,tr;
      static float stoplpwm_initvalue,stoprpwm_initvalue;
      if (Struc_ActuPra_Int.adcx < -200 || Struc_ActuPra_Int.adcx > 200 || Struc_ActuPra_Int.adcy > 200 || Struc_ActuPra_Int.adcy < -200)
      {
        gl_motor_data.pwm = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
        gr_motor_data.pwm = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed);
        static uint16_t overcurrent_lcnt,overcurrent_rcnt;
        if (gl_motor_data.current> 70 )
        {
            overcurrent_lcnt ++;
            if (overcurrent_lcnt>5000)
            {
                gl_motor_data.pwm =  gl_motor_data.pwm*0.5;    
            }
            if (overcurrent_lcnt>10000)
            {

                overcurrent_lcnt = 0;
                gl_motor_data.pwm = 0;
            }

        }
        else
        {
            overcurrent_lcnt = 0;

        }

        if (gr_motor_data.current >70)
        {
            overcurrent_rcnt ++;
            if (overcurrent_rcnt>5000)
            {
                gr_motor_data.pwm =  gr_motor_data.pwm*0.5;    
            }
            if (overcurrent_rcnt >10000)
            {
                overcurrent_rcnt = 0;
                gr_motor_data.pwm = 0;
            }
        }
        else
        {
           overcurrent_rcnt = 0;
        }

        /*理论占空比*/
        gl_motor_data.theory_pwm  =fabs(Struc_ActuPra_Out.L_Velocity *0.083);
        gr_motor_data.theory_pwm = fabs(Struc_ActuPra_Out.R_Velocity *0.083);
        /* 占空比约束*/
        // gl_motor_data.pwm = Value_limitf(-4*gl_motor_data.theory_pwm, gl_motor_data.pwm, 4*gl_motor_data.theory_pwm);
        // gr_motor_data.pwm = Value_limitf(-4*gr_motor_data.theory_pwm, gr_motor_data.pwm, 4*gr_motor_data.theory_pwm);
        /*算术平均滤波占空比滤波处理*/
        gl_motor_data.pwm = filterValue_float(&filter_Lpwm,gl_motor_data.pwm);
        gr_motor_data.pwm = filterValue_float(&filter_Rpwm,gr_motor_data.pwm);
        /*占空比约束*/
        gl_motor_data.pwm = Value_limitf(-0.8, gl_motor_data.pwm, 0.8);
        gr_motor_data.pwm = Value_limitf(-0.8, gr_motor_data.pwm, 0.8);
      }
      else
      {
        gl_motor_data.pwm = 0.0;
        gr_motor_data.pwm = 0.0;
        // LeftMoterMove(gl_motor_data.pwm,0);
        // RightMoterMove(gr_motor_data.pwm,1);
        //重置PID 参数
        pid_init();
      }
    }		
}

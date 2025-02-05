/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/R9CODE/R9_V2_2号机最新/R9_V2/R9_407F_num_2/Drivers/BSP/R9/moterdriver.c
 * @Description  :  
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-09-18 15:43:45
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
/**
 ****************************************************************************************************
 * @file        moterdriver.c
 * @author      Lisir
 * @version     V1.0
 * @date        2021-10-14
 * @brief       moterdriver 驱动代码
 * @license     Copyright (c) 2024, 深圳复成医疗科技有限公司
 */
#include "./BSP/R9/moterdriver.h"
#include "./BSP/LEG_ KINEMATICS/LegRestKinematics.h"
TIM_HandleTypeDef g_time1_pwm_chy_handle;  /* 靠背推杆 */
TIM_HandleTypeDef g_time8_pwm_chy_handle;  /* 座板推杆*/
TIM_HandleTypeDef g_time2_pwm_chy_handle;  /*举升推杆 */
TIM_HandleTypeDef g_time4_pwm_chy_handle;  /*腿托长度及角度*/
TIM_HandleTypeDef g_time12_pwm_chy_handle; /* 前支撑轮*/
LineActor_CMD linerun_cmd;
LineActor_STATE liner_state;
ACTORLIMITPARA ActorLimitPara;

/**********************************底盘L 电机1 驱动输出 TIME1 CH1 CH2 CH1N CH2N  *************************************/
TIM_HandleTypeDef g_tim1_cplm_pwm_handle;                              /* 定时器1句柄 */
TIM_BreakDeadTimeConfigTypeDef g_sbreak1_dead_time1_config = {0};        /* 死区时间设置 */

/**
 *  @brief       高级定时器TIM1 互补输出 初始化函数（使用PWM模式1）
 * @note
 *              配置高级定时器TIMX 互补输出, 一路OCy 一路OCyN, 并且可以设置死区时间
 *              高级定时器的时钟来自APB2, 而PCLK2 = 168Mhz, 我们设置PPRE2不分频, 因此
 *              高级定时器时钟 = 168Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率, 单位 : Mhz
 * @param         {uint16_t} arr:自动重装值。
 * @param         {uint16_t} psc:预分频系数
 * @param         {uint8_t} dtg: 
 * 
**/
void MoterL_pwm_chy_init(uint16_t arr, uint16_t psc) // 左轮电机 atim_timx_cplm_pwm_init(100 - 1, 84 - 1); /* 20Khz波形輸出. */
{
    /*左轮电机*/ 
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_cplm_pwm = {0};

    ATIM_TIM1_CPLM_CLK_ENABLE();            /* TIMx 时钟使能 */
    ATIM_TIM1_CPLM_CH1_GPIO_CLK_ENABLE();   /* 通道X对应IO口时钟使能 */
    ATIM_TIM1_CPLM_CH1N_GPIO_CLK_ENABLE();  /* 通道X互补通道对应IO口时钟使能 */
    ATIM_TIM1_CPLM_CH2_GPIO_CLK_ENABLE();   /* 通道X对应IO口时钟使能 */
    ATIM_TIM1_CPLM_CH2N_GPIO_CLK_ENABLE();  /* 通道X互补通道对应IO口时钟使能 */
    
    ATIM_TIM1_CPLM_BKIN_GPIO_CLK_ENABLE();  /* 通道X刹车输入对应IO口时钟使能 */
  
    gpio_init_struct.Pin = ATIM_TIM1_CPLM_BKIN_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = ATIM_TIM1_CPLM_CHY_GPIO_AF;                /* 端口复用 */
    HAL_GPIO_Init(ATIM_TIM1_CPLM_BKIN_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ATIM_TIM1_CPLM_CH1_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM1_CPLM_CH1_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ATIM_TIM1_CPLM_CH1N_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM1_CPLM_CH1N_GPIO_PORT, &gpio_init_struct);
    
    gpio_init_struct.Pin = ATIM_TIM1_CPLM_CH2_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM1_CPLM_CH2_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ATIM_TIM1_CPLM_CH2N_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM1_CPLM_CH2N_GPIO_PORT, &gpio_init_struct);

    g_tim1_cplm_pwm_handle.Instance = ATIM_TIM1_CPLM;                       /* 定时器x */
    g_tim1_cplm_pwm_handle.Init.Prescaler = psc;                            /* 预分频系数 */
    g_tim1_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;           /* 向上计数模式 */
    g_tim1_cplm_pwm_handle.Init.Period = arr;                               /* 自动重装载值 */
    g_tim1_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;     /* CKD[1:0] = 10, tDTS = 4 * tCK_INT = Ft / 4 = 42Mhz*/
    g_tim1_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  /* 使能影子寄存器TIMx_ARR */
  
    HAL_TIM_PWM_Init(&g_tim1_cplm_pwm_handle) ;

    tim_oc_cplm_pwm.OCMode = TIM_OCMODE_PWM1;                               /* PWM模式1 */
    tim_oc_cplm_pwm.OCPolarity = TIM_OCPOLARITY_HIGH;                       /* OCy 高电平有效 */
    tim_oc_cplm_pwm.OCNPolarity = TIM_OCPOLARITY_HIGH;                      /* OCyN 高电平有效 */
    tim_oc_cplm_pwm.OCIdleState = TIM_OCIDLESTATE_RESET;                      /* 当MOE=0，OCx=0 */
    tim_oc_cplm_pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;                    /* 当MOE=0，OCxN=0 */
    HAL_TIM_PWM_ConfigChannel(&g_tim1_cplm_pwm_handle, &tim_oc_cplm_pwm, ATIM_TIM1_CPLM_CH1);
    HAL_TIM_PWM_ConfigChannel(&g_tim1_cplm_pwm_handle, &tim_oc_cplm_pwm, ATIM_TIM1_CPLM_CH2);
    
    /* 设置死区参数，开启死区中断 */
    g_sbreak1_dead_time1_config.OffStateRunMode = TIM_OSSR_DISABLE;           /* 运行模式的关闭输出状态 */
    g_sbreak1_dead_time1_config.OffStateIDLEMode = TIM_OSSI_DISABLE;          /* 空闲模式的关闭输出状态 */
    g_sbreak1_dead_time1_config.LockLevel = TIM_LOCKLEVEL_OFF;                /* 不用寄存器锁功能 */
    g_sbreak1_dead_time1_config.BreakState = TIM_BREAK_ENABLE;                /* 使能刹车输入 */
    g_sbreak1_dead_time1_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;       /* 刹车输入有效信号极性为高 */
    g_sbreak1_dead_time1_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; /* 使能AOE位，允许刹车结束后自动恢复输出 */
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim1_cplm_pwm_handle, &g_sbreak1_dead_time1_config);

    HAL_TIM_PWM_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);         /* OC1 输出使能 */
    HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);      /* OC1N 输出使能 */
    
    HAL_TIM_PWM_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);         /* OC2 输出使能 */
    HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);      /* OC2N 输出使能 */                       /* 开启对应PWM通道 */
    LEFT_SOFTBRAKE(0); //软刹车停止
}

/**
 * @brief       定时器TIMX 设置输出比较值 & 死区时间
 * @param       ccr: 输出比较值
 * @param       dtg: 死区时间
 *   @arg       dtg[7:5]=0xx时, 死区时间 = dtg[7:0] * tDTS
 *   @arg       dtg[7:5]=10x时, 死区时间 = (64 + dtg[6:0]) * 2  * tDTS
 *   @arg       dtg[7:5]=110时, 死区时间 = (32 + dtg[5:0]) * 8  * tDTS
 *   @arg       dtg[7:5]=111时, 死区时间 = (32 + dtg[5:0]) * 16 * tDTS
 *   @note      tDTS = 1 / (Ft /  CKD[1:0]) = 1 / 42M = 23.8ns
 * @retval      无
 * @return        {*}
**/
void atim_tim1_cplm_pwm_set(uint16_t ccr1,uint16_t ccr2)
{
    g_sbreak1_dead_time1_config.DeadTime = 50;
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim1_cplm_pwm_handle, &g_sbreak1_dead_time1_config);      /*重设死区时间*/
    __HAL_TIM_MOE_ENABLE(&g_tim1_cplm_pwm_handle);      /* MOE=1,使能主输出 */    
    ATIM_TIM1_CPLM_CH1_CCR1 = ccr1;                      /* 设置比较寄存器 */
    ATIM_TIM1_CPLM_CH2_CCR2 = ccr2;  
}
void LeftMoterMove(uint8_t dir,double duty_cycle)
{
    LEFT_SOFTBRAKE(0); // 关闭软刹车
    if (dir==1)
    {
        atim_tim1_cplm_pwm_set((uint16_t)(duty_cycle*100),0);
    }
    else if(dir==0)
    {
        atim_tim1_cplm_pwm_set(0,(uint16_t)(duty_cycle*100));
    }
    else
    {
        LeftMoterStop();   
    }
}

void LeftMoterStop(void)
{
    atim_tim1_cplm_pwm_set(0,0);
    // LEFT_SOFTBRAKE(1);
}
/**********************************底盘R 电机2驱动输出 TIME8 CH1 CH2 CH1N CH2N*************************************/
TIM_HandleTypeDef g_tim8_cplm_pwm_handle;                              /* 定时器1句柄 */
TIM_BreakDeadTimeConfigTypeDef g_sbreak2_dead_time8_config = {0};        /* 死区时间设置 */
void MoterR_pwm_chy_init(uint16_t arr, uint16_t psc) // 右轮电机
{
    /*右轮电机*/
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_cplm_pwm = {0};

    ATIM_TIM8_CPLM_CLK_ENABLE();            /* TIMx 时钟使能 */
    ATIM_TIM8_CPLM_CH1_GPIO_CLK_ENABLE();   /* 通道X对应IO口时钟使能 */
    ATIM_TIM8_CPLM_CH1N_GPIO_CLK_ENABLE();  /* 通道X互补通道对应IO口时钟使能 */
    ATIM_TIM8_CPLM_CH2_GPIO_CLK_ENABLE();   /* 通道X对应IO口时钟使能 */
    ATIM_TIM8_CPLM_CH2N_GPIO_CLK_ENABLE();  /* 通道X互补通道对应IO口时钟使能 */
    
    ATIM_TIM8_CPLM_BKIN_GPIO_CLK_ENABLE();  /* 通道X刹车输入对应IO口时钟使能 */
    gpio_init_struct.Pin = ATIM_TIM8_CPLM_BKIN_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = ATIM_TIM8_CPLM_CHY_GPIO_AF;                /* 端口复用 */

    gpio_init_struct.Pin = ATIM_TIM8_CPLM_CH1_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM8_CPLM_CH1_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ATIM_TIM8_CPLM_CH1N_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM8_CPLM_CH1N_GPIO_PORT, &gpio_init_struct);
    
    gpio_init_struct.Pin = ATIM_TIM8_CPLM_CH2_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM8_CPLM_CH2_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ATIM_TIM8_CPLM_CH2N_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM8_CPLM_CH2N_GPIO_PORT, &gpio_init_struct);

    g_tim8_cplm_pwm_handle.Instance = ATIM_TIM8_CPLM;                       /* 定时器x */
    g_tim8_cplm_pwm_handle.Init.Prescaler = psc;                            /* 预分频系数 */
    g_tim8_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;           /* 向上计数模式 */
    g_tim8_cplm_pwm_handle.Init.Period = arr;                               /* 自动重装载值 */
    g_tim8_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;     /* CKD[1:0] = 10, tDTS = 4 * tCK_INT = Ft / 4 = 42Mhz*/
    g_tim8_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  /* 使能影子寄存器TIMx_ARR */
  
    HAL_TIM_PWM_Init(&g_tim8_cplm_pwm_handle) ;

    tim_oc_cplm_pwm.OCMode = TIM_OCMODE_PWM1;                               /* PWM模式1 */
    tim_oc_cplm_pwm.OCPolarity = TIM_OCPOLARITY_HIGH;                       /* OCy 高电平有效 */
    tim_oc_cplm_pwm.OCNPolarity = TIM_OCPOLARITY_HIGH;                      /* OCyN 高电平有效 */
    tim_oc_cplm_pwm.OCIdleState = TIM_OCIDLESTATE_RESET;                      /* 当MOE=0，OCx=0 */
    tim_oc_cplm_pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;                    /* 当MOE=0，OCxN=0 */
    HAL_TIM_PWM_ConfigChannel(&g_tim8_cplm_pwm_handle, &tim_oc_cplm_pwm, ATIM_TIM8_CPLM_CH1);
    HAL_TIM_PWM_ConfigChannel(&g_tim8_cplm_pwm_handle, &tim_oc_cplm_pwm, ATIM_TIM8_CPLM_CH2);
    
    /* 设置死区参数，开启死区中断 */
    g_sbreak2_dead_time8_config.OffStateRunMode = TIM_OSSR_DISABLE;           /* 运行模式的关闭输出状态 */
    g_sbreak2_dead_time8_config.OffStateIDLEMode = TIM_OSSI_DISABLE;          /* 空闲模式的关闭输出状态 */
    g_sbreak2_dead_time8_config.LockLevel = TIM_LOCKLEVEL_OFF;                /* 不用寄存器锁功能 */
    g_sbreak2_dead_time8_config.BreakState = TIM_BREAK_ENABLE;                /* 使能刹车输入 */
    g_sbreak2_dead_time8_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;       /* 刹车输入有效信号极性为高 */
    g_sbreak2_dead_time8_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; /* 使能AOE位，允许刹车结束后自动恢复输出 */
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim8_cplm_pwm_handle, &g_sbreak2_dead_time8_config);

    HAL_TIM_PWM_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH1);         /* OC1 输出使能 */
    HAL_TIMEx_PWMN_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH1);      /* OC1N 输出使能 */
    
    HAL_TIM_PWM_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH2);         /* OC2 输出使能 */
    HAL_TIMEx_PWMN_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH2);      /* OC2N 输出使能 */   
    RIGHT_SOFTBRAKE(0);//软刹车停止
}
void atim_tim8_cplm_pwm_set(uint16_t ccr1,uint16_t ccr2)
{
    g_sbreak2_dead_time8_config.DeadTime = 50;
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim8_cplm_pwm_handle, &g_sbreak2_dead_time8_config);      /*重设死区时间*/
    __HAL_TIM_MOE_ENABLE(&g_tim8_cplm_pwm_handle);      /* MOE=1,使能主输出 */    
    ATIM_TIM8_CPLM_CH1_CCR1 = ccr1;                      /* 设置比较寄存器 */
    ATIM_TIM8_CPLM_CH2_CCR2 = ccr2;  
}

void RightMoterMove(uint8_t dir,double duty_cycle)
{
    RIGHT_SOFTBRAKE(0);
    if (dir==1)
    {
        atim_tim8_cplm_pwm_set((uint16_t)(duty_cycle*100),0);
    }
    else if(dir==0)
    {
        atim_tim8_cplm_pwm_set(0,(uint16_t)(duty_cycle*100));
    }
    else
    {
        RightMoterStop();   
    }
}

void RightMoterStop(void)
{
    atim_tim8_cplm_pwm_set(0,0);
    // RIGHT_SOFTBRAKE(1);
}

/**********************************靠背角度撑杆A1*************************************/

void Moterbackrest_pwm_chy_init(uint16_t arr, uint16_t psc)
{

    TIM_OC_InitTypeDef tim1_oc_pwm_chy = {0};                                                /* 定时器9输出句柄 */
    g_time1_pwm_chy_handle.Instance = GTIM_TIM1_PWM;                                         /* 定时器x */
    g_time1_pwm_chy_handle.Init.Prescaler = psc;                                             /* 预分频系数 */
    g_time1_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;                            /* 递增计数模式 */
    g_time1_pwm_chy_handle.Init.Period = arr;                                                /* 自动重装载值 */
    HAL_TIM_PWM_Init(&g_time1_pwm_chy_handle);                                               /* 初始化PWM */
    tim1_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;                                                /* 模式选择PWM1 */
    tim1_oc_pwm_chy.Pulse = 0;                                                               /* 设置比较值,此值用来确定占空比 */
    tim1_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_LOW;                                         /* 输出比较极性为低 */
    HAL_TIM_PWM_ConfigChannel(&g_time1_pwm_chy_handle, &tim1_oc_pwm_chy, GTIM_TIM1_PWM_CH3); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH3);                           /* 开启对应PWM通道 */
    HAL_TIM_PWM_ConfigChannel(&g_time1_pwm_chy_handle, &tim1_oc_pwm_chy, GTIM_TIM1_PWM_CH4); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH4);                           /* 开启对应PWM通道 */
}
/**********************************座板角度撑杆B2*************************************/

void MoterPedestal_pwm_chy_init(uint16_t arr, uint16_t psc) // 电动推杆 2&3
{
    /*推杆电机2（座板）*/
    TIM_OC_InitTypeDef tim8_oc_pwm_chy = {0};                                                /* 定时器9输出句柄 */
    g_time8_pwm_chy_handle.Instance = GTIM_TIM8_PWM;                                         /* 定时器x */
    g_time8_pwm_chy_handle.Init.Prescaler = psc;                                             /* 预分频系数 */
    g_time8_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;                            /* 递增计数模式 */
    g_time8_pwm_chy_handle.Init.Period = arr;                                                /* 自动重装载值 */
    HAL_TIM_PWM_Init(&g_time8_pwm_chy_handle);                                               /* 初始化PWM */
    tim8_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;                                                /* 模式选择PWM1 */
    tim8_oc_pwm_chy.Pulse = 0;                                                               /* 设置比较值,此值用来确定占空比 */
    tim8_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_LOW;                                         /* 输出比较极性为低 */
    HAL_TIM_PWM_ConfigChannel(&g_time8_pwm_chy_handle, &tim8_oc_pwm_chy, GTIM_TIM8_PWM_CH3); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH3);                           /* 开启对应PWM通道 */
    HAL_TIM_PWM_ConfigChannel(&g_time8_pwm_chy_handle, &tim8_oc_pwm_chy, GTIM_TIM8_PWM_CH4); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH4);                           /* 开启对应PWM通道 */

}
/**********************************底盘举升撑杆B1(M)*************************************/
void MoterLift_pwm_chy_init(uint16_t arr, uint16_t psc) 
{

    TIM_OC_InitTypeDef tim2_oc_pwm_chy = {0};                                                /* 定时器9输出句柄 */
    g_time2_pwm_chy_handle.Instance = GTIM_TIM2_PWM;                                         /* 定时器x */
    g_time2_pwm_chy_handle.Init.Prescaler = psc;                                             /* 预分频系数 */
    g_time2_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;                            /* 递增计数模式 */
    g_time2_pwm_chy_handle.Init.Period = arr;                                                /* 自动重装载值 */
    HAL_TIM_PWM_Init(&g_time2_pwm_chy_handle);                                               /* 初始化PWM */
    tim2_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;                                                /* 模式选择PWM1 */
    tim2_oc_pwm_chy.Pulse = 0;                                                               /* 设置比较值,此值用来确定占空比 */
    tim2_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_LOW;                                         /* 输出比较极性为低 */
    HAL_TIM_PWM_ConfigChannel(&g_time2_pwm_chy_handle, &tim2_oc_pwm_chy, GTIM_TIM2_PWM_CH3); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH3);                           /* 开启对应PWM通道 */
    HAL_TIM_PWM_ConfigChannel(&g_time2_pwm_chy_handle, &tim2_oc_pwm_chy, GTIM_TIM2_PWM_CH4); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH4);         
}
/**********************************腿托角度撑杆A2 (L)、腿托长度撑杆A3(L)*************************************/

void MoterLeg_pwm_chy_init(uint16_t arr, uint16_t psc)
{
    // 腿托角度撑杆A2
    TIM_OC_InitTypeDef tim4_oc_pwm_chy = {0};                                                /* 定时器9输出句柄 */
    g_time4_pwm_chy_handle.Instance = GTIM_TIM4_PWM;                                         /* 定时器x */
    g_time4_pwm_chy_handle.Init.Prescaler = psc;                                             /* 预分频系数 */
    g_time4_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;                            /* 递增计数模式 */
    g_time4_pwm_chy_handle.Init.Period = arr;                                                /* 自动重装载值 */
    HAL_TIM_PWM_Init(&g_time4_pwm_chy_handle);                                               /* 初始化PWM */
    tim4_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;                                                /* 模式选择PWM1 */
    tim4_oc_pwm_chy.Pulse = 0;                                                               /* 设置比较值,此值用来确定占空比 */
    tim4_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_LOW;                                         /* 输出比较极性为低 */
    HAL_TIM_PWM_ConfigChannel(&g_time4_pwm_chy_handle, &tim4_oc_pwm_chy, GTIM_TIM4_PWM_CH3); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH3);                           /* 开启对应PWM通道 */
    HAL_TIM_PWM_ConfigChannel(&g_time4_pwm_chy_handle, &tim4_oc_pwm_chy, GTIM_TIM4_PWM_CH4); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH4);                           /* 开启对应PWM通道 */
    /* 推杆5 腿托长度撑杆A3*/
    HAL_TIM_PWM_ConfigChannel(&g_time4_pwm_chy_handle, &tim4_oc_pwm_chy, GTIM_TIM4_PWM_CH1); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH1);                           /* 开启对应PWM通道 */
    HAL_TIM_PWM_ConfigChannel(&g_time4_pwm_chy_handle, &tim4_oc_pwm_chy, GTIM_TIM4_PWM_CH2); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH2);                           /* 开启对应PWM通道 */
}
/**********************************前支撑轮推杆C1*************************************/
void MoterSupport_pwm_chy_init(uint16_t arr, uint16_t psc) // 电动推杆 6
{
    
    /*推杆电机6 前支撑*/
    TIM_OC_InitTypeDef tim12_oc_pwm_chy = {0};                                                  /* 定时器12输出句柄 */
    g_time12_pwm_chy_handle.Instance = GTIM_TIM12_PWM;                                          /* 定时器x */
    g_time12_pwm_chy_handle.Init.Prescaler = psc;                                               /* 预分频系数 */
    g_time12_pwm_chy_handle.Init.CounterMode = TIM_COUNTERMODE_UP;                              /* 递增计数模式 */
    g_time12_pwm_chy_handle.Init.Period = arr;                                                  /* 自动重装载值 */
    HAL_TIM_PWM_Init(&g_time12_pwm_chy_handle);                                                 /* 初始化PWM */
    tim12_oc_pwm_chy.OCMode = TIM_OCMODE_PWM1;                                                  /* 模式选择PWM1 */
    tim12_oc_pwm_chy.Pulse = 0;                                                                 /* 设置比较值,此值用来确定占空比 */
    tim12_oc_pwm_chy.OCPolarity = TIM_OCPOLARITY_LOW;                                           /* 输出比较极性为低 */
    HAL_TIM_PWM_ConfigChannel(&g_time12_pwm_chy_handle, &tim12_oc_pwm_chy, GTIM_TIM12_PWM_CH1); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH1);                            /* 开启对应PWM通道 */
    HAL_TIM_PWM_ConfigChannel(&g_time12_pwm_chy_handle, &tim12_oc_pwm_chy, GTIM_TIM12_PWM_CH2); /* 配置TIMx通道y */
    HAL_TIM_PWM_Start(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH2);                            /* 开启对应PWM通道 */
}

/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
                此函数会被HAL_TIM_PWM_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == GTIM_TIM2_PWM)
    {  //举升
        GPIO_InitTypeDef gpio_init_structB;
        GTIM_TIM2_PWM_CHY_GPIOB_CLK_ENABLE();                                              /* 开启通道y的GPIO时钟 */
        GTIM_TIM2_PWM_CHY_CLK_ENABLE();                                                    /* 使能定时器时钟 */
        gpio_init_structB.Pin = GTIM_TIM2_PWM_CHY_GPIO_PIN10 | GTIM_TIM2_PWM_CHY_GPIO_PIN11; /* 通道1 2的CPIO口 */
        gpio_init_structB.Mode = GPIO_MODE_AF_PP;                                          /* 复用推完输出 */
        gpio_init_structB.Pull = GPIO_PULLUP;                                              /* 上拉 */
        gpio_init_structB.Speed = GPIO_SPEED_FREQ_HIGH;                                    /* 高速 */
        gpio_init_structB.Alternate = GTIM_TIM2_PWM_CHY_GPIO_AF;                           /* IO口REMAP设置, 是否必要查看头文件配置的说明! */
        HAL_GPIO_Init(GTIM_TIM2_PWM_CHY_GPIO_PORTB, &gpio_init_structB);
    }

    if (htim->Instance == GTIM_TIM1_PWM)
    {
        //靠背
        GPIO_InitTypeDef gpio_init_structA;
        GTIM_TIM1_PWM_CHY_GPIOA_CLK_ENABLE();                                                /* 开启通道y的GPIO时钟 */
        GTIM_TIM1_PWM_CHY_CLK_ENABLE();                                                      /* 使能定时器时钟 */
        gpio_init_structA.Pin = GTIM_TIM1_PWM_CHY_GPIO_PIN10 | GTIM_TIM1_PWM_CHY_GPIO_PIN11; /* 通道1 2的CPIO口 */
        gpio_init_structA.Mode = GPIO_MODE_AF_PP;                                            /* 复用推完输出 */
        gpio_init_structA.Pull = GPIO_PULLUP;                                                /* 上拉 */
        gpio_init_structA.Speed = GPIO_SPEED_FREQ_HIGH;                                      /* 高速 */
        gpio_init_structA.Alternate = GTIM_TIM1_PWM_CHY_GPIO_AF;                             /* IO口REMAP设置, 是否必要查看头文件配置的说明! */
        HAL_GPIO_Init(GTIM_TIM1_PWM_CHY_GPIO_PORTA, &gpio_init_structA);
    }

    if (htim->Instance == GTIM_TIM8_PWM)
    {
        // 座板
        GPIO_InitTypeDef gpio_init_structC;
        GTIM_TIM8_PWM_CHY_GPIOC_CLK_ENABLE();                                                                                                          /* 开启通道y的GPIO时钟 */
        GTIM_TIM8_PWM_CHY_CLK_ENABLE();                                                                                                                /* 使能定时器时钟 */
        gpio_init_structC.Pin = GTIM_TIM8_PWM_CHY_GPIO_PIN8 | GTIM_TIM8_PWM_CHY_GPIO_PIN9 ; /* 通道3 4的CPIO口 */
        gpio_init_structC.Mode = GPIO_MODE_AF_PP;                                                                                                      /* 复用推完输出 */
        gpio_init_structC.Pull = GPIO_PULLUP;                                                                                                          /* 上拉 */
        gpio_init_structC.Speed = GPIO_SPEED_FREQ_HIGH;                                                                                                /* 高速 */
        gpio_init_structC.Alternate = GTIM_TIM8_PWM_CHY_GPIO_AF;                                                                                       /* IO口REMAP设置, 是否必要查看头文件配置的说明! */
        HAL_GPIO_Init(GTIM_TIM8_PWM_CHY_GPIO_PORTC, &gpio_init_structC);
    }

    if (htim->Instance == GTIM_TIM4_PWM)
    {
        // 推杆4&5
        GPIO_InitTypeDef gpio_init_structD;
        GTIM_TIM4_PWM_CHY_GPIOD_CLK_ENABLE();                                                                                                              /* 开启通道y的GPIO时钟 */
        GTIM_TIM4_PWM_CHY_CLK_ENABLE();                                                                                                                    /* 使能定时器时钟 */
        gpio_init_structD.Pin = GTIM_TIM4_PWM_CHY_GPIO_PIN12 | GTIM_TIM4_PWM_CHY_GPIO_PIN13 | GTIM_TIM4_PWM_CHY_GPIO_PIN14 | GTIM_TIM4_PWM_CHY_GPIO_PIN15; /* 通道1 2的CPIO口 */
        gpio_init_structD.Mode = GPIO_MODE_AF_PP;                                                                                                          /* 复用推完输出 */
        gpio_init_structD.Pull = GPIO_PULLUP;                                                                                                              /* 上拉 */
        gpio_init_structD.Speed = GPIO_SPEED_FREQ_HIGH;                                                                                                    /* 高速 */
        gpio_init_structD.Alternate = GTIM_TIM4_PWM_CHY_GPIO_AF;                                                                                           /* IO口REMAP设置, 是否必要查看头文件配置的说明! */
        HAL_GPIO_Init(GTIM_TIM4_PWM_CHY_GPIO_PORTD, &gpio_init_structD);
    }

    if (htim->Instance == GTIM_TIM12_PWM)
    {
        // 推杆6
        GPIO_InitTypeDef gpio_init_structB;
        GTIM_TIM12_PWM_CHY_GPIOB_CLK_ENABLE();                                                 /* 开启通道y的GPIO时钟 */
        GTIM_TIM12_PWM_CHY_CLK_ENABLE();                                                       /* 使能定时器时钟 */
        gpio_init_structB.Pin = GTIM_TIM12_PWM_CHY_GPIO_PIN14 | GTIM_TIM12_PWM_CHY_GPIO_PIN15; /* 通道1 2的CPIO口 */
        gpio_init_structB.Mode = GPIO_MODE_AF_PP;                                              /* 复用推完输出 */
        gpio_init_structB.Pull = GPIO_PULLUP;                                                  /* 上拉 */
        gpio_init_structB.Speed = GPIO_SPEED_FREQ_HIGH;                                        /* 高速 */
        gpio_init_structB.Alternate = GTIM_TIM12_PWM_CHY_GPIO_AF;                              /* IO口REMAP设置, 是否必要查看头文件配置的说明! */
        HAL_GPIO_Init(GTIM_TIM12_PWM_CHY_GPIO_PORTB, &gpio_init_structB);
    }   
}

void MoterdriveInit(void)
{
    MoterL_pwm_chy_init(100 - 1, 84 - 1);                  //* 168 000 000 / 100*84   TIME1   L 20khz频率的PWM 波形*  /
    MoterR_pwm_chy_init(100 - 1, 84 - 1);                  //* 168 000 000 / 100*84   TIME8   R  20khz频率的PWM 波形*/
    Moterbackrest_pwm_chy_init(100 - 1, 84 - 1);           //* 168 000 000 / 100*84   TIME1   靠背 20khz频率的PWM 波形*  /
    MoterPedestal_pwm_chy_init(100 - 1, 84 - 1);           //* 168 000 000 / 100*84   TIME8   座板 20khz频率的PWM 波形*  /
    MoterLift_pwm_chy_init(100-1,42-1);                     //* 84 000 000 / 100*42    TIME2   举升 20khz频率的PWM 波形*  /
    MoterLeg_pwm_chy_init(100-1,42-1);                      //* 84 000 000 / 100*42    TIME4   腿托长度+角度 20khz频率的PWM 波形*  /
    MoterSupport_pwm_chy_init(100-1,42-1);                  //* 84 000 000 / 100*42    TIME12  支撑轮 20khz频率的PWM 波形*  /
}
// 推杆测试程序
void linearactuatorTest(void)
{
    
    linearactuator();
}

void linearactuator(void)
{
	/*
	A1 : 靠背
	A2:  腿托角度
	A3:  腿托长度
	B1:  举升
	B2:  座板角度
	C1:  前支撑轮
	*/
    uint16_t T1_IN1 = 0;
    uint16_t T1_IN2 = 0;

    uint16_t T2_IN1 = 0;
    uint16_t T2_IN2 = 0;

    uint16_t T3_IN1 = 0;
    uint16_t T3_IN2 = 0;

    uint16_t T4_IN1 = 0;
    uint16_t T4_IN2 = 0;

    uint16_t T5_IN1 = 0;
    uint16_t T5_IN2 = 0;

    uint16_t T6_IN1 = 0;
    uint16_t T6_IN2 = 0;
    // 先屏限位用于调试标定
    ActorLimitPara.A1_Downpos =50;
    ActorLimitPara.A1_Uppos = 2800; // 前倾已经超出限位计，但是机械上暂时不约束;

    ActorLimitPara.A2_Downpos = 32;
    ActorLimitPara.A2_Uppos = 3000; // 上旋已经超出限位计长度，机械上暂时不约束

    ActorLimitPara.A3_Downpos = 0; // 下限位无约束
    ActorLimitPara.A3_Uppos = 2600;

    ActorLimitPara.B1_Downpos = 130;
    ActorLimitPara.B1_Uppos = 1300;

    ActorLimitPara.B2_Downpos = 50;
    ActorLimitPara.B2_Uppos = 2900;

    ActorLimitPara.C1_Uppos = 1200;
    ActorLimitPara.C2_Downpos = 0;
    
    static float acctemp = 0, acct = 0;
    static uint8_t accdoneflage = 0;
    static float Kp;

    Kp = B1MAX * SEAT_LIFTDROPRATIO / 2.0 + 0.5;

    if (CanKeybufReceive[1]!= 0 ) /*按键板有按键按下,需要远程打断远程控车的指令*/
    {
        g_slaveReg[21] = 1 ;
        g_slaveReg[79]=0;
        g_slaveReg[80]=0;
    }
    else
    {
        g_slaveReg[21] = 0 ;
    }

    /*如果上下位机通讯失败则下位机主动复位上位机姿态操作指令*/
    /*如果上位机在操作座椅则按键板需要能打断上位机的指令*/  
    /*驾驶过程中或充电过程中不允许调整座椅--清除所有座椅控制指令*/
    if(comheartstate.com_state == Fail|| CanKeybufReceive[1]!= 0 || g_slaveReg[2]==1)  //g_slaveReg[10] || g_slaveReg[11]||
    {
        g_slaveReg[66] = 0;
        g_slaveReg[67] = 0;
        g_slaveReg[68] = 0;
        g_slaveReg[69] = 0; 
        g_slaveReg[70] = 0;
        g_slaveReg[71] = 0; 
        g_slaveReg[72] = 0;
    }
    /*驾驶过程中不允许操作座椅*/
//    if ( g_slaveReg[10] || g_slaveReg[11])
//    {
//        CanKeybufReceive[1] = 0;
       
//    }

    /*座椅状态判断*/
    if(adcdata.lift_pos>=376 && adcdata.lift_pos<=576&&adcdata.pedestal_pos>=2690&&adcdata.pedestal_pos<=2890&&adcdata.pedestal_pos>=227&&adcdata.backboard_pos <=327&&adcdata.legangle_pos >=2427&&adcdata.legangle_pos<= 2487&&adcdata.leglength_pos>=483&&adcdata.leglength_pos<=683)
    {
        liner_state = Stand_state;
    }
    else
    {
        liner_state = None_Stand_state;
    }
    /*座椅举升控制*/
    if ((CanKeybufReceive[1] == SEAT_LIFT || g_slaveReg[67] == 1) && liner_state!=Stand_state )
    {
        linerun_cmd = Lift_cmd;
        g_slaveReg[16] = 1;
    }
    else if ((CanKeybufReceive[1] == SEAT_DROP || g_slaveReg[67] == 2) && liner_state!=Stand_state)
    {

        linerun_cmd = Down_cmd;
        g_slaveReg[16] = 2;
    }

    /* 座椅靠背控制*/
    else if (CanKeybufReceive[1] == BACKREST_FORWARD || g_slaveReg[68] == 1)
    {
        linerun_cmd = Backf_cmd;
        g_slaveReg[17] = 1;
    }
    
    
    else if (CanKeybufReceive[1] == BACKREST_BACK || g_slaveReg[68] == 2)
        
    {
        linerun_cmd = Backb_cmd;
        g_slaveReg[17] = 2;
        
    }
    /*座板控制*/
    else if (g_slaveReg[72] == 1 && liner_state!=Stand_state)
    {
        linerun_cmd = Seat_tiltf_cmd;
    }
    else if (g_slaveReg[72] == 2)
    {
        linerun_cmd = Seat_tiltb_cmd;
    }

    /*整体前后倾*/
    else if ((CanKeybufReceive[1] == ALL_FORWARD || g_slaveReg[69] == 1)&& liner_state!=Stand_state)
    {

        linerun_cmd = Alltiltfrun_cmd;
        g_slaveReg[19] = 1;
    }

    else if (CanKeybufReceive[1] == 8 || g_slaveReg[69] == 2)
    {

        linerun_cmd = Alltiltbrun_cmd;
        g_slaveReg[19] = 2;
    }
    /*腿托上下旋转*/
    else if ((CanKeybufReceive[1] == LEG_TOPSPIN || g_slaveReg[70] == 1)&& liner_state!=Stand_state)
    {

        linerun_cmd = Legspintop_cmd;
        g_slaveReg[20] = 1;
    }
    else if ((CanKeybufReceive[1] == LEG_BACKSPIN || g_slaveReg[70] == 2)&& liner_state!=Stand_state)
    {
        linerun_cmd = Legspindown_cmd;
        g_slaveReg[20] = 2;
    }
    /*腿托独立调节长度*/
    else if ((g_slaveReg[71] == 2)&& liner_state!=Stand_state)
    {
        linerun_cmd = Legexten_cmd;
    }

    else if ((g_slaveReg[71] == 1)&& liner_state!=Stand_state)
    {
        linerun_cmd = Legunexten_cmd;
    }
    /*一键站立/坐下*/
    else if (CanKeybufReceive[1] == STANCE || g_slaveReg[66] == 1)
    {

        linerun_cmd = Stand_cmd;
        g_slaveReg[18] = 1;
    }

    else if (CanKeybufReceive[1] == SITTING || g_slaveReg[66] == 2)

    {
        linerun_cmd = Site_cmd;
        g_slaveReg[18] = 2;
    }
    else
    {
        linerun_cmd = iddle_cmd;
        g_slaveReg[16] = 0;
        g_slaveReg[17] = 0;
        g_slaveReg[18] = 0;
        g_slaveReg[19] = 0;
        g_slaveReg[20] = 0;

    }
    /*充电过程中 或者防夹触发不可操作轮椅*/
    if(g_slaveReg[2]==1 || g_slaveReg[74] == 1)
    {
        linerun_cmd = iddle_cmd; 
    }
    
    switch (linerun_cmd)
    {
    case iddle_cmd:
        T1_IN1 = 100;
        T1_IN2 = 100;
        T2_IN1 = 100;
        T2_IN2 = 100;
        T3_IN1 = 100;
        T3_IN2 = 100;
        T4_IN1 = 100;
        T4_IN2 = 100;
        T5_IN1 = 100;
        T5_IN2 = 100;
        T6_IN1 = 100;
        T6_IN2 = 100;
        acct = 0;
        accdoneflage = 0;
        acctemp = 0;
        // 靠背角度撑杆A1
        __HAL_TIM_SET_COMPARE(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH3, T3_IN1);
        __HAL_TIM_SET_COMPARE(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH4, T3_IN2);
        // 腿托长度撑杆A2
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH1, T5_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH2, T5_IN2);
        // 腿托角度撑杆A3
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH3, T4_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH4, T4_IN2);
        // 底盘举升撑杆B1(M)
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH3, T1_IN1);
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH4, T1_IN2);
        // 座板角度撑杆B2(L)
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH3, T2_IN1);
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH4, T2_IN2); 
        // 前支撑轮推杆C1
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH1, T6_IN1);
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH2, T6_IN2);
       // printf("iddle_cmd");
        break;
    case Stand_cmd:
        if(chairkinematicspra.theta_x2X  >=10)
        {
            //下降
            T6_IN1 = 100 * (1.0 - 0.9);
            T6_IN2 = 100 * (1.0 - 0);
        }
        else 
        {
            //抬起
            T6_IN1 = 100 * (1.0 - 0);
            T6_IN2 = 100 * (1.0 - 0.9);
            // 抬起保护
            if(adcdata.support_pos >ActorLimitPara.C1_Uppos)
            {
                T6_IN1 = 100;
                T6_IN2 = 100;
            }
        }
        //腿托角度推杆目标位置2457： 2427 ~ 2487
        if(adcdata.legangle_pos < 2427 ) // 要伸长 
        {
            T4_IN1 = 100 * (1.0 - 0);
            T4_IN2 = 100 * (1.0 - 0.9);  
        }
        else if(adcdata.legangle_pos > 2487) // 要缩回
        {
            T4_IN1 = 100 * (1.0 - 0.9);
            T4_IN2 = 100 * (1.0 - 0); 
        }
        else                         // 要停止
        {
            T4_IN1 = 100;
            T4_IN2 = 100;
        }

        // 腿托旋转先到位再动其它
        if (adcdata.legangle_pos >=1000)
        {
            if ((acct < 200) && (accdoneflage == 0)) // 0 --- 0.8  200ms
            {
                acct++;
                acctemp = 6.56250000000002e-12 * pow(acct, 5) -3.281250000000008e-09 * pow(acct, 4) + 4.375000000000009e-07 * pow(acct, 3) + 0.5;
            }
            else
            {
                accdoneflage = 1;
                acct = 0;
            }
            //腿托长度推杆目标位置583： 483 ~ 683
            if(adcdata.leglength_pos < 483) // 要伸长 
            {
                T5_IN1 = 100 * (1.0 - 0);
                T5_IN2 = 100 * (1.0 - acctemp);  
            }
            else if(adcdata.leglength_pos > 683) // 要缩回
            {
                T5_IN1 = 100 * (1.0 - acctemp);
                T5_IN2 = 100 * (1.0 - 0); 
            }
            else                         // 要停止
            {
                T5_IN1 = 100;
                T5_IN2 = 100;
            }
            // 举升推杆目标位置476 ：   376~576
            if(adcdata.lift_pos < 376) // 要伸长 
            {
                T1_IN1 = 100 * (1.0 - 0);
                T1_IN2 = 100 * (1.0 - acctemp);   
            }
            else if(adcdata.lift_pos > 576) // 要缩回
            {
                T1_IN1 = 100 * (1.0 - acctemp);
                T1_IN2 = 100 * (1.0 - 0); 
            }
            else                         // 要停止
            {
                T1_IN1 = 100;
                T1_IN2 = 100;
            }

            // 座板推杆目标位置2791： 2690 ~ 2890
            if(adcdata.pedestal_pos < 2690) // 要伸长 
            {
                T2_IN1 = 100 * (1.0 - 0);
                T2_IN2 = 100 * (1.0 - acctemp);   
            }
            else if(adcdata.pedestal_pos > 2890) // 要缩回
            {
                T2_IN1 = 100 * (1.0 - acctemp);
                T2_IN2 = 100 * (1.0 - 0); 
            }
            else                         // 要停止
            {
                T2_IN1 = 100;
                T2_IN2 = 100;
            }
            // 靠背推杆目标位置277： 227 ~ 327
            if(adcdata.backboard_pos < 227) // 要伸长 
            {
                T3_IN1 = 100 * (1.0 - 0);
                T3_IN2 = 100 * (1.0 - acctemp);  
            }
            else if(adcdata.backboard_pos > 327) // 要缩回
            {
                T3_IN1 = 100 * (1.0 - acctemp);
                T3_IN2 = 100 * (1.0 - 0); 
            }
            else                         // 要停止
            {
                T3_IN1 = 100;
                T3_IN2 = 100;
            }
        }
        // 靠背角度撑杆A1
        __HAL_TIM_SET_COMPARE(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH3, T3_IN1);
        __HAL_TIM_SET_COMPARE(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH4, T3_IN2);
        // 腿托长度撑杆A2
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH1, T5_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH2, T5_IN2);
        // 腿托角度撑杆A3
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH3, T4_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH4, T4_IN2);
        // 底盘举升撑杆B1(M)
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH3, T1_IN1);
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH4, T1_IN2);
        // 座板角度撑杆B2(L)
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH3, T2_IN1);
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH4, T2_IN2); 
        // 前支撑轮推杆C1
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH1, T6_IN1);
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH2, T6_IN2);  
        break;
    case Site_cmd:
        if ((acct < 200) && (accdoneflage == 0)) // 0 --- 0.95  200ms
        {
            acct++;
            acctemp = 6.56250000000002e-12 * pow(acct, 5) -3.281250000000008e-09 * pow(acct, 4) + 4.375000000000009e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        if(chairkinematicspra.theta_x2X  >=10)
        {
            //下降
            T6_IN1 = 100 * (1.0 - 0.9);
            T6_IN2 = 100 * (1.0 - 0);
        }
        else 
        {
            //抬起
            T6_IN1 = 100 * (1.0 - 0);
            T6_IN2 = 100 * (1.0 - 0.9);
            // 抬起保护
            if(adcdata.support_pos >ActorLimitPara.C1_Uppos)
            {
                T6_IN1 = 100;
                T6_IN2 = 100;
            }
        }
    // 举升推杆目标位置111 ：   81~131
    if (adcdata.pedestal_pos < 500) // 座板放平其它推杆再运动
      {
        if(adcdata.lift_pos < 81) // 要伸长 
        {
            T1_IN1 = 100 * (1.0 - 0);
            T1_IN2 = 100 * (1.0 - 0.65);   
        }
        else if(adcdata.lift_pos > 131) // 要缩回
        {
            T1_IN1 = 100 * (1.0 - 0.65);
            T1_IN2 = 100 * (1.0 - 0); 
        }
        else                         // 要停止
        {
            T1_IN1 = 100;
            T1_IN2 = 100;
        }
      }

        // 座板推杆目标位置41： 11 ~ 71
        if(adcdata.pedestal_pos < 11) // 要伸长 
        {
            T2_IN1 = 100 * (1.0 - 0);
            T2_IN2 = 100 * (1.0 - acctemp);   
        }
        else if(adcdata.pedestal_pos > 71) // 要缩回
        {
            T2_IN1 = 100 * (1.0 - acctemp);
            T2_IN2 = 100 * (1.0 - 0); 
        }
        else                         // 要停止
        {
            T2_IN1 = 100;
            T2_IN2 = 100;
        }
        // 靠背推杆目标位置2400： 2350 ~ 2450
        if(adcdata.backboard_pos < 2350) // 要伸长 
        {
            T3_IN1 = 100 * (1.0 - 0);
            T3_IN2 = 100 * (1.0 -0.85*acctemp);  
        }
        else if(adcdata.backboard_pos > 2450) // 要缩回
        {
            T3_IN1 = 100 * (1.0 - 0.85*acctemp);
            T3_IN2 = 100 * (1.0 - 0); 
        }
        else                         // 要停止
        {
            T3_IN1 = 100;
            T3_IN2 = 100;
        }
    if (adcdata.pedestal_pos < 1200) 
    { 
        //腿托角度推杆目标位置215： 185 ~ 245
        if(adcdata.legangle_pos < 185 ) // 要伸长 
        {
            T4_IN1 = 100 * (1.0 - 0);
            T4_IN2 = 100 * (1.0 - 0.85);  
        }
        else if(adcdata.legangle_pos > 245) // 要缩回
        {
            T4_IN1 = 100 * (1.0 - 0.85);
            T4_IN2 = 100 * (1.0 - 0); 
        }
        else                         // 要停止
        {
            T4_IN1 = 100;
            T4_IN2 = 100;
        }
        //腿托长度推杆目标位置62： 32 ~ 92
        if(adcdata.leglength_pos < 1) // 要伸长 
        {
            T5_IN1 = 100 * (1.0 - 0);
            T5_IN2 = 100 * (1.0 - 0.9);  
        }
        else if(adcdata.leglength_pos > 112) // 要缩回
        {
            T5_IN1 = 100 * (1.0 - 0.9);
            T5_IN2 = 100 * (1.0 - 0); 
        }
        else                         // 要停止
        {
            T5_IN1 = 100;
            T5_IN2 = 100;
        }
    }
        // 靠背角度撑杆A1
        __HAL_TIM_SET_COMPARE(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH3, T3_IN1);
        __HAL_TIM_SET_COMPARE(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH4, T3_IN2);
        // 腿托长度撑杆A2
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH1, T5_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH2, T5_IN2);
        // 腿托角度撑杆A3
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH3, T4_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH4, T4_IN2);
        // 底盘举升撑杆B1(M)
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH3, T1_IN1);
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH4, T1_IN2);
        // 座板角度撑杆B2(L)
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH3, T2_IN1);
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH4, T2_IN2); 
        // 前支撑轮推杆C1
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH1, T6_IN1);
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH2, T6_IN2);

        break;


    case Lift_cmd:
        if ((acct < 200) && (accdoneflage == 0)) // 0 --- 0.95  200ms
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T1_IN1 = 100 * (1.0 - 0);
        T1_IN2 = 100 * (1.0 - acctemp);
        T2_IN1 = 100 * (1.0 - 0);
        T2_IN2 = 100 * (1.0 - Kp * acctemp);
        // 举升状态下的举升推杆上限位保护
        if ( adcdata.lift_pos  > ActorLimitPara.B1_Uppos)
        {
            T1_IN1 = 100;
            T1_IN2 = 100;
            /*举升推杆到上限时，座板推杆也必须停下来*/
            T2_IN1 = 100;
            T2_IN2 = 100;
        }
        // 举升状态下的座椅推杆上限位保护
        if (adcdata.pedestal_pos > ActorLimitPara.B2_Uppos)
        {
            T2_IN1 = 100;
            T2_IN2 = 100;
        }
        // 底盘举升撑杆B1(M)
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH3, T1_IN1);
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH4, T1_IN2);
        // 座板角度撑杆B2(L)
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH3, T2_IN1);
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH4, T2_IN2);
        break;

    case Down_cmd:
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T1_IN1 = 100 * (1.0 - Kp * acctemp);
        T1_IN2 = 100 * (1.0 - 0);
        T2_IN1 = 100 * (1.0 - acctemp);
        T2_IN2 = 100 * (1.0 - 0);
        // 举升推杆降低时 要保证正运动学的 y0 >-240 
        if ( adcdata.lift_pos  < ActorLimitPara.B1_Downpos || legkinematicspra.y_o < -240.0)
        {
            T1_IN1 = 100;
            T1_IN2 = 100;
        }
       // 座板推杆降低时 要保证正运动学的 y0 >-240 
        if (adcdata.pedestal_pos < ActorLimitPara.B2_Downpos|| legkinematicspra.y_o < -240.0)
        {
            T2_IN1 = 100;
            T2_IN2 = 100;
        }    
        // 底盘举升撑杆B1(M)
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH3, T1_IN1);
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH4, T1_IN2);
        // 座板角度撑杆B2(L)
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH3, T2_IN1);
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH4, T2_IN2);
        break;

    case Backf_cmd:
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T3_IN1 = 100 * (1.0 - 0);
        T3_IN2 = 100 * (1.0 - acctemp);
        /*前倾推杆位置约束*/
        if (adcdata.backboard_pos > ActorLimitPara.A1_Uppos)
        {
                T3_IN1 = 100;
                T3_IN2 = 100;
        }
        __HAL_TIM_SET_COMPARE(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH3, T3_IN1);
        __HAL_TIM_SET_COMPARE(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH4, T3_IN2);
        
        break;

    case Backb_cmd:
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T3_IN1 = 100 * (1.0 - acctemp);
        T3_IN2 = 100 * (1.0 - 0);
        /*后倾推杆位置约束*/
        if (adcdata.backboard_pos < ActorLimitPara.A1_Downpos)
        {
                T3_IN1 = 100;
                T3_IN2 = 100;
        }
        __HAL_TIM_SET_COMPARE(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH3, T3_IN1);
        __HAL_TIM_SET_COMPARE(&g_time1_pwm_chy_handle, GTIM_TIM1_PWM_CH4, T3_IN2);
        break;

    case Alltiltfrun_cmd:
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T1_IN1 = 100 * (1.0 - acctemp);
        T1_IN2 = 100 * (1.0 - 0);
        /*整体前倾约束*/
        if ( adcdata.lift_pos < ActorLimitPara.B1_Downpos|| legkinematicspra.y_o <-240|| legkinematicspra.x_o < 85)
        {
            T1_IN1 = 100;
            T1_IN2 = 100;
        }
        // 底盘举升撑杆B1(M)
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH3, T1_IN1);
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH4, T1_IN2);
        /*整体前后倾前支撑论保护*/
        if(chairkinematicspra.theta_x2X  >=10)
        {
            //下降
            T6_IN1 = 100 * (1.0 - 0.9);
            T6_IN2 = 100 * (1.0 - 0);
        }
        else 
        {
            //抬起
            T6_IN1 = 100 * (1.0 - 0);
            T6_IN2 = 100 * (1.0 - 0.9);
            // 抬起保护
            if(adcdata.support_pos >ActorLimitPara.C1_Uppos)
            {
                T6_IN1 = 100;
                T6_IN2 = 100;
            }
        }
        // 前支撑轮推杆C1
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH1, T6_IN1);
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH2, T6_IN2);
        break;

    case Alltiltbrun_cmd:
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T1_IN1 = 100 * (1.0 - 0);
        T1_IN2 = 100 * (1.0 - acctemp);
        /*整体后倾约束*/
        if ( adcdata.lift_pos  > ActorLimitPara.B1_Uppos)
        {
            T1_IN1 = 100;
            T1_IN2 = 100;
        }    
        // 底盘举升撑杆B1(M)
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH3, T1_IN1);
        __HAL_TIM_SET_COMPARE(&g_time2_pwm_chy_handle, GTIM_TIM2_PWM_CH4, T1_IN2);
        /*整体前后倾前支撑论保护*/
        if(chairkinematicspra.theta_x2X  >=10)
        {
            //下降
            T6_IN1 = 100 * (1.0 - 0.9);
            T6_IN2 = 100 * (1.0 - 0);
        }
        else 
        {
            //抬起
            T6_IN1 = 100 * (1.0 - 0);
            T6_IN2 = 100 * (1.0 - 0.9);
            // 抬起保护
            if(adcdata.support_pos >ActorLimitPara.C1_Uppos)
            {
                T6_IN1 = 100;
                T6_IN2 = 100;
            }
        }
            // 前支撑轮推杆C1
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH1, T6_IN1);
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH2, T6_IN2);
        break;

    case Legspintop_cmd:
        // 腿托上旋
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T4_IN1 = 100 * (1.0 - 0);
        T4_IN2 = 100 * (1.0 - acctemp);
        T5_IN1 = 100 * (1.0 - 0);
        T5_IN2 = 100 * (1.0 - acctemp);
        /*上旋腿托伸长约束*/
        if (adcdata.leglength_pos > ActorLimitPara.A3_Uppos)
        {
            T5_IN1 = 100;
            T5_IN2 = 100;
        }
        /*腿托旋转无约束*/
        // 腿托角度撑杆A3
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH3, T4_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH4, T4_IN2);
        // 腿托长度撑杆A2
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH1, T5_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH2, T5_IN2);
        break;
    case Legspindown_cmd:
        // 腿托下旋
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T4_IN1 = 100 * (1.0 - acctemp);
        T4_IN2 = 100 * (1.0 - 0);
        T5_IN1 = 100 * (1.0 - acctemp);
        T5_IN2 = 100 * (1.0 - 0);

        /*下旋腿托角度旋转约束*/
        if (adcdata.legangle_pos < ActorLimitPara.A2_Downpos || legkinematicspra.y_o <-240|| legkinematicspra.x_o < 85)
        {
            T4_IN1 = 100;

            T4_IN2 = 100;
        }
        /*腿托独立伸长量约束*/
        // if (adcdata.leglength_pos > ActorLimitPara.A3_Uppos)
        // {
        //     T5_IN1 = 100;
        //     T5_IN2 = 100;
        // }
        // 腿托角度撑杆A2
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH3, T4_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH4, T4_IN2);
        // 腿托长度撑杆A3
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH1, T5_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH2, T5_IN2);
        break;
    case Legexten_cmd:
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T5_IN1 = 100 * (1.0 - 0);
        T5_IN2 = 100 * (1.0 - acctemp);      
         /*腿托独立伸长量约束*/
        if (adcdata.leglength_pos > ActorLimitPara.A3_Uppos|| legkinematicspra.y_o <-240)
        {
            T5_IN1 = 100;
            T5_IN2 = 100;
        }
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH1, T5_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH2, T5_IN2);
        break;

    case Legunexten_cmd:
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }

        T5_IN1 = 100 * (1.0 - acctemp);
        T5_IN2 = 100 * (1.0 - 0);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH1, T5_IN1);
        __HAL_TIM_SET_COMPARE(&g_time4_pwm_chy_handle, GTIM_TIM4_PWM_CH2, T5_IN2);

        break;

    case Seat_tiltf_cmd:
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
             acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T2_IN1 = 100 * (1.0 - 0);
        T2_IN2 = 100 * (1.0 - acctemp);
        /*座板前倾约束*/
        if (adcdata.pedestal_pos > ActorLimitPara.B2_Uppos || legkinematicspra.y_o <-240 || legkinematicspra.x_o < 85 )
        {
            T2_IN1 = 100;
            T2_IN2 = 100;
        }
        // 座板角度撑杆B2(L)
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH3, T2_IN1);
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH4, T2_IN2);
        /*整体前后倾前支撑论保护*/
        if(chairkinematicspra.theta_x2X  >=10)
        {
            //下降
            T6_IN1 = 100 * (1.0 - 0.9);
            T6_IN2 = 100 * (1.0 - 0);
        }
        else 
        {
            //抬起
            T6_IN1 = 100 * (1.0 - 0);
            T6_IN2 = 100 * (1.0 - 0.9);
            // 抬起保护
            if(adcdata.support_pos >ActorLimitPara.C1_Uppos)
            {
                T6_IN1 = 100;
                T6_IN2 = 100;
            }
        }
            // 前支撑轮推杆C1
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH1, T6_IN1);
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH2, T6_IN2);
        break;
    case Seat_tiltb_cmd:
        if ((acct < 200) && (accdoneflage == 0))
        {
            acct++;
            acctemp = 8.437500000000000e-12 * pow(acct, 5) - 4.218750000000000e-09 * pow(acct, 4) + 5.625000000000000e-07 * pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        T2_IN1 = 100 * (1.0 - acctemp);
        T2_IN2 = 100 * (1.0 - 0);
        // 座板角度撑杆B2(L)
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH3, T2_IN1);
        __HAL_TIM_SET_COMPARE(&g_time8_pwm_chy_handle, GTIM_TIM8_PWM_CH4, T2_IN2);
            /*整体前后倾前支撑论保护*/
        if(chairkinematicspra.theta_x2X  >=10)
        {
            //下降
            T6_IN1 = 100 * (1.0 - 0.9);
            T6_IN2 = 100 * (1.0 - 0);
        }
        else 
        {
            //抬起
            T6_IN1 = 100 * (1.0 - 0);
            T6_IN2 = 100 * (1.0 - 0.9);
            // 抬起保护
            if(adcdata.support_pos >ActorLimitPara.C1_Uppos)
            {
                T6_IN1 = 100;
                T6_IN2 = 100;
            }
        }
            // 前支撑轮推杆C1
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH1, T6_IN1);
        __HAL_TIM_SET_COMPARE(&g_time12_pwm_chy_handle, GTIM_TIM12_PWM_CH2, T6_IN2);
        break;

    default:
        break;
    
    }
}

/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/第二阶段新项目/底盘闭环Demo板项目/R9_407F_num_2/Drivers/BSP/R9/moterdriver.c
 * @Description  :  
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2024-12-17 17:43:51
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
    
    // ATIM_TIM1_CPLM_BKIN_GPIO_CLK_ENABLE();  /* 通道X刹车输入对应IO口时钟使能 */
  
    // gpio_init_struct.Pin = ATIM_TIM1_CPLM_BKIN_GPIO_PIN;
    // gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    // gpio_init_struct.Pull = GPIO_PULLDOWN;
    // gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    // gpio_init_struct.Alternate = ATIM_TIM1_CPLM_CHY_GPIO_AF;                /* 端口复用 */
    // HAL_GPIO_Init(ATIM_TIM1_CPLM_BKIN_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ATIM_TIM1_CPLM_CH1_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = ATIM_TIM1_CPLM_CHY_GPIO_AF;     
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
    g_sbreak1_dead_time1_config.BreakState = TIM_BREAK_DISABLE;                /* 使能刹车输入 */
    g_sbreak1_dead_time1_config.BreakPolarity = TIM_BREAKPOLARITY_LOW;       /* 刹车输入有效信号极性为高，即高电平刹车 */
    g_sbreak1_dead_time1_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; /* 使能AOE位，允许刹车结束后自动恢复输出 */
    g_sbreak1_dead_time1_config.DeadTime = 50; 
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim1_cplm_pwm_handle, &g_sbreak1_dead_time1_config);

    HAL_TIM_PWM_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);         /* OC1 输出使能 */
    HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);      /* OC1N 输出使能 */
    
    HAL_TIM_PWM_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);         /* OC2 输出使能 */
    HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);      /* OC2N 输出使能 */                       /* 开启对应PWM通道 */
    // LEFT_SOFTBRAKE(0); //软刹车停止 ,高电平刹车；  
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
    // g_sbreak1_dead_time1_config.DeadTime = 50; // 50 即 dtg[7:0] 001  死区时间 = 50 *23.8 ns=1.19us
    // HAL_TIMEx_ConfigBreakDeadTime(&g_tim1_cplm_pwm_handle, &g_sbreak1_dead_time1_config);      /*重设死区时间*/
    __HAL_TIM_MOE_ENABLE(&g_tim1_cplm_pwm_handle);      /* MOE=1,使能主输出 */    
    ATIM_TIM1_CPLM_CH1_CCR1 = ccr1;                      /* 设置比较寄存器 */
    ATIM_TIM1_CPLM_CH2_CCR2 = ccr2;  
}

void LeftMoterMove(double duty_cycle, uint8_t islmoter_reverse)
{
    if (islmoter_reverse)
    {
        if (duty_cycle>0)
        {
            atim_tim1_cplm_pwm_set((uint16_t)(duty_cycle*100),0);  //向 后

        }
        else if(duty_cycle<0)
        {
            atim_tim1_cplm_pwm_set(0,(uint16_t)(-duty_cycle*100));//向 前
        }
        else
        {
            LeftMoterStop(); 

        }
    }
    else
    {
        if (duty_cycle>0)
        {
            atim_tim1_cplm_pwm_set(0,(uint16_t)(duty_cycle*100));  //向 前

        }
        else if(duty_cycle<0)
        {
            atim_tim1_cplm_pwm_set((uint16_t)(-duty_cycle*100),0);//向 后 转
        }
        else
        {
            LeftMoterStop(); 
        }
    }
}
void dcmotor_stop(void)
{
    HAL_TIM_PWM_Stop(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);         /* OC1 输出使能 */
    HAL_TIMEx_PWMN_Stop(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);      /* OC1N 输出使能 */
    
    HAL_TIM_PWM_Stop(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);         /* OC2 输出使能 */
    HAL_TIMEx_PWMN_Stop(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);      /* OC2N 输出使能 */                       /* 开启对应PWM通道 */
}

void dcmoter_dir(uint8_t para)
{
	
	dcmotor_stop();
	if (para==1) // 正转
	{
		HAL_TIM_PWM_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);         /* OC1 输出使能 */
        HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);      /* OC1N 输出使能 */
		HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);      /* OC2N 输出使能 */                       
		
		
	}
	else if (para==0) //反转
	{
		HAL_TIM_PWM_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);         /* OC2 输出使能 */
        HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);      /* OC2N 输出使能 */
		HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);      /* OC1N 输出使能 */                      
		
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
    
    // ATIM_TIM8_CPLM_BKIN_GPIO_CLK_ENABLE();  /* 通道X刹车输入对应IO口时钟使能 */
    // gpio_init_struct.Pin = ATIM_TIM8_CPLM_BKIN_GPIO_PIN;
    // gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    // gpio_init_struct.Pull = GPIO_PULLDOWN;
    // gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    // gpio_init_struct.Alternate = ATIM_TIM8_CPLM_CHY_GPIO_AF;                /* 端口复用 */

    gpio_init_struct.Pin = ATIM_TIM8_CPLM_CH1_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = ATIM_TIM8_CPLM_CHY_GPIO_AF;                /* 端口复用 */
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
    g_sbreak2_dead_time8_config.BreakState = TIM_BREAK_DISABLE;                /* 使能刹车输入 */
    g_sbreak2_dead_time8_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;       /* 刹车输入有效信号极性为高 */
    g_sbreak2_dead_time8_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; /* 使能AOE位，允许刹车结束后自动恢复输出 */
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim8_cplm_pwm_handle, &g_sbreak2_dead_time8_config);

    HAL_TIM_PWM_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH1);         /* OC1 输出使能 */
    HAL_TIMEx_PWMN_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH1);      /* OC1N 输出使能 */
    
    HAL_TIM_PWM_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH2);         /* OC2 输出使能 */
    HAL_TIMEx_PWMN_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH2);      /* OC2N 输出使能 */   
    // RIGHT_SOFTBRAKE(0);//软刹车停止
}
void atim_tim8_cplm_pwm_set(uint16_t ccr1,uint16_t ccr2)
{
    g_sbreak2_dead_time8_config.DeadTime = 50;
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim8_cplm_pwm_handle, &g_sbreak2_dead_time8_config);      /*重设死区时间*/
    __HAL_TIM_MOE_ENABLE(&g_tim8_cplm_pwm_handle);      /* MOE=1,使能主输出 */    
    ATIM_TIM8_CPLM_CH1_CCR1 = ccr1;                      /* 设置比较寄存器 */
    ATIM_TIM8_CPLM_CH2_CCR2 = ccr2;  
}

void RightMoterMove(double duty_cycle,uint8_t isrmoter_revers)
{
 if (isrmoter_revers)
 {    
    if (duty_cycle>0)
    {
        atim_tim8_cplm_pwm_set(0,(uint16_t)(duty_cycle*100));// 向前 转
    }
    else if (duty_cycle<0)
    {
        atim_tim8_cplm_pwm_set((uint16_t)(-duty_cycle*100),0);//向后 转
    }
    else
    {
        RightMoterStop(); 
    } 
 }
 else
 {
    if (duty_cycle>0)
    {
        atim_tim8_cplm_pwm_set((uint16_t)(duty_cycle*100),0);// 向后转
    }
    else if (duty_cycle<0)
    {
        atim_tim8_cplm_pwm_set(0,(uint16_t)(-duty_cycle*100));//向前转
    }
    else
    {
        RightMoterStop(); 
    }
 }

}

void RightMoterStop(void)
{
    atim_tim8_cplm_pwm_set(0,0);
    // RIGHT_SOFTBRAKE(1);
}

void MoterdriveInit(void)
{
   /* 周期：Tout= ((arr+1)*(psc+1))/Tclk   ；频率  = 1/Tout*/
    MoterL_pwm_chy_init(100 - 1, 84 - 1);                  //* 168 000 000 / 100*84   TIME1   L 20khz频率的PWM 波形*  /
    MoterR_pwm_chy_init(100 - 1, 84 - 1);                  //* 168 000 000 / 100*84   TIME8   R  20khz频率的PWM 波形*/
}

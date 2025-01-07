/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/�ڶ��׶�����Ŀ/���̱ջ�Demo����Ŀ/R9_407F_num_2/Drivers/BSP/R9/moterdriver.c
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
 * @brief       moterdriver ��������
 * @license     Copyright (c) 2024, ���ڸ���ҽ�ƿƼ����޹�˾
 */
#include "./BSP/R9/moterdriver.h"
/**********************************����L ���1 ������� TIME1 CH1 CH2 CH1N CH2N  *************************************/
TIM_HandleTypeDef g_tim1_cplm_pwm_handle;                              /* ��ʱ��1��� */
TIM_BreakDeadTimeConfigTypeDef g_sbreak1_dead_time1_config = {0};        /* ����ʱ������ */

/**
 *  @brief       �߼���ʱ��TIM1 ������� ��ʼ��������ʹ��PWMģʽ1��
 * @note
 *              ���ø߼���ʱ��TIMX �������, һ·OCy һ·OCyN, ���ҿ�����������ʱ��
 *              �߼���ʱ����ʱ������APB2, ��PCLK2 = 168Mhz, ��������PPRE2����Ƶ, ���
 *              �߼���ʱ��ʱ�� = 168Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��, ��λ : Mhz
 * @param         {uint16_t} arr:�Զ���װֵ��
 * @param         {uint16_t} psc:Ԥ��Ƶϵ��
 * @param         {uint8_t} dtg: 
 * 
**/
void MoterL_pwm_chy_init(uint16_t arr, uint16_t psc) // ���ֵ�� atim_timx_cplm_pwm_init(100 - 1, 84 - 1); /* 20Khz����ݔ��. */
{
    /*���ֵ��*/ 
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_cplm_pwm = {0};

    ATIM_TIM1_CPLM_CLK_ENABLE();            /* TIMx ʱ��ʹ�� */
    ATIM_TIM1_CPLM_CH1_GPIO_CLK_ENABLE();   /* ͨ��X��ӦIO��ʱ��ʹ�� */
    ATIM_TIM1_CPLM_CH1N_GPIO_CLK_ENABLE();  /* ͨ��X����ͨ����ӦIO��ʱ��ʹ�� */
    ATIM_TIM1_CPLM_CH2_GPIO_CLK_ENABLE();   /* ͨ��X��ӦIO��ʱ��ʹ�� */
    ATIM_TIM1_CPLM_CH2N_GPIO_CLK_ENABLE();  /* ͨ��X����ͨ����ӦIO��ʱ��ʹ�� */
    
    // ATIM_TIM1_CPLM_BKIN_GPIO_CLK_ENABLE();  /* ͨ��Xɲ�������ӦIO��ʱ��ʹ�� */
  
    // gpio_init_struct.Pin = ATIM_TIM1_CPLM_BKIN_GPIO_PIN;
    // gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    // gpio_init_struct.Pull = GPIO_PULLDOWN;
    // gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    // gpio_init_struct.Alternate = ATIM_TIM1_CPLM_CHY_GPIO_AF;                /* �˿ڸ��� */
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

    g_tim1_cplm_pwm_handle.Instance = ATIM_TIM1_CPLM;                       /* ��ʱ��x */
    g_tim1_cplm_pwm_handle.Init.Prescaler = psc;                            /* Ԥ��Ƶϵ�� */
    g_tim1_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;           /* ���ϼ���ģʽ */
    g_tim1_cplm_pwm_handle.Init.Period = arr;                               /* �Զ���װ��ֵ */
    g_tim1_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;     /* CKD[1:0] = 10, tDTS = 4 * tCK_INT = Ft / 4 = 42Mhz*/
    g_tim1_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  /* ʹ��Ӱ�ӼĴ���TIMx_ARR */
  
    HAL_TIM_PWM_Init(&g_tim1_cplm_pwm_handle) ;

    tim_oc_cplm_pwm.OCMode = TIM_OCMODE_PWM1;                               /* PWMģʽ1 */
    tim_oc_cplm_pwm.OCPolarity = TIM_OCPOLARITY_HIGH;                       /* OCy �ߵ�ƽ��Ч */
    tim_oc_cplm_pwm.OCNPolarity = TIM_OCPOLARITY_HIGH;                      /* OCyN �ߵ�ƽ��Ч */
    tim_oc_cplm_pwm.OCIdleState = TIM_OCIDLESTATE_RESET;                      /* ��MOE=0��OCx=0 */
    tim_oc_cplm_pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;                    /* ��MOE=0��OCxN=0 */
    HAL_TIM_PWM_ConfigChannel(&g_tim1_cplm_pwm_handle, &tim_oc_cplm_pwm, ATIM_TIM1_CPLM_CH1);
    HAL_TIM_PWM_ConfigChannel(&g_tim1_cplm_pwm_handle, &tim_oc_cplm_pwm, ATIM_TIM1_CPLM_CH2);
    
    /* �����������������������ж� */
    g_sbreak1_dead_time1_config.OffStateRunMode = TIM_OSSR_DISABLE;           /* ����ģʽ�Ĺر����״̬ */
    g_sbreak1_dead_time1_config.OffStateIDLEMode = TIM_OSSI_DISABLE;          /* ����ģʽ�Ĺر����״̬ */
    g_sbreak1_dead_time1_config.LockLevel = TIM_LOCKLEVEL_OFF;                /* ���üĴ��������� */
    g_sbreak1_dead_time1_config.BreakState = TIM_BREAK_DISABLE;                /* ʹ��ɲ������ */
    g_sbreak1_dead_time1_config.BreakPolarity = TIM_BREAKPOLARITY_LOW;       /* ɲ��������Ч�źż���Ϊ�ߣ����ߵ�ƽɲ�� */
    g_sbreak1_dead_time1_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; /* ʹ��AOEλ������ɲ���������Զ��ָ���� */
    g_sbreak1_dead_time1_config.DeadTime = 50; 
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim1_cplm_pwm_handle, &g_sbreak1_dead_time1_config);

    HAL_TIM_PWM_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);         /* OC1 ���ʹ�� */
    HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);      /* OC1N ���ʹ�� */
    
    HAL_TIM_PWM_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);         /* OC2 ���ʹ�� */
    HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);      /* OC2N ���ʹ�� */                       /* ������ӦPWMͨ�� */
    // LEFT_SOFTBRAKE(0); //��ɲ��ֹͣ ,�ߵ�ƽɲ����  
}

/**
 * @brief       ��ʱ��TIMX ��������Ƚ�ֵ & ����ʱ��
 * @param       ccr: ����Ƚ�ֵ
 * @param       dtg: ����ʱ��
 *   @arg       dtg[7:5]=0xxʱ, ����ʱ�� = dtg[7:0] * tDTS
 *   @arg       dtg[7:5]=10xʱ, ����ʱ�� = (64 + dtg[6:0]) * 2  * tDTS
 *   @arg       dtg[7:5]=110ʱ, ����ʱ�� = (32 + dtg[5:0]) * 8  * tDTS
 *   @arg       dtg[7:5]=111ʱ, ����ʱ�� = (32 + dtg[5:0]) * 16 * tDTS
 *   @note      tDTS = 1 / (Ft /  CKD[1:0]) = 1 / 42M = 23.8ns
 * @retval      ��
 * @return        {*}
**/
void atim_tim1_cplm_pwm_set(uint16_t ccr1,uint16_t ccr2)
{
    // g_sbreak1_dead_time1_config.DeadTime = 50; // 50 �� dtg[7:0] 001  ����ʱ�� = 50 *23.8 ns=1.19us
    // HAL_TIMEx_ConfigBreakDeadTime(&g_tim1_cplm_pwm_handle, &g_sbreak1_dead_time1_config);      /*��������ʱ��*/
    __HAL_TIM_MOE_ENABLE(&g_tim1_cplm_pwm_handle);      /* MOE=1,ʹ������� */    
    ATIM_TIM1_CPLM_CH1_CCR1 = ccr1;                      /* ���ñȽϼĴ��� */
    ATIM_TIM1_CPLM_CH2_CCR2 = ccr2;  
}

void LeftMoterMove(double duty_cycle, uint8_t islmoter_reverse)
{
    if (islmoter_reverse)
    {
        if (duty_cycle>0)
        {
            atim_tim1_cplm_pwm_set((uint16_t)(duty_cycle*100),0);  //�� ��

        }
        else if(duty_cycle<0)
        {
            atim_tim1_cplm_pwm_set(0,(uint16_t)(-duty_cycle*100));//�� ǰ
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
            atim_tim1_cplm_pwm_set(0,(uint16_t)(duty_cycle*100));  //�� ǰ

        }
        else if(duty_cycle<0)
        {
            atim_tim1_cplm_pwm_set((uint16_t)(-duty_cycle*100),0);//�� �� ת
        }
        else
        {
            LeftMoterStop(); 
        }
    }
}
void dcmotor_stop(void)
{
    HAL_TIM_PWM_Stop(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);         /* OC1 ���ʹ�� */
    HAL_TIMEx_PWMN_Stop(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);      /* OC1N ���ʹ�� */
    
    HAL_TIM_PWM_Stop(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);         /* OC2 ���ʹ�� */
    HAL_TIMEx_PWMN_Stop(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);      /* OC2N ���ʹ�� */                       /* ������ӦPWMͨ�� */
}

void dcmoter_dir(uint8_t para)
{
	
	dcmotor_stop();
	if (para==1) // ��ת
	{
		HAL_TIM_PWM_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);         /* OC1 ���ʹ�� */
        HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);      /* OC1N ���ʹ�� */
		HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);      /* OC2N ���ʹ�� */                       
		
		
	}
	else if (para==0) //��ת
	{
		HAL_TIM_PWM_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);         /* OC2 ���ʹ�� */
        HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH2);      /* OC2N ���ʹ�� */
		HAL_TIMEx_PWMN_Start(&g_tim1_cplm_pwm_handle, ATIM_TIM1_CPLM_CH1);      /* OC1N ���ʹ�� */                      
		
	}
	
}
void LeftMoterStop(void)
{
    atim_tim1_cplm_pwm_set(0,0);
    // LEFT_SOFTBRAKE(1);
}
/**********************************����R ���2������� TIME8 CH1 CH2 CH1N CH2N*************************************/
TIM_HandleTypeDef g_tim8_cplm_pwm_handle;                              /* ��ʱ��1��� */
TIM_BreakDeadTimeConfigTypeDef g_sbreak2_dead_time8_config = {0};        /* ����ʱ������ */
void MoterR_pwm_chy_init(uint16_t arr, uint16_t psc) // ���ֵ��
{
    /*���ֵ��*/
    GPIO_InitTypeDef gpio_init_struct = {0};
    TIM_OC_InitTypeDef tim_oc_cplm_pwm = {0};

    ATIM_TIM8_CPLM_CLK_ENABLE();            /* TIMx ʱ��ʹ�� */
    ATIM_TIM8_CPLM_CH1_GPIO_CLK_ENABLE();   /* ͨ��X��ӦIO��ʱ��ʹ�� */
    ATIM_TIM8_CPLM_CH1N_GPIO_CLK_ENABLE();  /* ͨ��X����ͨ����ӦIO��ʱ��ʹ�� */
    ATIM_TIM8_CPLM_CH2_GPIO_CLK_ENABLE();   /* ͨ��X��ӦIO��ʱ��ʹ�� */
    ATIM_TIM8_CPLM_CH2N_GPIO_CLK_ENABLE();  /* ͨ��X����ͨ����ӦIO��ʱ��ʹ�� */
    
    // ATIM_TIM8_CPLM_BKIN_GPIO_CLK_ENABLE();  /* ͨ��Xɲ�������ӦIO��ʱ��ʹ�� */
    // gpio_init_struct.Pin = ATIM_TIM8_CPLM_BKIN_GPIO_PIN;
    // gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    // gpio_init_struct.Pull = GPIO_PULLDOWN;
    // gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    // gpio_init_struct.Alternate = ATIM_TIM8_CPLM_CHY_GPIO_AF;                /* �˿ڸ��� */

    gpio_init_struct.Pin = ATIM_TIM8_CPLM_CH1_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLDOWN;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = ATIM_TIM8_CPLM_CHY_GPIO_AF;                /* �˿ڸ��� */
    HAL_GPIO_Init(ATIM_TIM8_CPLM_CH1_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ATIM_TIM8_CPLM_CH1N_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM8_CPLM_CH1N_GPIO_PORT, &gpio_init_struct);
    
    gpio_init_struct.Pin = ATIM_TIM8_CPLM_CH2_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM8_CPLM_CH2_GPIO_PORT, &gpio_init_struct);

    gpio_init_struct.Pin = ATIM_TIM8_CPLM_CH2N_GPIO_PIN;
    HAL_GPIO_Init(ATIM_TIM8_CPLM_CH2N_GPIO_PORT, &gpio_init_struct);

    g_tim8_cplm_pwm_handle.Instance = ATIM_TIM8_CPLM;                       /* ��ʱ��x */
    g_tim8_cplm_pwm_handle.Init.Prescaler = psc;                            /* Ԥ��Ƶϵ�� */
    g_tim8_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;           /* ���ϼ���ģʽ */
    g_tim8_cplm_pwm_handle.Init.Period = arr;                               /* �Զ���װ��ֵ */
    g_tim8_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;     /* CKD[1:0] = 10, tDTS = 4 * tCK_INT = Ft / 4 = 42Mhz*/
    g_tim8_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;  /* ʹ��Ӱ�ӼĴ���TIMx_ARR */
  
    HAL_TIM_PWM_Init(&g_tim8_cplm_pwm_handle) ;

    tim_oc_cplm_pwm.OCMode = TIM_OCMODE_PWM1;                               /* PWMģʽ1 */
    tim_oc_cplm_pwm.OCPolarity = TIM_OCPOLARITY_HIGH;                       /* OCy �ߵ�ƽ��Ч */
    tim_oc_cplm_pwm.OCNPolarity = TIM_OCPOLARITY_HIGH;                      /* OCyN �ߵ�ƽ��Ч */
    tim_oc_cplm_pwm.OCIdleState = TIM_OCIDLESTATE_RESET;                      /* ��MOE=0��OCx=0 */
    tim_oc_cplm_pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;                    /* ��MOE=0��OCxN=0 */
    HAL_TIM_PWM_ConfigChannel(&g_tim8_cplm_pwm_handle, &tim_oc_cplm_pwm, ATIM_TIM8_CPLM_CH1);
    HAL_TIM_PWM_ConfigChannel(&g_tim8_cplm_pwm_handle, &tim_oc_cplm_pwm, ATIM_TIM8_CPLM_CH2);
    
    /* �����������������������ж� */
    g_sbreak2_dead_time8_config.OffStateRunMode = TIM_OSSR_DISABLE;           /* ����ģʽ�Ĺر����״̬ */
    g_sbreak2_dead_time8_config.OffStateIDLEMode = TIM_OSSI_DISABLE;          /* ����ģʽ�Ĺر����״̬ */
    g_sbreak2_dead_time8_config.LockLevel = TIM_LOCKLEVEL_OFF;                /* ���üĴ��������� */
    g_sbreak2_dead_time8_config.BreakState = TIM_BREAK_DISABLE;                /* ʹ��ɲ������ */
    g_sbreak2_dead_time8_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;       /* ɲ��������Ч�źż���Ϊ�� */
    g_sbreak2_dead_time8_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE; /* ʹ��AOEλ������ɲ���������Զ��ָ���� */
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim8_cplm_pwm_handle, &g_sbreak2_dead_time8_config);

    HAL_TIM_PWM_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH1);         /* OC1 ���ʹ�� */
    HAL_TIMEx_PWMN_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH1);      /* OC1N ���ʹ�� */
    
    HAL_TIM_PWM_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH2);         /* OC2 ���ʹ�� */
    HAL_TIMEx_PWMN_Start(&g_tim8_cplm_pwm_handle, ATIM_TIM8_CPLM_CH2);      /* OC2N ���ʹ�� */   
    // RIGHT_SOFTBRAKE(0);//��ɲ��ֹͣ
}
void atim_tim8_cplm_pwm_set(uint16_t ccr1,uint16_t ccr2)
{
    g_sbreak2_dead_time8_config.DeadTime = 50;
    HAL_TIMEx_ConfigBreakDeadTime(&g_tim8_cplm_pwm_handle, &g_sbreak2_dead_time8_config);      /*��������ʱ��*/
    __HAL_TIM_MOE_ENABLE(&g_tim8_cplm_pwm_handle);      /* MOE=1,ʹ������� */    
    ATIM_TIM8_CPLM_CH1_CCR1 = ccr1;                      /* ���ñȽϼĴ��� */
    ATIM_TIM8_CPLM_CH2_CCR2 = ccr2;  
}

void RightMoterMove(double duty_cycle,uint8_t isrmoter_revers)
{
 if (isrmoter_revers)
 {    
    if (duty_cycle>0)
    {
        atim_tim8_cplm_pwm_set(0,(uint16_t)(duty_cycle*100));// ��ǰ ת
    }
    else if (duty_cycle<0)
    {
        atim_tim8_cplm_pwm_set((uint16_t)(-duty_cycle*100),0);//��� ת
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
        atim_tim8_cplm_pwm_set((uint16_t)(duty_cycle*100),0);// ���ת
    }
    else if (duty_cycle<0)
    {
        atim_tim8_cplm_pwm_set(0,(uint16_t)(-duty_cycle*100));//��ǰת
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
   /* ���ڣ�Tout= ((arr+1)*(psc+1))/Tclk   ��Ƶ��  = 1/Tout*/
    MoterL_pwm_chy_init(100 - 1, 84 - 1);                  //* 168 000 000 / 100*84   TIME1   L 20khzƵ�ʵ�PWM ����*  /
    MoterR_pwm_chy_init(100 - 1, 84 - 1);                  //* 168 000 000 / 100*84   TIME8   R  20khzƵ�ʵ�PWM ����*/
}

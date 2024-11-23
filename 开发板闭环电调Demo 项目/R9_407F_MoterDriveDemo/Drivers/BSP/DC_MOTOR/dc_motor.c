/**
 ****************************************************************************************************
 * @file        dc_motor.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       ֱ����ˢ������ƴ���
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� F407���������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com/forum.php
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211014
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#include "./BSP/DC_MOTOR/dc_motor.h"
#include "./SYSTEM/delay/delay.h"


/*************************************    ��������    *****************************************************/

TIM_HandleTypeDef g_atimx_cplm_pwm_handle;      /* ��ʱ����ʼ����� */

/**
 * @brief       �߼���ʱ��TIMX ������� ��ʼ��������ʹ��PWMģʽ1��
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc)
{

    GPIO_InitTypeDef gpio_init_struct;
    TIM_OC_InitTypeDef sConfigOC ;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig ;

    /******************************* ��һ��  ������ʱ����IOʱ�� ******************************/

    __HAL_RCC_GPIOA_CLK_ENABLE();           /* ��ͨ��IOʱ��ʹ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();           /* ����ͨ��IOʱ��ʹ�� */
    __HAL_RCC_TIM1_CLK_ENABLE();            /* ��ʱ��1ʱ��ʹ�� */


    /******************************* �ڶ���  ������ͨ��������ͨ��IO **************************/

    gpio_init_struct.Pin = GPIO_PIN_8;                                      /* ��ͨ������ */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH ;
    gpio_init_struct.Alternate = GPIO_AF1_TIM1;                             /* �˿ڸ��� */
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_PIN_13;                                     /* ����ͨ������ */
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);


    /******************************* ������  ���ö�ʱ�� **************************************/

    g_atimx_cplm_pwm_handle.Instance = TIM1;                                /* ��ʱ��1 */
    g_atimx_cplm_pwm_handle.Init.Prescaler = psc;                           /* ��ʱ��Ԥ��Ƶϵ�� */
    g_atimx_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;          /* ���ϼ���ģʽ */
    g_atimx_cplm_pwm_handle.Init.Period = arr;                              /* �Զ���װ��ֵ */
    g_atimx_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;    /* ʱ�ӷ�Ƶ���� */
    g_atimx_cplm_pwm_handle.Init.RepetitionCounter = 0;                     /* �ظ��������Ĵ���Ϊ0 */
    g_atimx_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;        /* ʹ��Ӱ�ӼĴ���TIMx_ARR */
    HAL_TIM_PWM_Init(&g_atimx_cplm_pwm_handle) ;


    /******************************* ���Ĳ�  ����PWM��� *************************************/

    sConfigOC.OCMode = TIM_OCMODE_PWM1;                                     /* PWMģʽ1 */
    sConfigOC.Pulse = 1200;                                                 /* Ԥ��һ���Ƚ�ֵ�������һ���̶�ת�� */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;                              /* OCy �͵�ƽ��Ч */
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;                            /* OCyN �͵�ƽ��Ч */
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;                               /* ��ʹ�ÿ���ģʽ */
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;                          /* ��ͨ���Ŀ���״̬ */
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;                        /* ����ͨ���Ŀ���״̬ */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_cplm_pwm_handle, &sConfigOC, TIM_CHANNEL_1);        /* ����PWMͨ�� */


    /******************************* ���岽  �������� **************************************/

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;                 /* OSSR����Ϊ1 �����ͨ����Чʱ�����Ч��ƽ */
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;               /* OSSI����Ϊ0 */
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;                     /* �ϵ�ֻ��дһ�Σ���Ҫ��������ʱ��ʱֻ���ô�ֵ */
    sBreakDeadTimeConfig.DeadTime = 0X0F;                                   /* ����ʱ�� */
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;                    /* BKE = 0, �ر�BKIN��� */
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;             /* BKP = 1, BKIN�͵�ƽ��Ч */
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;     /* ʹ��AOEλ������ɲ�����Զ��ָ���� */
    HAL_TIMEx_ConfigBreakDeadTime(&g_atimx_cplm_pwm_handle, &sBreakDeadTimeConfig);         /* ����BDTR�Ĵ��� */

}


/**
 * @brief       �����ʼ��
 * @param       ��
 * @retval      ��
 */
void dcmotor_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    __HAL_RCC_GPIOF_CLK_ENABLE();                           /* SD����ʱ��ʹ�� */

    gpio_init_struct.Pin = GPIO_PIN_10;                     /* SD���� */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* ������� */
    gpio_init_struct.Pull = GPIO_PULLDOWN;                  /* ���� */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* ���� */
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);                /* ��ʼ��SD���� */
    
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);  /* ����SD���� */
}

/**
 * @brief       �������
 * @param       ��
 * @retval      ��
 */
void dcmotor_start(void)
{
    HAL_TIM_Base_Start(&g_atimx_cplm_pwm_handle);           /* ������ʱ�� */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);    /* ����SD���� */
}

/**
 * @brief       ���ֹͣ
 * @param       ��
 * @retval      ��
 */
void dcmotor_stop(void)
{
    HAL_TIM_Base_Stop(&g_atimx_cplm_pwm_handle);            /* ������ʱ�� */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);  /* ����SD���� */
}

/**
 * @brief       �����ת��������
 * @param       para:���� 0��ת��1��ת
 * @note        �Ե�����棬˳ʱ�뷽����תΪ��ת
 * @retval      ��
 */
void dcmotor_dir(uint8_t para)
{
    HAL_TIM_PWM_Stop(&g_atimx_cplm_pwm_handle,TIM_CHANNEL_1);            /* �ر���ͨ����� */
    HAL_TIMEx_PWMN_Stop(&g_atimx_cplm_pwm_handle,TIM_CHANNEL_1);         /* �رջ���ͨ����� */
    
    if(para == 0)
    {
        HAL_TIM_PWM_Start(&g_atimx_cplm_pwm_handle,TIM_CHANNEL_1);       /* ������ͨ����� */
    }
    else if (para == 1)
    {
        HAL_TIMEx_PWMN_Start(&g_atimx_cplm_pwm_handle,TIM_CHANNEL_1);    /* ��������ͨ����� */
    }

}





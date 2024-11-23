/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/�ջ����Demo/R9_407F_MoterDriveDemo/Drivers/BSP/R9/moterdriver.c
 * @Description  :  
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-10-28 14:33:05
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
#include "./BSP/R9/moterdriver.h"
#include "./SYSTEM/delay/delay.h"

//******************************* ��ʱ�� **************************************/

TIM_HandleTypeDef g_atimx_cplm_pwm_handle;                              /* ��ʱ��x��� */

/**
 * @brief       �߼���ʱ��TIMX ������� ��ʼ��������ʹ��PWMģʽ1��
 * @note
 *              ���ø߼���ʱ��TIMX �������, һ·OCy һ·OCyN, ���ҿ�����������ʱ��
 *
 *              �߼���ʱ����ʱ������APB2, ��PCLK2 = 168Mhz, ��������PPRE2����Ƶ, ���
 *              �߼���ʱ��ʱ�� = 168Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��, ��λ : Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef sConfigOC ;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

    g_atimx_cplm_pwm_handle.Instance = ATIM_TIMX_CPLM;                      /* ��ʱ��x */
    g_atimx_cplm_pwm_handle.Init.Prescaler = psc;                           /* ��ʱ��Ԥ��Ƶϵ�� */
    g_atimx_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;          /* ���ϼ���ģʽ */
    g_atimx_cplm_pwm_handle.Init.Period = arr;                              /* �Զ���װ��ֵ */
    g_atimx_cplm_pwm_handle.Init.RepetitionCounter = 0;                     /* �ظ��������Ĵ���Ϊ0 */
    g_atimx_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;        /* ʹ��Ӱ�ӼĴ���TIMx_ARR */
    HAL_TIM_PWM_Init(&g_atimx_cplm_pwm_handle) ;

    /* ����PWM��� */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;                                     /* PWMģʽ1 */
    sConfigOC.Pulse = 0;                                                    /* �Ƚ�ֵΪ0 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;                              /* OCy �͵�ƽ��Ч */
    sConfigOC.OCNPolarity = TIM_OCPOLARITY_HIGH;                            /* OCyN �͵�ƽ��Ч */
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;                               /* ʹ�ÿ���ģʽ */
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;                          /* ��ͨ���Ŀ���״̬ */
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;                        /* ����ͨ���Ŀ���״̬ */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_cplm_pwm_handle, &sConfigOC, ATIM_TIMX_CPLM_CHY);    /* ���ú�Ĭ����CCER�Ļ������λ */   
    
    /* �����������������������ж� */
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;                 /* OSSR����Ϊ1 */
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;               /* OSSI����Ϊ0 */
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;                     /* �ϵ�ֻ��дһ�Σ���Ҫ��������ʱ��ʱֻ���ô�ֵ */
    sBreakDeadTimeConfig.DeadTime = 0X0F;                                   /* ����ʱ�� */
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;                    /* BKE = 0, �ر�BKIN��� */
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;             /* BKP = 1, BKIN�͵�ƽ��Ч */
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;     /* ʹ��AOEλ������ɲ�����Զ��ָ���� */
    HAL_TIMEx_ConfigBreakDeadTime(&g_atimx_cplm_pwm_handle, &sBreakDeadTimeConfig);         /* ����BDTR�Ĵ��� */

}

/**
 * @brief       ��ʱ���ײ�������ʱ��ʹ�ܣ���������
                �˺����ᱻHAL_TIM_PWM_Init()����
 * @param       htim:��ʱ�����
 * @retval      ��
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_CPLM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        
        ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE();   /* ͨ��X��ӦIO��ʱ��ʹ�� */
        ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE();  /* ����ͨ����ӦIO��ʱ��ʹ�� */
        ATIM_TIMX_CPLM_CLK_ENABLE();            /* ��ʱ��xʱ��ʹ�� */

        /* ����PWM��ͨ������ */
        gpio_init_struct.Pin = ATIM_TIMX_CPLM_CHY_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH ;
        gpio_init_struct.Alternate = ATIM_TIMX_CPLM_CHY_GPIO_AF;                /* �˿ڸ��� */
        HAL_GPIO_Init(ATIM_TIMX_CPLM_CHY_GPIO_PORT, &gpio_init_struct);

        /* ����PWM����ͨ������ */
        gpio_init_struct.Pin = ATIM_TIMX_CPLM_CHYN_GPIO_PIN;
        HAL_GPIO_Init(ATIM_TIMX_CPLM_CHYN_GPIO_PORT, &gpio_init_struct);
    }
}





/*************************************    ��������    *****************************************************/

TIM_HandleTypeDef g_atimx_cplm_pwm_handle;                              /* ��ʱ��x��� */

/**
 * @brief       �����ʼ��
 * @param       ��
 * @retval      ��
 */
void dcmotor_init(void)
{
    SHUTDOWN_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init_struct;
    
    /* SD�������ã�����Ϊ������� */
    gpio_init_struct.Pin = SHUTDOWN1_Pin;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SHUTDOWN1_GPIO_Port, &gpio_init_struct);

    HAL_GPIO_WritePin(GPIOF, SHUTDOWN1_Pin, GPIO_PIN_RESET);      /* SD���ͣ��ر���� */
    motor_pwm_set(0,0);
    dcmotor_start();                /* ������� */
}

/**
 * @brief       �������
 * @param       ��
 * @retval      ��
 */
void dcmotor_start(void)
{
    ENABLE_MOTOR;                                                       /* ����SD���ţ�������� */
}

/**
 * @brief       ���ֹͣ
 * @param       ��
 * @retval      ��
 */
void dcmotor_stop(void)
{
    HAL_TIM_PWM_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);          /* �ر���ͨ����� */
    HAL_TIMEx_PWMN_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);       /* �رջ���ͨ����� */
    DISABLE_MOTOR;                                                      /* ����SD���ţ�ֹͣ��� */
}

/**
 * @brief       �����ת��������
 * @param       para:���� 0��ת��1��ת
 * @note        �Ե�����棬˳ʱ�뷽����תΪ��ת
 * @retval      ��
 */
void dcmotor_dir(uint8_t para)
{
   
    if (para == 0)                /* ��ת */
    {
        HAL_TIM_PWM_Start(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);     /* ������ͨ����� */
        HAL_TIMEx_PWMN_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);       /* �رջ���ͨ����� */
				
    } 
    else if (para == 1)           /* ��ת */
    {
        HAL_TIMEx_PWMN_Start(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);  /* ��������ͨ����� */
        HAL_TIM_PWM_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);          /* �ر���ͨ����� */
    }
	else
	{
		dcmotor_stop();
		
	}
}

/**
 * @brief       ����ٶ�����
 * @param       para:�ȽϼĴ���ֵ
 * @retval      ��
 */
void dcmotor_speed(double  dutycycle)
{
   uint16_t  para;
   if (dutycycle<0)
   {
    dutycycle = 0;
   }
   else if(dutycycle>1.0)
   {
    dutycycle = 1.0;

   }
   else
   {

    dutycycle = dutycycle;
   }
    para = 100*dutycycle;
    __HAL_TIM_SetCompare(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1, para);

}

/**
 * @brief       �������
 * @param       para: pwm�Ƚ�ֵ ,�������Ϊ��ת������Ϊ��ת
 * @note        ���ݴ���Ĳ������Ƶ����ת����ٶ�
 * @retval      ��
 */
void motor_pwm_set(uint8_t para , double  dutycycle)
{
   
    dcmotor_dir(para);           
    dcmotor_speed(dutycycle);
    dcmotor_start();

}

/*************************************    ��������    ����������    ****************************************************/
/********************************* 1 ͨ�ö�ʱ�� ���������� *************************************/

TIM_HandleTypeDef g_timx_encode_chy_handle;         /* ��ʱ��x��� */
TIM_Encoder_InitTypeDef g_timx_encoder_chy_handle;  /* ��ʱ����������� */

/**
 * @brief       ͨ�ö�ʱ��TIMX ͨ��Y �������ӿ�ģʽ ��ʼ������
 * @note
 *              ͨ�ö�ʱ����ʱ������APB1,��PPRE1 �� 2��Ƶ��ʱ��
 *              ͨ�ö�ʱ����ʱ��ΪAPB1ʱ�ӵ�2��, ��APB1Ϊ42M, ���Զ�ʱ��ʱ�� = 84Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc)
{
    /* ��ʱ��x���� */
    g_timx_encode_chy_handle.Instance = GTIM_TIMX_ENCODER;                      /* ��ʱ��x */
    g_timx_encode_chy_handle.Init.Prescaler = psc;                              /* ��ʱ����Ƶ */
    g_timx_encode_chy_handle.Init.Period = arr;                                 /* �Զ���װ��ֵ */
    g_timx_encode_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;       /* ʱ�ӷ�Ƶ���� */
    
    /* ��ʱ��x���������� */
    g_timx_encoder_chy_handle.EncoderMode = TIM_ENCODERMODE_TI12;               /* TI1��TI2����⣬4��Ƶ */
    g_timx_encoder_chy_handle.IC1Polarity = TIM_ICPOLARITY_RISING;              /* ���뼫�ԣ��Ƿ��� */
    g_timx_encoder_chy_handle.IC1Selection = TIM_ICSELECTION_DIRECTTI;          /* ����ͨ��ѡ�� */
    g_timx_encoder_chy_handle.IC1Prescaler = TIM_ICPSC_DIV1;                    /* ����Ƶ */
    g_timx_encoder_chy_handle.IC1Filter = 10;                                   /* �˲������� */
    g_timx_encoder_chy_handle.IC2Polarity = TIM_ICPOLARITY_RISING;              /* ���뼫�ԣ��Ƿ��� */
    g_timx_encoder_chy_handle.IC2Selection = TIM_ICSELECTION_DIRECTTI;          /* ����ͨ��ѡ�� */
    g_timx_encoder_chy_handle.IC2Prescaler = TIM_ICPSC_DIV1;                    /* ����Ƶ */
    g_timx_encoder_chy_handle.IC2Filter = 10;                                   /* �˲������� */
    HAL_TIM_Encoder_Init(&g_timx_encode_chy_handle, &g_timx_encoder_chy_handle);/* ��ʼ����ʱ��x������ */
     
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle,GTIM_TIMX_ENCODER_CH1);     /* ʹ�ܱ�����ͨ��1 */
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle,GTIM_TIMX_ENCODER_CH2);     /* ʹ�ܱ�����ͨ��2 */
    __HAL_TIM_ENABLE_IT(&g_timx_encode_chy_handle,TIM_IT_UPDATE);               /* ʹ�ܸ����ж� */
    __HAL_TIM_CLEAR_FLAG(&g_timx_encode_chy_handle,TIM_IT_UPDATE);              /* ��������жϱ�־λ */
    
}

/**
 * @brief       ��ʱ���ײ�������ʱ��ʹ�ܣ���������
                �˺����ᱻHAL_TIM_Encoder_Init()����
 * @param       htim:��ʱ�����
 * @retval      ��
 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == GTIM_TIMX_ENCODER)
    {
        GPIO_InitTypeDef gpio_init_struct;
        GTIM_TIMX_ENCODER_CH1_GPIO_CLK_ENABLE();                                 /* ����ͨ��y��GPIOʱ�� */
        GTIM_TIMX_ENCODER_CH2_GPIO_CLK_ENABLE();
        GTIM_TIMX_ENCODER_CH1_CLK_ENABLE();                                      /* ������ʱ��ʱ�� */
        GTIM_TIMX_ENCODER_CH2_CLK_ENABLE();

        gpio_init_struct.Pin = GTIM_TIMX_ENCODER_CH1_GPIO_PIN;                   /* ͨ��y��GPIO�� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                                 /* ����������� */
        gpio_init_struct.Pull = GPIO_NOPULL;                                     /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                           /* ���� */
        gpio_init_struct.Alternate = GTIM_TIMX_ENCODERCH1_GPIO_AF;               /* �˿ڸ��� */
        HAL_GPIO_Init(GTIM_TIMX_ENCODER_CH1_GPIO_PORT, &gpio_init_struct);  
        
        gpio_init_struct.Pin = GTIM_TIMX_ENCODER_CH2_GPIO_PIN;                   /* ͨ��y��GPIO�� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                                 /* ����������� */
        gpio_init_struct.Pull = GPIO_NOPULL;                                     /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                           /* ���� */
        gpio_init_struct.Alternate = GTIM_TIMX_ENCODERCH2_GPIO_AF;               /* �˿ڸ��� */
        HAL_GPIO_Init(GTIM_TIMX_ENCODER_CH2_GPIO_PORT, &gpio_init_struct);         
       
        HAL_NVIC_SetPriority(GTIM_TIMX_ENCODER_INT_IRQn, 2, 0);                  /* �ж����ȼ����� */
        HAL_NVIC_EnableIRQ(GTIM_TIMX_ENCODER_INT_IRQn);                          /* �����ж� */
    }
}

/**
 * @brief       ��ʱ���жϷ�����
 * @param       ��
 * @retval      ��
 */
void GTIM_TIMX_ENCODER_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_encode_chy_handle);
}

Motor_TypeDef g_motor_data;  /*�����������*/
ENCODE_TypeDef g_encode;     /*��������������*/

/**
 * @brief       ����ٶȼ���
 * @param       encode_now����ǰ�������ܵļ���ֵ
 *              ms�������ٶȵļ�����ж�1ms����һ�Σ�����ms = 5��5ms����һ���ٶ�
 * @retval      ��
 */
void speed_computer(int32_t encode_now, uint8_t ms)
{
    uint8_t i = 0, j = 0;
    float temp = 0.0;
    static uint8_t sp_count = 0, k = 0;
    static float speed_arr[10] = {0.0};                     /* �洢�ٶȽ����˲����� */

    if (sp_count == ms)                                     /* ����һ���ٶ� */
    {
        /* ������ת�� 
           ��һ�� ������ms�����ڼ����仯��
           �ڶ��� ������1min�ڼ����仯����g_encode.speed * ((1000 / ms) * 60 ��
           ������ �����Ա�������תһȦ�ļ�����������Ƶ���� * �������ֱ��ʣ�
           ���Ĳ� �����Լ��ٱȼ��ɵó����ת��
        */
        g_encode.encode_now = encode_now;                                /* ȡ����������ǰ����ֵ */
        g_encode.speed = (g_encode.encode_now - g_encode.encode_old);    /* �������������ֵ�ı仯�� */
        
        speed_arr[k++] = (float)(g_encode.speed * ((1000 / ms) * 60.0) / REDUCTION_RATIO / ROTO_RATIO );    /* ������ת�� */
        
        g_encode.encode_old = g_encode.encode_now;          /* ���浱ǰ��������ֵ */

        /* �ۼ�10���ٶ�ֵ�����������˲�*/
        if (k == 10)
        {
            for (i = 10; i >= 1; i--)                       /* ð������*/
            {
                for (j = 0; j < (i - 1); j++) 
                {
                    if (speed_arr[j] > speed_arr[j + 1])    /* ��ֵ�Ƚ� */
                    { 
                        temp = speed_arr[j];                /* ��ֵ��λ */
                        speed_arr[j] = speed_arr[j + 1];
                        speed_arr[j + 1] = temp;
                    }
                }
            }
            
            temp = 0.0;
            
            for (i = 2; i < 8; i++)                         /* ȥ�����߸ߵ����� */
            {
                temp += speed_arr[i];                       /* ���м���ֵ�ۼ� */
            }
            
            temp = (float)(temp / 6);                       /*���ٶ�ƽ��ֵ*/
            
            /* һ�׵�ͨ�˲�
             * ��ʽΪ��Y(n)= qX(n) + (1-q)Y(n-1)
             * ����X(n)Ϊ���β���ֵ��Y(n-1)Ϊ�ϴ��˲����ֵ��Y(n)Ϊ�����˲����ֵ��qΪ�˲�ϵ��
             * qֵԽС����һ������Ա������Ӱ��Խ����������Խƽ�ȣ����Ƕ����ٶȱ仯����ӦҲ��Խ��
             */
            g_motor_data.speed = (float)( ((float)0.58 * temp) + (g_motor_data.speed * (float)0.42) );
            k = 0;
        }
        sp_count = 0;
    }
    sp_count ++;
}


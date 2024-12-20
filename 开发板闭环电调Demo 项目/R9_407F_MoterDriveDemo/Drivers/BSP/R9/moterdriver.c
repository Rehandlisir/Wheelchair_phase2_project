/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/闭环电调Demo/R9_407F_MoterDriveDemo/Drivers/BSP/R9/moterdriver.c
 * @Description  :  
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-10-28 14:33:05
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
#include "./BSP/R9/moterdriver.h"
#include "./SYSTEM/delay/delay.h"

//******************************* 定时器 **************************************/

TIM_HandleTypeDef g_atimx_cplm_pwm_handle;                              /* 定时器x句柄 */

/**
 * @brief       高级定时器TIMX 互补输出 初始化函数（使用PWM模式1）
 * @note
 *              配置高级定时器TIMX 互补输出, 一路OCy 一路OCyN, 并且可以设置死区时间
 *
 *              高级定时器的时钟来自APB2, 而PCLK2 = 168Mhz, 我们设置PPRE2不分频, 因此
 *              高级定时器时钟 = 168Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率, 单位 : Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc)
{
    TIM_OC_InitTypeDef sConfigOC ;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

    g_atimx_cplm_pwm_handle.Instance = ATIM_TIMX_CPLM;                      /* 定时器x */
    g_atimx_cplm_pwm_handle.Init.Prescaler = psc;                           /* 定时器预分频系数 */
    g_atimx_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;          /* 向上计数模式 */
    g_atimx_cplm_pwm_handle.Init.Period = arr;                              /* 自动重装载值 */
    g_atimx_cplm_pwm_handle.Init.RepetitionCounter = 0;                     /* 重复计数器寄存器为0 */
    g_atimx_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;        /* 使能影子寄存器TIMx_ARR */
    HAL_TIM_PWM_Init(&g_atimx_cplm_pwm_handle) ;

    /* 设置PWM输出 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;                                     /* PWM模式1 */
    sConfigOC.Pulse = 0;                                                    /* 比较值为0 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;                              /* OCy 低电平有效 */
    sConfigOC.OCNPolarity = TIM_OCPOLARITY_HIGH;                            /* OCyN 低电平有效 */
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;                               /* 使用快速模式 */
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;                          /* 主通道的空闲状态 */
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;                        /* 互补通道的空闲状态 */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_cplm_pwm_handle, &sConfigOC, ATIM_TIMX_CPLM_CHY);    /* 配置后默认清CCER的互补输出位 */   
    
    /* 设置死区参数，开启死区中断 */
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;                 /* OSSR设置为1 */
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;               /* OSSI设置为0 */
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;                     /* 上电只能写一次，需要更新死区时间时只能用此值 */
    sBreakDeadTimeConfig.DeadTime = 0X0F;                                   /* 死区时间 */
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;                    /* BKE = 0, 关闭BKIN检测 */
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;             /* BKP = 1, BKIN低电平有效 */
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;     /* 使能AOE位，允许刹车后自动恢复输出 */
    HAL_TIMEx_ConfigBreakDeadTime(&g_atimx_cplm_pwm_handle, &sBreakDeadTimeConfig);         /* 设置BDTR寄存器 */

}

/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
                此函数会被HAL_TIM_PWM_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_CPLM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        
        ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE();   /* 通道X对应IO口时钟使能 */
        ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE();  /* 互补通道对应IO口时钟使能 */
        ATIM_TIMX_CPLM_CLK_ENABLE();            /* 定时器x时钟使能 */

        /* 配置PWM主通道引脚 */
        gpio_init_struct.Pin = ATIM_TIMX_CPLM_CHY_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH ;
        gpio_init_struct.Alternate = ATIM_TIMX_CPLM_CHY_GPIO_AF;                /* 端口复用 */
        HAL_GPIO_Init(ATIM_TIMX_CPLM_CHY_GPIO_PORT, &gpio_init_struct);

        /* 配置PWM互补通道引脚 */
        gpio_init_struct.Pin = ATIM_TIMX_CPLM_CHYN_GPIO_PIN;
        HAL_GPIO_Init(ATIM_TIMX_CPLM_CHYN_GPIO_PORT, &gpio_init_struct);
    }
}





/*************************************    基本驱动    *****************************************************/

TIM_HandleTypeDef g_atimx_cplm_pwm_handle;                              /* 定时器x句柄 */

/**
 * @brief       电机初始化
 * @param       无
 * @retval      无
 */
void dcmotor_init(void)
{
    SHUTDOWN_GPIO_CLK_ENABLE();
    GPIO_InitTypeDef gpio_init_struct;
    
    /* SD引脚设置，设置为推挽输出 */
    gpio_init_struct.Pin = SHUTDOWN1_Pin;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SHUTDOWN1_GPIO_Port, &gpio_init_struct);

    HAL_GPIO_WritePin(GPIOF, SHUTDOWN1_Pin, GPIO_PIN_RESET);      /* SD拉低，关闭输出 */
    motor_pwm_set(0,0);
    dcmotor_start();                /* 开启电机 */
}

/**
 * @brief       电机开启
 * @param       无
 * @retval      无
 */
void dcmotor_start(void)
{
    ENABLE_MOTOR;                                                       /* 拉高SD引脚，开启电机 */
}

/**
 * @brief       电机停止
 * @param       无
 * @retval      无
 */
void dcmotor_stop(void)
{
    HAL_TIM_PWM_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);          /* 关闭主通道输出 */
    HAL_TIMEx_PWMN_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);       /* 关闭互补通道输出 */
    DISABLE_MOTOR;                                                      /* 拉低SD引脚，停止电机 */
}

/**
 * @brief       电机旋转方向设置
 * @param       para:方向 0正转，1反转
 * @note        以电机正面，顺时针方向旋转为正转
 * @retval      无
 */
void dcmotor_dir(uint8_t para)
{
   
    if (para == 0)                /* 正转 */
    {
        HAL_TIM_PWM_Start(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);     /* 开启主通道输出 */
        HAL_TIMEx_PWMN_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);       /* 关闭互补通道输出 */
				
    } 
    else if (para == 1)           /* 反转 */
    {
        HAL_TIMEx_PWMN_Start(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);  /* 开启互补通道输出 */
        HAL_TIM_PWM_Stop(&g_atimx_cplm_pwm_handle, TIM_CHANNEL_1);          /* 关闭主通道输出 */
    }
	else
	{
		dcmotor_stop();
		
	}
}

/**
 * @brief       电机速度设置
 * @param       para:比较寄存器值
 * @retval      无
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
 * @brief       电机控制
 * @param       para: pwm比较值 ,正数电机为正转，负数为反转
 * @note        根据传入的参数控制电机的转向和速度
 * @retval      无
 */
void motor_pwm_set(uint8_t para , double  dutycycle)
{
   
    dcmotor_dir(para);           
    dcmotor_speed(dutycycle);
    dcmotor_start();

}

/*************************************    第三部分    编码器测速    ****************************************************/
/********************************* 1 通用定时器 编码器程序 *************************************/

TIM_HandleTypeDef g_timx_encode_chy_handle;         /* 定时器x句柄 */
TIM_Encoder_InitTypeDef g_timx_encoder_chy_handle;  /* 定时器编码器句柄 */

/**
 * @brief       通用定时器TIMX 通道Y 编码器接口模式 初始化函数
 * @note
 *              通用定时器的时钟来自APB1,当PPRE1 ≥ 2分频的时候
 *              通用定时器的时钟为APB1时钟的2倍, 而APB1为42M, 所以定时器时钟 = 84Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc)
{
    /* 定时器x配置 */
    g_timx_encode_chy_handle.Instance = GTIM_TIMX_ENCODER;                      /* 定时器x */
    g_timx_encode_chy_handle.Init.Prescaler = psc;                              /* 定时器分频 */
    g_timx_encode_chy_handle.Init.Period = arr;                                 /* 自动重装载值 */
    g_timx_encode_chy_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;       /* 时钟分频因子 */
    
    /* 定时器x编码器配置 */
    g_timx_encoder_chy_handle.EncoderMode = TIM_ENCODERMODE_TI12;               /* TI1、TI2都检测，4倍频 */
    g_timx_encoder_chy_handle.IC1Polarity = TIM_ICPOLARITY_RISING;              /* 输入极性，非反向 */
    g_timx_encoder_chy_handle.IC1Selection = TIM_ICSELECTION_DIRECTTI;          /* 输入通道选择 */
    g_timx_encoder_chy_handle.IC1Prescaler = TIM_ICPSC_DIV1;                    /* 不分频 */
    g_timx_encoder_chy_handle.IC1Filter = 10;                                   /* 滤波器设置 */
    g_timx_encoder_chy_handle.IC2Polarity = TIM_ICPOLARITY_RISING;              /* 输入极性，非反向 */
    g_timx_encoder_chy_handle.IC2Selection = TIM_ICSELECTION_DIRECTTI;          /* 输入通道选择 */
    g_timx_encoder_chy_handle.IC2Prescaler = TIM_ICPSC_DIV1;                    /* 不分频 */
    g_timx_encoder_chy_handle.IC2Filter = 10;                                   /* 滤波器设置 */
    HAL_TIM_Encoder_Init(&g_timx_encode_chy_handle, &g_timx_encoder_chy_handle);/* 初始化定时器x编码器 */
     
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle,GTIM_TIMX_ENCODER_CH1);     /* 使能编码器通道1 */
    HAL_TIM_Encoder_Start(&g_timx_encode_chy_handle,GTIM_TIMX_ENCODER_CH2);     /* 使能编码器通道2 */
    __HAL_TIM_ENABLE_IT(&g_timx_encode_chy_handle,TIM_IT_UPDATE);               /* 使能更新中断 */
    __HAL_TIM_CLEAR_FLAG(&g_timx_encode_chy_handle,TIM_IT_UPDATE);              /* 清除更新中断标志位 */
    
}

/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
                此函数会被HAL_TIM_Encoder_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == GTIM_TIMX_ENCODER)
    {
        GPIO_InitTypeDef gpio_init_struct;
        GTIM_TIMX_ENCODER_CH1_GPIO_CLK_ENABLE();                                 /* 开启通道y的GPIO时钟 */
        GTIM_TIMX_ENCODER_CH2_GPIO_CLK_ENABLE();
        GTIM_TIMX_ENCODER_CH1_CLK_ENABLE();                                      /* 开启定时器时钟 */
        GTIM_TIMX_ENCODER_CH2_CLK_ENABLE();

        gpio_init_struct.Pin = GTIM_TIMX_ENCODER_CH1_GPIO_PIN;                   /* 通道y的GPIO口 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                                 /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_NOPULL;                                     /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                           /* 高速 */
        gpio_init_struct.Alternate = GTIM_TIMX_ENCODERCH1_GPIO_AF;               /* 端口复用 */
        HAL_GPIO_Init(GTIM_TIMX_ENCODER_CH1_GPIO_PORT, &gpio_init_struct);  
        
        gpio_init_struct.Pin = GTIM_TIMX_ENCODER_CH2_GPIO_PIN;                   /* 通道y的GPIO口 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                                 /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_NOPULL;                                     /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;                           /* 高速 */
        gpio_init_struct.Alternate = GTIM_TIMX_ENCODERCH2_GPIO_AF;               /* 端口复用 */
        HAL_GPIO_Init(GTIM_TIMX_ENCODER_CH2_GPIO_PORT, &gpio_init_struct);         
       
        HAL_NVIC_SetPriority(GTIM_TIMX_ENCODER_INT_IRQn, 2, 0);                  /* 中断优先级设置 */
        HAL_NVIC_EnableIRQ(GTIM_TIMX_ENCODER_INT_IRQn);                          /* 开启中断 */
    }
}

/**
 * @brief       定时器中断服务函数
 * @param       无
 * @retval      无
 */
void GTIM_TIMX_ENCODER_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_encode_chy_handle);
}

Motor_TypeDef g_motor_data;  /*电机参数变量*/
ENCODE_TypeDef g_encode;     /*编码器参数变量*/

/**
 * @brief       电机速度计算
 * @param       encode_now：当前编码器总的计数值
 *              ms：计算速度的间隔，中断1ms进入一次，例如ms = 5即5ms计算一次速度
 * @retval      无
 */
void speed_computer(int32_t encode_now, uint8_t ms)
{
    uint8_t i = 0, j = 0;
    float temp = 0.0;
    static uint8_t sp_count = 0, k = 0;
    static float speed_arr[10] = {0.0};                     /* 存储速度进行滤波运算 */

    if (sp_count == ms)                                     /* 计算一次速度 */
    {
        /* 计算电机转速 
           第一步 ：计算ms毫秒内计数变化量
           第二步 ；计算1min内计数变化量：g_encode.speed * ((1000 / ms) * 60 ，
           第三步 ：除以编码器旋转一圈的计数次数（倍频倍数 * 编码器分辨率）
           第四步 ：除以减速比即可得出电机转速
        */
        g_encode.encode_now = encode_now;                                /* 取出编码器当前计数值 */
        g_encode.speed = (g_encode.encode_now - g_encode.encode_old);    /* 计算编码器计数值的变化量 */
        
        speed_arr[k++] = (float)(g_encode.speed * ((1000 / ms) * 60.0) / REDUCTION_RATIO / ROTO_RATIO );    /* 保存电机转速 */
        
        g_encode.encode_old = g_encode.encode_now;          /* 保存当前编码器的值 */

        /* 累计10次速度值，后续进行滤波*/
        if (k == 10)
        {
            for (i = 10; i >= 1; i--)                       /* 冒泡排序*/
            {
                for (j = 0; j < (i - 1); j++) 
                {
                    if (speed_arr[j] > speed_arr[j + 1])    /* 数值比较 */
                    { 
                        temp = speed_arr[j];                /* 数值换位 */
                        speed_arr[j] = speed_arr[j + 1];
                        speed_arr[j + 1] = temp;
                    }
                }
            }
            
            temp = 0.0;
            
            for (i = 2; i < 8; i++)                         /* 去除两边高低数据 */
            {
                temp += speed_arr[i];                       /* 将中间数值累加 */
            }
            
            temp = (float)(temp / 6);                       /*求速度平均值*/
            
            /* 一阶低通滤波
             * 公式为：Y(n)= qX(n) + (1-q)Y(n-1)
             * 其中X(n)为本次采样值；Y(n-1)为上次滤波输出值；Y(n)为本次滤波输出值，q为滤波系数
             * q值越小则上一次输出对本次输出影响越大，整体曲线越平稳，但是对于速度变化的响应也会越慢
             */
            g_motor_data.speed = (float)( ((float)0.58 * temp) + (g_motor_data.speed * (float)0.42) );
            k = 0;
        }
        sp_count = 0;
    }
    sp_count ++;
}


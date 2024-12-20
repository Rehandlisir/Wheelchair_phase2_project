
#include "./SYSTEM/delay/delay.h"
#include "./BSP/ADC/adc.h"
#include "./BSP/R9/moterdriver.h"
/* 多通道ADC采集 DMA读取 */
ADC_HandleTypeDef g_adc_nch_dma_handle;                 /* 与DMA关联的ADC句柄 */
DMA_HandleTypeDef g_dma_nch_adc_handle;                 /* 与ADC关联的DMA句柄 */
uint16_t g_adc_value[ADC_CH_NUM * ADC_COLL] = {0};      /* 存储ADC原始值 */

/***************************************  电压、电流、温度 多通道ADC采集(DMA读取)程序*****************************************/

/**
 * @brief       ADC初始化函数
 * @param       无
 * @retval      无
 */
void adc_init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    g_adc_nch_dma_handle.Instance = ADC_ADCX;                                       /* ADCx */
    g_adc_nch_dma_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;            /* 4分频，ADCCLK = PCLK2/4 = 84/4 = 21Mhz */
    g_adc_nch_dma_handle.Init.Resolution = ADC_RESOLUTION_12B;                      /* 12位模式 */
    g_adc_nch_dma_handle.Init.ScanConvMode = ENABLE;                                /* 扫描模式 多通道使用 */
    g_adc_nch_dma_handle.Init.ContinuousConvMode = ENABLE;                          /* 连续转换模式，转换完成之后接着继续转换 */
    g_adc_nch_dma_handle.Init.DiscontinuousConvMode = DISABLE;                      /* 禁止不连续采样模式 */
    g_adc_nch_dma_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* 使用软件触发 */
    g_adc_nch_dma_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                /* 软件触发 */
    g_adc_nch_dma_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                      /* 右对齐 */
    g_adc_nch_dma_handle.Init.NbrOfConversion = ADC_CH_NUM;                         /* 使用转换通道数，需根据实际转换通道去设置 */
    g_adc_nch_dma_handle.Init.DMAContinuousRequests = ENABLE;                       /* 开启DMA连续转换请求 */
    g_adc_nch_dma_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&g_adc_nch_dma_handle);

    /* 配置使用的ADC通道，采样序列里的第几个转换，增加或者减少通道需要修改这部分 */
    sConfig.Channel = ADC_ADCX_CH0;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);

    sConfig.Channel = ADC_ADCX_CH1;
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);

    sConfig.Channel = ADC_ADCX_CH2;
    sConfig.Rank = 3;
    HAL_ADC_ConfigChannel(&g_adc_nch_dma_handle, &sConfig);
    
}

/**
 * @brief       ADC DMA读取 初始化函数
 *   @note      本函数还是使用adc_init对ADC进行大部分配置,有差异的地方再单独配置
 * @param       par         : 外设地址
 * @param       mar         : 存储器地址
 * @retval      无
 */
void adc_nch_dma_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
      
    ADC_ADCX_CHY_CLK_ENABLE();                                  /* 使能ADCx时钟 */
    ADC_ADCX_CH0_GPIO_CLK_ENABLE();                             /* 开启GPIO时钟 */
    ADC_ADCX_CH1_GPIO_CLK_ENABLE();
    ADC_ADCX_CH2_GPIO_CLK_ENABLE();
    
    /* AD采集引脚模式设置,模拟输入 */
    GPIO_InitStruct.Pin = ADC_ADCX_CH0_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_ADCX_CH0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_ADCX_CH1_GPIO_PIN;
    HAL_GPIO_Init(ADC_ADCX_CH1_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = ADC_ADCX_CH2_GPIO_PIN;   
    HAL_GPIO_Init(ADC_ADCX_CH2_GPIO_PORT, &GPIO_InitStruct); 
    
    adc_init();                                                 /* 先初始化ADC */
    
    if ((uint32_t)ADC_ADCX_DMASx > (uint32_t)DMA2)              /* 大于DMA1_Channel7, 则为DMA2的通道了 */
    {
        __HAL_RCC_DMA2_CLK_ENABLE();                            /* DMA2时钟使能 */
    }
    else 
    {
        __HAL_RCC_DMA1_CLK_ENABLE();                            /* DMA1时钟使能 */
    }

    /* DMA配置 */
    g_dma_nch_adc_handle.Instance = ADC_ADCX_DMASx;                             /* 数据流x */
    g_dma_nch_adc_handle.Init.Channel = ADC_ADCX_DMASx_Chanel;                  /* DMA通道x */
    g_dma_nch_adc_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;                 /* DIR = 1 ,外设到存储器模式 */
    g_dma_nch_adc_handle.Init.PeriphInc = DMA_PINC_DISABLE;                     /* 外设非增量模式 */
    g_dma_nch_adc_handle.Init.MemInc = DMA_MINC_ENABLE;                         /* 存储器增量模式 */
    g_dma_nch_adc_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;    /* 外设数据长度:16位 */
    g_dma_nch_adc_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;       /* 存储器数据长度:16位 */
    g_dma_nch_adc_handle.Init.Mode = DMA_CIRCULAR;                              /* 外设流控模式 */
    g_dma_nch_adc_handle.Init.Priority = DMA_PRIORITY_MEDIUM;                   /* 中等优先级 */
    HAL_DMA_Init(&g_dma_nch_adc_handle);
 
    __HAL_LINKDMA(&g_adc_nch_dma_handle,DMA_Handle,g_dma_nch_adc_handle);       /* 把ADC和DMA关联，用DMA传输ADC数据 */
    
    /* ADC DMA中断配置 */
    HAL_NVIC_SetPriority(ADC_ADCX_DMASx_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(ADC_ADCX_DMASx_IRQn);
    
    HAL_ADC_Start_DMA(&g_adc_nch_dma_handle,(uint32_t *)g_adc_value,ADC_SUM);    /* 开启ADC的DMA传输 */
}

/**
 * @brief       DMA2 数据流4中断服务函数
 * @param       无 
 * @retval      无
 */
void ADC_ADCX_DMASx_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_dma_nch_adc_handle);
}

uint16_t g_adc_val[ADC_CH_NUM];                   /*ADC平均值存放数组*/
/**
 * @brief       ADC采集中断服务回调函数
 * @param       无 
 * @retval      无
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    float temp_c = 0.0;
    static float add_adc = 0;
    static float init_adc_value = 0;
    static uint8_t adc_count1 = 0, adc_count2 = 0;
    if (hadc->Instance == ADC_ADCX)
    {
        HAL_ADC_Stop_DMA(&g_adc_nch_dma_handle);  /*关闭DMA转换*/
        calc_adc_val(g_adc_val);                  /*计算ADC的平均值*/
        g_motor_data.volatage =  g_adc_val[0] * ADC2VBUS-45.88; // 计算电源电压
    }   
    HAL_ADC_Start_DMA(&g_adc_nch_dma_handle, (uint32_t *)&g_adc_value, (uint32_t)(ADC_SUM)); /*启动DMA转换*/
}


/**
 * @brief       计算ADC的平均值（滤波）
 * @param       * p ：存放ADC值的指针地址
 * @note        此函数对电压、温度、电流对应的ADC值进行滤波，
 *              p[0]-p[2]对应的分别是电压、温度和电流
 * @retval      无
 */
void calc_adc_val(uint16_t * p)
{
    uint32_t temp[3] = {0,0,0};
    int i;
    for(i=0;i<ADC_COLL;i++)         /* 循环ADC_COLL次取值，累加 */
    {
        temp[0] += g_adc_value[0+i*ADC_CH_NUM];
        temp[1] += g_adc_value[1+i*ADC_CH_NUM];
        temp[2] += g_adc_value[2+i*ADC_CH_NUM];
    }
    temp[0] /= ADC_COLL;            /* 取平均值 */
    temp[1] /= ADC_COLL;
    temp[2] /= ADC_COLL;
    p[0] = temp[0];                 /* 存入电压ADC通道平均值 */
    p[1] = temp[1];                 /* 存入温度ADC通道平均值 */
    p[2] = temp[2];                 /* 存入电流ADC通道平均值 */
}


void getcurrent_initADC(void)
{
    uint16_t t;
    g_motor_data.init_current_adc_val = g_adc_val [2]; /* 取出第一次得到的值 */ 
    for(t=0;t<1000;t++) 
    {  g_motor_data.init_current_adc_val += g_adc_val [2]; /* 现在的值和上一次存储的值相加 */ 
       g_motor_data.init_current_adc_val /= 2; /* 取平均值 */
       delay_us(500);
    }
}
void get_actualmoterdata(void)
{
    float temp_c = 0.0;                                                 
    temp_c = fabs(g_adc_val[2] - g_motor_data.init_current_adc_val) * ADC2CURT;
    g_motor_data.current = (float)((g_motor_data.current * (float)0.30) + ((float)0.70 * temp_c));  /* 一阶低通滤波 */
    if (g_motor_data.current <= 5)                                                                 /* 过滤掉微弱浮动电流 */
    {
        g_motor_data.current = 0.0;
    }
}

/*
 * @file        adc.c
 * @author    lis
 * @version     V1.2
 * @date        2021-10-18
 * @brief       ADC 驱动代码
 * @license     复成医疗
 * note ：
 * 二、   ADC3 数据采集  包含 10通道
 * (1) 推杆1~6  位置检测  PF5 PF3 PF4 PF6 PF8 PF7
 * (2) 左路电机电动势检测  PF9/PF10
 * (3) 右路电机电动势检测  PC0 PC1 
 */
#include "./SYSTEM/delay/delay.h"
#include "./BSP/ADC/adc.h"
#include "./BSP/R9/WheelSpeedMap.h"

ADC_HandleTypeDef g_adc3_nch_dma_handle; /* 与DMA2关联的ADC句柄 */
DMA_HandleTypeDef g_dma_nch_adc3_handle; /* 与ADC3关联的DMA句柄 */
uint16_t g_adc_value[ADC3_CH_NUM * ADC3_COLL] = {0};      /* 存储ADC原始值 */
uint16_t g_adc_val[ADC3_CH_NUM];                         /*ADC平均值存放数组*/

void adc_channel_set(ADC_HandleTypeDef *adc_handle, uint32_t ch, uint32_t rank, uint32_t stime)
{
    /* 配置对应ADC通道 */
    ADC_ChannelConfTypeDef adc_channel;
    adc_channel.Channel = ch;                        /* 设置ADCX对通道ch */
    adc_channel.Rank = rank;                         /* 设置采样序列 */
    adc_channel.SamplingTime = stime;                /* 设置采样时间 */
    HAL_ADC_ConfigChannel(adc_handle, &adc_channel); /* 初始化ADC通道 */
}

/***************************************多通道ADC3 采集(DMA读取)程序*****************************************/
void adc_init(void)
{
    g_adc3_nch_dma_handle.Instance = ADC_ADC3;
    g_adc3_nch_dma_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;            /* 4分频，ADCCLK = PCLK2/4 = 84/4 = 21Mhz */
    g_adc3_nch_dma_handle.Init.Resolution = ADC_RESOLUTION_12B;                      /* 12位模式 */
    g_adc3_nch_dma_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                      /* 右对齐 */
    g_adc3_nch_dma_handle.Init.ScanConvMode = ENABLE;                                /* 扫描模式 */
    g_adc3_nch_dma_handle.Init.ContinuousConvMode = ENABLE;                          /* 连续转换模式，转换完成之后接着继续转换 */
    g_adc3_nch_dma_handle.Init.DiscontinuousConvMode = DISABLE;                      /* 禁止不连续采样模式 */
    g_adc3_nch_dma_handle.Init.NbrOfConversion = ADC3_CH_NUM;                        /* 使用转换通道数，需根据实际转换通道去设置 */
    g_adc3_nch_dma_handle.Init.NbrOfDiscConversion = 0;                              /* 不连续采样通道数为0 */
    g_adc3_nch_dma_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                /* 软件触发 */
    g_adc3_nch_dma_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* 使用软件触发, 此位忽略 */
    g_adc3_nch_dma_handle.Init.DMAContinuousRequests = ENABLE;                       /* 开启DMA连续转换 */
    HAL_ADC_Init(&g_adc3_nch_dma_handle);                                            /* 初始化ADC */  
    /*电机电流采集通道配置*/                                                                                                                            
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_480CYCLES); /* 设置采样规则序列1~12 */
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_1, 2, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_2, 3, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_3, 4, ADC_SAMPLETIME_480CYCLES);
    /*电池及器件供电电压采集*/
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_5, 5, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_6, 6, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_7, 7, ADC_SAMPLETIME_480CYCLES); 
    /*电机电压采集*/
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_10, 8, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_11, 9, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_12, 10, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_13, 11, ADC_SAMPLETIME_480CYCLES);
}

/**
 * @brief       多通道ADC的gpio初始化函数
 * @param       无
 * @note        此函数会被adc_nch_dma_init()调用
 * (1) 推杆1~6  位置检测  PF5 PF3 PF4 PF6 PF8 PF7
 * (2) 左路电机电动势检测  PF9/PF10
 * (3) 右路电机电动势检测  PC0 PC1
* @retval       无
 */

void adc3_nch_dma_init(void)
{
    /* ADC3-DMA2stream1配置 */
    ADC_ADC3_CHY_CLK_ENABLE();                     /* 使能ADC3时钟 */
    if ((uint32_t)ADC_ADC3_DMASx > (uint32_t)DMA2) /* 大于DMA2_Stream7, 则为DMA2 */
    {
        __HAL_RCC_DMA2_CLK_ENABLE(); /* DMA2时钟使能 */
    }
    else
    {
        __HAL_RCC_DMA1_CLK_ENABLE(); /*   */
    }
    GPIO_InitTypeDef gpio_init_struct;
    __HAL_RCC_GPIOA_CLK_ENABLE(); /* 开启GPIOA引脚时钟 */
    __HAL_RCC_GPIOC_CLK_ENABLE(); /* 开启GPIOC引脚时钟 */
    __HAL_RCC_GPIOF_CLK_ENABLE(); /* 开启GPIOF引脚时钟 */
    /* AD采集引脚模式设置,模拟输入 A 接口 */
    gpio_init_struct.Pin =  GPIO_PIN_0 | GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3; /* GPIOC0~3 */
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    /* AD采集引脚模式设置,模拟输入 F  口  */
    gpio_init_struct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 ;
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);
    /* AD采集引脚模式设置,模拟输入 C 接口 */
    gpio_init_struct.Pin =  GPIO_PIN_0 | GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3; /* GPIOC0~3 */
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);
    adc_init();
    /*DMA的配置*/
    g_dma_nch_adc3_handle.Instance = ADC_ADC3_DMASx;                          /* 设置DMA数据流 */
    g_dma_nch_adc3_handle.Init.Channel = DMA_CHANNEL_2;                       /* 设置DMA通道 */
    g_dma_nch_adc3_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;              /* DIR = 1 , 外设到存储器模式 */
    g_dma_nch_adc3_handle.Init.PeriphInc = DMA_PINC_DISABLE;                  /* 外设非增量模式 */
    g_dma_nch_adc3_handle.Init.MemInc = DMA_MINC_ENABLE;                      /* 存储器增量模式 */
    g_dma_nch_adc3_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* 外设数据长度:16位 */
    g_dma_nch_adc3_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    /* 存储器数据长度:16位 */
    g_dma_nch_adc3_handle.Init.Mode = DMA_NORMAL;                             /* 外设流控模式 */
    g_dma_nch_adc3_handle.Init.Priority = DMA_PRIORITY_MEDIUM;                /* 中等优先级 */
    HAL_DMA_Init(&g_dma_nch_adc3_handle);                                     /* 初始化DMA */
    __HAL_LINKDMA(&g_adc3_nch_dma_handle,DMA_Handle,g_dma_nch_adc3_handle);       /* 把ADC和DMA关联，用DMA传输ADC数据 */
    /*ADC DMA中断配置*/
    HAL_NVIC_SetPriority(ADC_ADC3_DMASx_IRQn, 2, 1);                   /* 设置DMA中断优先级为3，子优先级为3 */
    HAL_NVIC_EnableIRQ(ADC_ADC3_DMASx_IRQn);                           /* 使能DMA中断 */
    HAL_ADC_Start_DMA(&g_adc3_nch_dma_handle,(uint32_t *)g_adc_value, ADC3_SUM);  /* 开启ADC的DMA传输 */

}


/**
 * @brief       ADC3通道  DMA2数据流1 采集中断服务函数
 * @param       无
 * @retval      无
 */
void ADC_ADC3_DMASx_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_dma_nch_adc3_handle);
}


/**
 * @brief       ADC 采集中断服务回调函数
 * @param       无 
 * @retval      无
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC_ADC3)
    {
        HAL_ADC_Stop_DMA(&g_adc3_nch_dma_handle);  /* 关闭DMA转换 */
        calc_adc_val(g_adc_val);                  /* 计算ADC的平均值 */

        /*供电电压采集*/
        g_r9sys_data.r9_battary_v= g_adc_val[5] * ADC2BATTV;
        g_r9sys_data.r9_10v= g_adc_val[4] * ADC2R9_10V;
        g_r9sys_data.r9_15v= g_adc_val[6] * ADC2R9_15V;
        HAL_ADC_Start_DMA(&g_adc3_nch_dma_handle, (uint32_t *)&g_adc_value, (uint32_t)(ADC3_SUM)); /* 启动DMA转换 */
    }
}

void calc_adc_val(uint16_t * p)
{
    uint32_t temp[11] = {0,0,0,0,0,0,0,0,0,0,0};
    int i;
    for(i=0;i<ADC3_COLL;i++)         /* 循环ADC_COLL次取值，累加 */
    {
        temp[0] += g_adc_value[0+i*ADC3_CH_NUM];
        temp[1] += g_adc_value[1+i*ADC3_CH_NUM];
        temp[2] += g_adc_value[2+i*ADC3_CH_NUM];
        temp[3] += g_adc_value[3+i*ADC3_CH_NUM];
        temp[4] += g_adc_value[4+i*ADC3_CH_NUM];
        temp[5] += g_adc_value[5+i*ADC3_CH_NUM];
        temp[6] += g_adc_value[6+i*ADC3_CH_NUM];
        temp[7] += g_adc_value[7+i*ADC3_CH_NUM];
        temp[8] += g_adc_value[8+i*ADC3_CH_NUM];
        temp[9] += g_adc_value[9+i*ADC3_CH_NUM];
        temp[10] += g_adc_value[10+i*ADC3_CH_NUM];  
    }
    temp[0] /= ADC3_COLL;            /* 取平均值 */
    temp[1] /= ADC3_COLL;
    temp[2] /= ADC3_COLL;
    temp[3] /= ADC3_COLL;            /* 取平均值 */
    temp[4] /= ADC3_COLL;
    temp[5] /= ADC3_COLL;
    temp[6] /= ADC3_COLL;            /* 取平均值 */
    temp[7] /= ADC3_COLL;
    temp[8] /= ADC3_COLL;
    temp[9] /= ADC3_COLL;            /* 取平均值 */
    temp[10] /= ADC3_COLL;
    p[0]  =  temp[0] ;                /* 接口A回路电压 2 */
    p[1]  =  temp[1] ;                /* 接口A回路电流 1*/
    p[2]  =  temp[2] ;                /* 接口A回路电流 2 */
    p[3]  =  temp[3] ;                /* 接口A回路电压 1*/
    p[4]  =  temp[4] ;                /* 10V电压ADC通道平均值 */
    p[5]  =  temp[5] ;                /* 电池电压ADC通道平均值 */
    p[6]  =  temp[6] ;                /*15V电压ADC通道平均值 */
    p[7]  =  temp[7] ;                /* 接口B回路电流 1 */
    p[8]  =  temp[8] ;                /* 接口B回路电流 2 */
    p[9]  =  temp[9] ;                /* 接口B回路电压 1 */
    p[10] =  temp[10];                /* 接口B回路电压 2*/
}

/*
 * @file        adc.c
 * @author    lis
 * @version     V1.2
 * @date        2021-10-18
 * @brief       ADC ��������
 * @license     ����ҽ��
 * note ��
 * ����   ADC3 ���ݲɼ�  ���� 10ͨ��
 * (1) �Ƹ�1~6  λ�ü��  PF5 PF3 PF4 PF6 PF8 PF7
 * (2) ��·����綯�Ƽ��  PF9/PF10
 * (3) ��·����綯�Ƽ��  PC0 PC1 
 */
#include "./SYSTEM/delay/delay.h"
#include "./BSP/ADC/adc.h"
#include "./BSP/R9/WheelSpeedMap.h"

ADC_HandleTypeDef g_adc3_nch_dma_handle; /* ��DMA2������ADC��� */
DMA_HandleTypeDef g_dma_nch_adc3_handle; /* ��ADC3������DMA��� */
uint16_t g_adc_value[ADC3_CH_NUM * ADC3_COLL] = {0};      /* �洢ADCԭʼֵ */
uint16_t g_adc_val[ADC3_CH_NUM];                         /*ADCƽ��ֵ�������*/

void adc_channel_set(ADC_HandleTypeDef *adc_handle, uint32_t ch, uint32_t rank, uint32_t stime)
{
    /* ���ö�ӦADCͨ�� */
    ADC_ChannelConfTypeDef adc_channel;
    adc_channel.Channel = ch;                        /* ����ADCX��ͨ��ch */
    adc_channel.Rank = rank;                         /* ���ò������� */
    adc_channel.SamplingTime = stime;                /* ���ò���ʱ�� */
    HAL_ADC_ConfigChannel(adc_handle, &adc_channel); /* ��ʼ��ADCͨ�� */
}

/***************************************��ͨ��ADC3 �ɼ�(DMA��ȡ)����*****************************************/
void adc_init(void)
{
    g_adc3_nch_dma_handle.Instance = ADC_ADC3;
    g_adc3_nch_dma_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;            /* 4��Ƶ��ADCCLK = PCLK2/4 = 84/4 = 21Mhz */
    g_adc3_nch_dma_handle.Init.Resolution = ADC_RESOLUTION_12B;                      /* 12λģʽ */
    g_adc3_nch_dma_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                      /* �Ҷ��� */
    g_adc3_nch_dma_handle.Init.ScanConvMode = ENABLE;                                /* ɨ��ģʽ */
    g_adc3_nch_dma_handle.Init.ContinuousConvMode = ENABLE;                          /* ����ת��ģʽ��ת�����֮����ż���ת�� */
    g_adc3_nch_dma_handle.Init.DiscontinuousConvMode = DISABLE;                      /* ��ֹ����������ģʽ */
    g_adc3_nch_dma_handle.Init.NbrOfConversion = ADC3_CH_NUM;                        /* ʹ��ת��ͨ�����������ʵ��ת��ͨ��ȥ���� */
    g_adc3_nch_dma_handle.Init.NbrOfDiscConversion = 0;                              /* ����������ͨ����Ϊ0 */
    g_adc3_nch_dma_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                /* ������� */
    g_adc3_nch_dma_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* ʹ���������, ��λ���� */
    g_adc3_nch_dma_handle.Init.DMAContinuousRequests = ENABLE;                       /* ����DMA����ת�� */
    HAL_ADC_Init(&g_adc3_nch_dma_handle);                                            /* ��ʼ��ADC */  
    /*��������ɼ�ͨ������*/                                                                                                                            
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_480CYCLES); /* ���ò�����������1~12 */
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_1, 2, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_2, 3, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_3, 4, ADC_SAMPLETIME_480CYCLES);
    /*��ؼ����������ѹ�ɼ�*/
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_5, 5, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_6, 6, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_7, 7, ADC_SAMPLETIME_480CYCLES); 
    /*�����ѹ�ɼ�*/
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_10, 8, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_11, 9, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_12, 10, ADC_SAMPLETIME_480CYCLES);
    adc_channel_set(&g_adc3_nch_dma_handle, ADC_CHANNEL_13, 11, ADC_SAMPLETIME_480CYCLES);
}

/**
 * @brief       ��ͨ��ADC��gpio��ʼ������
 * @param       ��
 * @note        �˺����ᱻadc_nch_dma_init()����
 * (1) �Ƹ�1~6  λ�ü��  PF5 PF3 PF4 PF6 PF8 PF7
 * (2) ��·����綯�Ƽ��  PF9/PF10
 * (3) ��·����綯�Ƽ��  PC0 PC1
* @retval       ��
 */

void adc3_nch_dma_init(void)
{
    /* ADC3-DMA2stream1���� */
    ADC_ADC3_CHY_CLK_ENABLE();                     /* ʹ��ADC3ʱ�� */
    if ((uint32_t)ADC_ADC3_DMASx > (uint32_t)DMA2) /* ����DMA2_Stream7, ��ΪDMA2 */
    {
        __HAL_RCC_DMA2_CLK_ENABLE(); /* DMA2ʱ��ʹ�� */
    }
    else
    {
        __HAL_RCC_DMA1_CLK_ENABLE(); /*   */
    }
    GPIO_InitTypeDef gpio_init_struct;
    __HAL_RCC_GPIOA_CLK_ENABLE(); /* ����GPIOA����ʱ�� */
    __HAL_RCC_GPIOC_CLK_ENABLE(); /* ����GPIOC����ʱ�� */
    __HAL_RCC_GPIOF_CLK_ENABLE(); /* ����GPIOF����ʱ�� */
    /* AD�ɼ�����ģʽ����,ģ������ A �ӿ� */
    gpio_init_struct.Pin =  GPIO_PIN_0 | GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3; /* GPIOC0~3 */
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);
    /* AD�ɼ�����ģʽ����,ģ������ F  ��  */
    gpio_init_struct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 ;
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);
    /* AD�ɼ�����ģʽ����,ģ������ C �ӿ� */
    gpio_init_struct.Pin =  GPIO_PIN_0 | GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3; /* GPIOC0~3 */
    gpio_init_struct.Mode = GPIO_MODE_ANALOG;
    gpio_init_struct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio_init_struct);
    adc_init();
    /*DMA������*/
    g_dma_nch_adc3_handle.Instance = ADC_ADC3_DMASx;                          /* ����DMA������ */
    g_dma_nch_adc3_handle.Init.Channel = DMA_CHANNEL_2;                       /* ����DMAͨ�� */
    g_dma_nch_adc3_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;              /* DIR = 1 , ���赽�洢��ģʽ */
    g_dma_nch_adc3_handle.Init.PeriphInc = DMA_PINC_DISABLE;                  /* ���������ģʽ */
    g_dma_nch_adc3_handle.Init.MemInc = DMA_MINC_ENABLE;                      /* �洢������ģʽ */
    g_dma_nch_adc3_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; /* �������ݳ���:16λ */
    g_dma_nch_adc3_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    /* �洢�����ݳ���:16λ */
    g_dma_nch_adc3_handle.Init.Mode = DMA_NORMAL;                             /* ��������ģʽ */
    g_dma_nch_adc3_handle.Init.Priority = DMA_PRIORITY_MEDIUM;                /* �е����ȼ� */
    HAL_DMA_Init(&g_dma_nch_adc3_handle);                                     /* ��ʼ��DMA */
    __HAL_LINKDMA(&g_adc3_nch_dma_handle,DMA_Handle,g_dma_nch_adc3_handle);       /* ��ADC��DMA��������DMA����ADC���� */
    /*ADC DMA�ж�����*/
    HAL_NVIC_SetPriority(ADC_ADC3_DMASx_IRQn, 2, 1);                   /* ����DMA�ж����ȼ�Ϊ3�������ȼ�Ϊ3 */
    HAL_NVIC_EnableIRQ(ADC_ADC3_DMASx_IRQn);                           /* ʹ��DMA�ж� */
    HAL_ADC_Start_DMA(&g_adc3_nch_dma_handle,(uint32_t *)g_adc_value, ADC3_SUM);  /* ����ADC��DMA���� */

}


/**
 * @brief       ADC3ͨ��  DMA2������1 �ɼ��жϷ�����
 * @param       ��
 * @retval      ��
 */
void ADC_ADC3_DMASx_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_dma_nch_adc3_handle);
}


/**
 * @brief       ADC �ɼ��жϷ���ص�����
 * @param       �� 
 * @retval      ��
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC_ADC3)
    {
        HAL_ADC_Stop_DMA(&g_adc3_nch_dma_handle);  /* �ر�DMAת�� */
        calc_adc_val(g_adc_val);                  /* ����ADC��ƽ��ֵ */

        /*�����ѹ�ɼ�*/
        g_r9sys_data.r9_battary_v= g_adc_val[5] * ADC2BATTV;
        g_r9sys_data.r9_10v= g_adc_val[4] * ADC2R9_10V;
        g_r9sys_data.r9_15v= g_adc_val[6] * ADC2R9_15V;
        HAL_ADC_Start_DMA(&g_adc3_nch_dma_handle, (uint32_t *)&g_adc_value, (uint32_t)(ADC3_SUM)); /* ����DMAת�� */
    }
}

void calc_adc_val(uint16_t * p)
{
    uint32_t temp[11] = {0,0,0,0,0,0,0,0,0,0,0};
    int i;
    for(i=0;i<ADC3_COLL;i++)         /* ѭ��ADC_COLL��ȡֵ���ۼ� */
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
    temp[0] /= ADC3_COLL;            /* ȡƽ��ֵ */
    temp[1] /= ADC3_COLL;
    temp[2] /= ADC3_COLL;
    temp[3] /= ADC3_COLL;            /* ȡƽ��ֵ */
    temp[4] /= ADC3_COLL;
    temp[5] /= ADC3_COLL;
    temp[6] /= ADC3_COLL;            /* ȡƽ��ֵ */
    temp[7] /= ADC3_COLL;
    temp[8] /= ADC3_COLL;
    temp[9] /= ADC3_COLL;            /* ȡƽ��ֵ */
    temp[10] /= ADC3_COLL;
    p[0]  =  temp[0] ;                /* �ӿ�A��·��ѹ 2 */
    p[1]  =  temp[1] ;                /* �ӿ�A��·���� 1*/
    p[2]  =  temp[2] ;                /* �ӿ�A��·���� 2 */
    p[3]  =  temp[3] ;                /* �ӿ�A��·��ѹ 1*/
    p[4]  =  temp[4] ;                /* 10V��ѹADCͨ��ƽ��ֵ */
    p[5]  =  temp[5] ;                /* ��ص�ѹADCͨ��ƽ��ֵ */
    p[6]  =  temp[6] ;                /*15V��ѹADCͨ��ƽ��ֵ */
    p[7]  =  temp[7] ;                /* �ӿ�B��·���� 1 */
    p[8]  =  temp[8] ;                /* �ӿ�B��·���� 2 */
    p[9]  =  temp[9] ;                /* �ӿ�B��·��ѹ 1 */
    p[10] =  temp[10];                /* �ӿ�B��·��ѹ 2*/
}


#include "./SYSTEM/delay/delay.h"
#include "./BSP/ADC/adc.h"
#include "./BSP/R9/moterdriver.h"
/* ��ͨ��ADC�ɼ� DMA��ȡ */
ADC_HandleTypeDef g_adc_nch_dma_handle;                 /* ��DMA������ADC��� */
DMA_HandleTypeDef g_dma_nch_adc_handle;                 /* ��ADC������DMA��� */
uint16_t g_adc_value[ADC_CH_NUM * ADC_COLL] = {0};      /* �洢ADCԭʼֵ */

/***************************************  ��ѹ���������¶� ��ͨ��ADC�ɼ�(DMA��ȡ)����*****************************************/

/**
 * @brief       ADC��ʼ������
 * @param       ��
 * @retval      ��
 */
void adc_init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    g_adc_nch_dma_handle.Instance = ADC_ADCX;                                       /* ADCx */
    g_adc_nch_dma_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;            /* 4��Ƶ��ADCCLK = PCLK2/4 = 84/4 = 21Mhz */
    g_adc_nch_dma_handle.Init.Resolution = ADC_RESOLUTION_12B;                      /* 12λģʽ */
    g_adc_nch_dma_handle.Init.ScanConvMode = ENABLE;                                /* ɨ��ģʽ ��ͨ��ʹ�� */
    g_adc_nch_dma_handle.Init.ContinuousConvMode = ENABLE;                          /* ����ת��ģʽ��ת�����֮����ż���ת�� */
    g_adc_nch_dma_handle.Init.DiscontinuousConvMode = DISABLE;                      /* ��ֹ����������ģʽ */
    g_adc_nch_dma_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* ʹ��������� */
    g_adc_nch_dma_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                /* ������� */
    g_adc_nch_dma_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                      /* �Ҷ��� */
    g_adc_nch_dma_handle.Init.NbrOfConversion = ADC_CH_NUM;                         /* ʹ��ת��ͨ�����������ʵ��ת��ͨ��ȥ���� */
    g_adc_nch_dma_handle.Init.DMAContinuousRequests = ENABLE;                       /* ����DMA����ת������ */
    g_adc_nch_dma_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&g_adc_nch_dma_handle);

    /* ����ʹ�õ�ADCͨ��������������ĵڼ���ת�������ӻ��߼���ͨ����Ҫ�޸��ⲿ�� */
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
 * @brief       ADC DMA��ȡ ��ʼ������
 *   @note      ����������ʹ��adc_init��ADC���д󲿷�����,�в���ĵط��ٵ�������
 * @param       par         : �����ַ
 * @param       mar         : �洢����ַ
 * @retval      ��
 */
void adc_nch_dma_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
      
    ADC_ADCX_CHY_CLK_ENABLE();                                  /* ʹ��ADCxʱ�� */
    ADC_ADCX_CH0_GPIO_CLK_ENABLE();                             /* ����GPIOʱ�� */
    ADC_ADCX_CH1_GPIO_CLK_ENABLE();
    ADC_ADCX_CH2_GPIO_CLK_ENABLE();
    
    /* AD�ɼ�����ģʽ����,ģ������ */
    GPIO_InitStruct.Pin = ADC_ADCX_CH0_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ADC_ADCX_CH0_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ADC_ADCX_CH1_GPIO_PIN;
    HAL_GPIO_Init(ADC_ADCX_CH1_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = ADC_ADCX_CH2_GPIO_PIN;   
    HAL_GPIO_Init(ADC_ADCX_CH2_GPIO_PORT, &GPIO_InitStruct); 
    
    adc_init();                                                 /* �ȳ�ʼ��ADC */
    
    if ((uint32_t)ADC_ADCX_DMASx > (uint32_t)DMA2)              /* ����DMA1_Channel7, ��ΪDMA2��ͨ���� */
    {
        __HAL_RCC_DMA2_CLK_ENABLE();                            /* DMA2ʱ��ʹ�� */
    }
    else 
    {
        __HAL_RCC_DMA1_CLK_ENABLE();                            /* DMA1ʱ��ʹ�� */
    }

    /* DMA���� */
    g_dma_nch_adc_handle.Instance = ADC_ADCX_DMASx;                             /* ������x */
    g_dma_nch_adc_handle.Init.Channel = ADC_ADCX_DMASx_Chanel;                  /* DMAͨ��x */
    g_dma_nch_adc_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;                 /* DIR = 1 ,���赽�洢��ģʽ */
    g_dma_nch_adc_handle.Init.PeriphInc = DMA_PINC_DISABLE;                     /* ���������ģʽ */
    g_dma_nch_adc_handle.Init.MemInc = DMA_MINC_ENABLE;                         /* �洢������ģʽ */
    g_dma_nch_adc_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;    /* �������ݳ���:16λ */
    g_dma_nch_adc_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;       /* �洢�����ݳ���:16λ */
    g_dma_nch_adc_handle.Init.Mode = DMA_CIRCULAR;                              /* ��������ģʽ */
    g_dma_nch_adc_handle.Init.Priority = DMA_PRIORITY_MEDIUM;                   /* �е����ȼ� */
    HAL_DMA_Init(&g_dma_nch_adc_handle);
 
    __HAL_LINKDMA(&g_adc_nch_dma_handle,DMA_Handle,g_dma_nch_adc_handle);       /* ��ADC��DMA��������DMA����ADC���� */
    
    /* ADC DMA�ж����� */
    HAL_NVIC_SetPriority(ADC_ADCX_DMASx_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(ADC_ADCX_DMASx_IRQn);
    
    HAL_ADC_Start_DMA(&g_adc_nch_dma_handle,(uint32_t *)g_adc_value,ADC_SUM);    /* ����ADC��DMA���� */
}

/**
 * @brief       DMA2 ������4�жϷ�����
 * @param       �� 
 * @retval      ��
 */
void ADC_ADCX_DMASx_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_dma_nch_adc_handle);
}

uint16_t g_adc_val[ADC_CH_NUM];                   /*ADCƽ��ֵ�������*/
/**
 * @brief       ADC�ɼ��жϷ���ص�����
 * @param       �� 
 * @retval      ��
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    float temp_c = 0.0;
    static float add_adc = 0;
    static float init_adc_value = 0;
    static uint8_t adc_count1 = 0, adc_count2 = 0;
    if (hadc->Instance == ADC_ADCX)
    {
        HAL_ADC_Stop_DMA(&g_adc_nch_dma_handle);  /*�ر�DMAת��*/
        calc_adc_val(g_adc_val);                  /*����ADC��ƽ��ֵ*/
        g_motor_data.volatage =  g_adc_val[0] * ADC2VBUS-45.88; // �����Դ��ѹ
    }   
    HAL_ADC_Start_DMA(&g_adc_nch_dma_handle, (uint32_t *)&g_adc_value, (uint32_t)(ADC_SUM)); /*����DMAת��*/
}


/**
 * @brief       ����ADC��ƽ��ֵ���˲���
 * @param       * p �����ADCֵ��ָ���ַ
 * @note        �˺����Ե�ѹ���¶ȡ�������Ӧ��ADCֵ�����˲���
 *              p[0]-p[2]��Ӧ�ķֱ��ǵ�ѹ���¶Ⱥ͵���
 * @retval      ��
 */
void calc_adc_val(uint16_t * p)
{
    uint32_t temp[3] = {0,0,0};
    int i;
    for(i=0;i<ADC_COLL;i++)         /* ѭ��ADC_COLL��ȡֵ���ۼ� */
    {
        temp[0] += g_adc_value[0+i*ADC_CH_NUM];
        temp[1] += g_adc_value[1+i*ADC_CH_NUM];
        temp[2] += g_adc_value[2+i*ADC_CH_NUM];
    }
    temp[0] /= ADC_COLL;            /* ȡƽ��ֵ */
    temp[1] /= ADC_COLL;
    temp[2] /= ADC_COLL;
    p[0] = temp[0];                 /* �����ѹADCͨ��ƽ��ֵ */
    p[1] = temp[1];                 /* �����¶�ADCͨ��ƽ��ֵ */
    p[2] = temp[2];                 /* �������ADCͨ��ƽ��ֵ */
}


void getcurrent_initADC(void)
{
    uint16_t t;
    g_motor_data.init_current_adc_val = g_adc_val [2]; /* ȡ����һ�εõ���ֵ */ 
    for(t=0;t<1000;t++) 
    {  g_motor_data.init_current_adc_val += g_adc_val [2]; /* ���ڵ�ֵ����һ�δ洢��ֵ��� */ 
       g_motor_data.init_current_adc_val /= 2; /* ȡƽ��ֵ */
       delay_us(500);
    }
}
void get_actualmoterdata(void)
{
    float temp_c = 0.0;                                                 
    temp_c = fabs(g_adc_val[2] - g_motor_data.init_current_adc_val) * ADC2CURT;
    g_motor_data.current = (float)((g_motor_data.current * (float)0.30) + ((float)0.70 * temp_c));  /* һ�׵�ͨ�˲� */
    if (g_motor_data.current <= 5)                                                                 /* ���˵�΢���������� */
    {
        g_motor_data.current = 0.0;
    }
}

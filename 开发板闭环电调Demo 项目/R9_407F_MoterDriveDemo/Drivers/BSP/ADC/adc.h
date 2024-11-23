#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx.h"

/***************************************  电压、电流、温度 多通道ADC采集(DMA读取)*****************************************/
/* ADC及引脚 定义 */

#define ADC_ADCX_CH0_GPIO_PORT              GPIOB
#define ADC_ADCX_CH0_GPIO_PIN               GPIO_PIN_1
#define ADC_ADCX_CH0_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)         /* PB口时钟使能 */

#define ADC_ADCX_CH1_GPIO_PORT              GPIOA
#define ADC_ADCX_CH1_GPIO_PIN               GPIO_PIN_0
#define ADC_ADCX_CH1_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)         /* PA口时钟使能 */

#define ADC_ADCX_CH2_GPIO_PORT              GPIOB
#define ADC_ADCX_CH2_GPIO_PIN               GPIO_PIN_0
#define ADC_ADCX_CH2_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)         /* PB口时钟使能 */

#define ADC_ADCX                            ADC1 
#define ADC_ADCX_CH0                        ADC_CHANNEL_9                                       /* 通道Y,  0 <= Y <= 17 */ 
#define ADC_ADCX_CH1                        ADC_CHANNEL_0
#define ADC_ADCX_CH2                        ADC_CHANNEL_8

#define ADC_ADCX_CHY_CLK_ENABLE()           do{ __HAL_RCC_ADC1_CLK_ENABLE(); }while(0)          /* ADC1 时钟使能 */

#define ADC_CH_NUM                          3                                                   /* 需要转换的通道数目 */
#define ADC_COLL                            1200                                                /* 单采集次数 */
#define ADC_SUM                             ADC_CH_NUM * ADC_COLL                               /* 总采集次数 */

/* ADC单通道/多通道 DMA采集 DMA数据流相关 定义
 * 注意: 这里我们的通道还是使用上面的定义.
 */
#define ADC_ADCX_DMASx                      DMA2_Stream4                                        /* 数据流4 */
#define ADC_ADCX_DMASx_Chanel               DMA_CHANNEL_0                                       /* 通道0 */
#define ADC_ADCX_DMASx_IRQn                 DMA2_Stream4_IRQn
#define ADC_ADCX_DMASx_IRQHandler           DMA2_Stream4_IRQHandler

#define ADC_ADCX_DMASx_IS_TC()              ( __HAL_DMA_GET_FLAG(&g_dma_nch_adc_handle, DMA_FLAG_TCIF0_4) )                /* 判断DMA2 Stream4传输完成标志, 这是一个假函数形式,
                                                                                                                            * 不能当函数使用, 只能用在if等语句里面 
                                                                                                                            */
#define ADC_ADCX_DMASx_CLR_TC()             do{ __HAL_DMA_CLEAR_FLAG(&g_dma_nch_adc_handle, DMA_FLAG_TCIF0_4); }while(0)   /* 清除DMA2 Stream4传输完成标志 */

/******************************************************************************************/
extern uint16_t g_adc_value[ADC_CH_NUM * ADC_COLL];

/* 电流计算公式：
 * I=（当前工作电压-初始参考电压）/（6*0.02）
 * ADC值转换为电压值：电压=ADC值*3.3/4096，这里电压单位为V，我们换算成mV,4096/1000=4.096，后面就直接算出为mA
 * 整合公式可以得出电流 I= （当前ADC值-初始ADC值）* （3.3 / 4.096 / 0.12）
 */
#define ADC2CURT    5.0354

/* 电压计算公式：
 * V_POWER = V_BUS * 25
 * ADC值转换为电压值：电压=ADC值*3.3/4096
 * 整合公式可以得出电压V_POWER= ADC值 *（3.3f * 25 / 4096）
 */
#define ADC2VBUS    (float)(3.3f * 37.0 / 4096)

void adc_init(void);                                            /* ADC初始化 */
void adc_nch_dma_init(void);                                    /* ADC DMA传输 初始化函数 */
void calc_adc_val(uint16_t * p);
void getcurrent_initADC(void);
void get_actualmoterdata(void);
#endif 

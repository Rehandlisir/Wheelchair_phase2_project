/**
 ****************************************************************************************************
 * @file        adc.h
 * @author      R9()
 * @version     V1.2
 * @date        2021-10-18
 * @brief       ADC 驱动代码
 * @license     
 ****************************************************************************************************
 * @attention
 *
 *
 * 针对 R9系统的所有ADC 数据采集 ，
 * 一、   ADC1 数据采集  包含 8 通道
 * (1) PA0 24V 灯线检测
 * (2) PA1 12V 灯线检测
 * (3) PA2 PA3 ADCX ADCY 
   (4) PC2 PC3 左路/右路电机电流检测
   (5) PC4     电池电量检测
   (6) PC5     充电状态检测
 * 二、   ADC3 数据采集  包含 10通道
 * (1) 推杆1~6  位置检测  PF5 PF3 PF4 PF6 PF8 PF7
 * (2) 左路电机电动势检测  PF9/PF10
 * (3) 右路电机电动势检测  PC0 PC1 
 * 
 ****************************************************************************************************
 */

#ifndef __ADC_H
#define __ADC_H

#include "./SYSTEM/sys/sys.h"


/************************************ ADC1 及引脚 定义 ******************************************************/

#define ADC_ADC1                            ADC1 /* 通道Y,  0 <= Y <= 17 */                                 
#define ADC_ADC1_CHY_CLK_ENABLE()           do{ __HAL_RCC_ADC1_CLK_ENABLE(); }while(0)      /* ADC1 时钟使能 */


#define ADC1_CH_NUM                          11                                               /* 需要转换的通道数目 */

/* ADC1 多通道 DMA2 Stream4 采集 DMA数据流相关 定义
 * 注意: 这里我们的通道还是使用上面的定义.
 */
#define ADC_ADC1_DMASx                      DMA2_Stream4
#define ADC_ADC1_DMASx_Chanel               DMA_CHANNEL_0                                   /* ADC1_DMA请求源 */
#define ADC_ADC1_DMASx_IRQn                 DMA2_Stream4_IRQn
#define ADC_ADC1_DMASx_IRQHandler           DMA2_Stream4_IRQHandler

#define ADC_ADC1_DMASx_IS_TC()              ( DMA2->HISR & (1 << 5) )                       /* 判断 DMA2_Stream4 传输完成标志, 这是一个假函数形式,不能当函数使用, 只能用在if等语句里面 */
#define ADC_ADC1_DMASx_CLR_TC()             do{ DMA2->HIFCR |= 1 << 5; }while(0)            /* 清除 DMA2_Stream4 传输完成标志 */


/************************************ADC3 及引脚 定义******************************************************/
/*  */
#define ADC_ADC3                            ADC3 /* 通道Y,  0 <= Y <= 17 */                                 
#define ADC_ADC3_CHY_CLK_ENABLE()           do{ __HAL_RCC_ADC3_CLK_ENABLE(); }while(0)      /* ADC1 时钟使能 */


#define ADC3_CH_NUM                          10                                               /* 需要转换的通道数目 */

/* ADC3 多通道 DMA2 Stream1 采集 DMA数据流相关 定义
 * 注意: 这里我们的通道还是使用上面的定义.
 */
#define ADC_ADC3_DMASx                      DMA2_Stream1
#define ADC_ADC3_DMASx_Chanel               DMA_CHANNEL_2                                   /* ADC3_DMA请求源 */
#define ADC_ADC3_DMASx_IRQn                 DMA2_Stream1_IRQn
#define ADC_ADC3_DMASx_IRQHandler           DMA2_Stream1_IRQHandler

#define ADC_ADC3_DMASx_IS_TC()              ( DMA2->LISR & (1 << 11) )                       /* 判断 DMA2_Stream1 传输完成标志, 这是一个假函数形式,不能当函数使用, 只能用在if等语句里面 */
#define ADC_ADC3_DMASx_CLR_TC()             do{ DMA2->LIFCR |= 1 << 11; }while(0)            /* 清除 DMA2_Stream1 传输完成标志 */

/******************************************************************************************/

#define ADC2VOLATAGE         37.3/4096.0  // 电压换算
#define SAMP_RA              1.8e-3       //采样电阻
/*电流与采样电压换算关系 Vout = 2.868 *Vin + 0.9892 => 实际电流 与采样电压换算关系 I = (AD/4096 *3.3 - 0.9892）/0.0051624 ; */
#define ADC2BATTV            (254.0*3.3)/(4096.0*22.0) // 电池电压换算系数
#define ADC2R9_10V           25.3/4096.0 // 10v电压换算系数
#define ADC2R9_15V           (41.2*3.3)/(4096.0*8.2) // 15v电压换算系数
#define MOTER_RA             0.06                 // 实际0.08 取其 70% 电机内阻                      
#define MOTER_CE             0.133// 0.1073298V     // 

void adc_channel_set(ADC_HandleTypeDef *adc_handle, uint32_t ch, uint32_t rank, uint32_t stime);    /* ADC通道设置 */
void adc1_nch_dma_init(uint32_t tmr);        /* ADC1多通道 DMA采集初始化 */
void adc1_nch_dma_gpio_init(void);           /* ADC1多通道 GPIO初始化 */
void adc1_nch_dma_enable(uint16_t ndtr);     /* 使能一次ADC1 DMA多通道采集传输 */
void adc3_nch_dma_init(uint32_t tmr);        /* ADC3多通道 DMA采集初始化 */
void adc3_nch_dma_gpio_init(void);           /* ADC3多通道 GPIO初始化 */
void adc3_nch_dma_enable(uint16_t ndtr);     /* 使能一次ADC3 DMA多通道采集传输 */

#endif 






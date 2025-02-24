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

/************************************ADC3 及引脚 定义******************************************************/
/*  */
#define ADC_ADC3                            ADC3 /* 通道Y,  0 <= Y <= 17 */                                 
#define ADC_ADC3_CHY_CLK_ENABLE()           do{ __HAL_RCC_ADC3_CLK_ENABLE(); }while(0)      /* ADC1 时钟使能 */


#define ADC3_CH_NUM                           11        /* 需要转换的通道数目 */
#define ADC3_COLL                             5                                                /* 单采集次数 */
#define ADC3_SUM                              ADC3_CH_NUM * ADC3_COLL                               /* 总采集次数 */
extern uint16_t  g_adc_val[ADC3_CH_NUM];

#define ADC2VOLATAGE         37.3/4096.0  // 电压换算
#define SAMP_RA              1.8e-3       //采样电阻
/*电流与采样电压换算关系 Vout = 2.868 *Vin + 0.9892 => 实际电流 与采样电压换算关系 I = (AD/4096 *3.3 - 0.9892）/0.0051624 ; */
#define ADC2BATTV            (254.0*3.3)/(4096.0*22.0) // 电池电压换算系数
#define ADC2R9_10V           25.3/4096.0 // 10v电压换算系数
#define ADC2R9_15V           (41.2*3.3)/(4096.0*8.2) // 15v电压换算系数
#define MOTER_RA             0.08                  // 电机内阻                      
#define MOTER_CE             0.1073298// 0.1073298V     // 捷和电机反电势系数


/* ADC3 多通道 DMA2 Stream1 采集 DMA数据流相关 定义
 * 注意: 这里我们的通道还是使用上面的定义.
 */
#define ADC_ADC3_DMASx                      DMA2_Stream1
#define ADC_ADC3_DMASx_Chanel               DMA_CHANNEL_2                                   /* ADC3_DMA请求源 */
#define ADC_ADC3_DMASx_IRQn                 DMA2_Stream1_IRQn
#define ADC_ADC3_DMASx_IRQHandler           DMA2_Stream1_IRQHandler

// #define ADC_ADC3_DMASx_IS_TC()              ( DMA2->LISR & (1 << 11) )                       /* 判断 DMA2_Stream1 传输完成标志, 这是一个假函数形式,不能当函数使用, 只能用在if等语句里面 */
// #define ADC_ADC3_DMASx_CLR_TC()             do{ DMA2->LIFCR |= 1 << 11; }while(0)            /* 清除 DMA2_Stream1 传输完成标志 */

/******************************************************************************************/

void adc_channel_set(ADC_HandleTypeDef *adc_handle, uint32_t ch, uint32_t rank, uint32_t stime);    /* ADC通道设置 */
void adc_init(void);     
void adc3_nch_dma_init(void);          
void calc_adc_val(uint16_t * p);

#endif 




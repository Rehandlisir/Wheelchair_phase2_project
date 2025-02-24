/**
 ****************************************************************************************************
 * @file        beep.h
 * @author      Lisir
 * @version     V1.0
 * @date        2021-10-14
 * @brief       �������� ��������
 * @license     Copyright (c) 2024, ���ڸ���ҽ�ƿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:F407������
 * none
 * none
 * none
 * none
 *
 * �޸�˵��
 * none
 * ��һ�η���
 *
 ****************************************************************************************************
 */

 #ifndef __COMMON_H
 #define __COMMON_H
 
 #include "./SYSTEM/sys/sys.h"
 #include "math.h"
 
 
 
 #define WINDOW_SIZE 100
 // 均值滤波器结构体
 typedef struct {
	 float window_float[WINDOW_SIZE];
	 int16_t window_int[WINDOW_SIZE];
	 uint8_t index;
 } AverageFilter;
 
 // 浮点型低通滤波器滤波器结构体
 typedef struct {
	 double alpha;  // 滤波系数
	 double output; // 滤波器输出
 } LowPassFilter;
 
 
 extern AverageFilter filter_LN;
 extern AverageFilter filter_RN;

 extern AverageFilter filter_lspeed;
 extern AverageFilter filter_rspeed;
 extern AverageFilter filter_ADCX;
 extern AverageFilter filter_ADCY;
 
 
 extern LowPassFilter lowpassl_speed;
 extern LowPassFilter lowpassr_speed;
 extern LowPassFilter lowpassl_volatage;
 extern LowPassFilter lowpassr_volatage;
 extern LowPassFilter lowpassl_current;
 extern LowPassFilter lowpassr_current;
 extern LowPassFilter lowpassy_ADC;
 extern LowPassFilter lowpassx_ADC;
 extern LowPassFilter lowpass_lspeedTarget;
 extern LowPassFilter lowpass_rspeedTarget;
 extern LowPassFilter filter_Lpwm;
 extern LowPassFilter filter_Rpwm;
 
 int16_t Value_limit(int16_t min_value, int16_t current_value, int16_t max_value);
 float Value_limitf(float min_value, float current_value, float max_value);
 void average_init(AverageFilter *filter);
 void initLowPassFilter(LowPassFilter *filter, double cutoffFrequency, double samplingFrequency) ;
 float  filterValue_float(AverageFilter *filter, float input);
 int16_t filterValue_int16(AverageFilter *filter, int16_t input);
 double lowPassFilter(LowPassFilter *filter, double input) ;
 void filterInit(void);
 void lowpass_init(void);
 
 uint16_t low_pass_filter1(uint16_t value,float alpha);
 uint16_t low_pass_filter2(uint16_t value,float alpha);
 uint16_t low_pass_filter3(uint16_t value,float alpha);
 uint16_t low_pass_filter4(uint16_t value,float alpha);
 uint16_t low_pass_filter5(uint16_t value,float alpha);
 uint16_t low_pass_filter6(uint16_t value,float alpha);
 uint16_t low_pass_filter7(uint16_t value,float alpha);
 int16_t low_pass_filter8(int16_t value,float alpha);
 int16_t low_pass_filter9(int16_t value,float alpha);
 #endif
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
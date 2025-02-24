/**
 * @FilePath     : /AS5013/JOYSTIC_HAL/Drivers/BSP/FILTER/filter.h
 * @Description  :  
 * @Author       : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-02-18 18:48:50
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
#ifndef __MLX90393_H_
#define __MLX90393_H_

#include "./SYSTEM/sys/sys.h"
 #define WINDOW_SIZE 50
 // 均值滤波器结构体
 typedef struct {
	 float window_float[WINDOW_SIZE];
	 int8_t window_int[WINDOW_SIZE];
	 uint8_t index;
 } AverageFilter;
 
 // 浮点型低通滤波器滤波器结构体
 typedef struct {
	 double alpha;  // 滤波系数
	 int8_t output; // 滤波器输出
 } LowPassFilter;
 
 
extern AverageFilter filter_ash5031_xdata;
extern AverageFilter filter_ash5031_ydata;

extern LowPassFilter lowpass_ash5031_xdata;
extern LowPassFilter lowpass_ash5031_ydata;


void average_init(AverageFilter *filter);
void initLowPassFilter(LowPassFilter *filter, double cutoffFrequency, double samplingFrequency) ;
float  filterValue_float(AverageFilter *filter, float input);
int8_t filterValue_int8(AverageFilter *filter, int8_t input);
double lowPassFilter(LowPassFilter *filter, double input) ;
void filterInit(void);
void lowpass_init(void);

#endif
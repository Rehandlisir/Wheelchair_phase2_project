/**
 * @FilePath     : /AS5013/JOYSTIC_HAL/Drivers/BSP/FILTER/filter.c
 * @Description  :  
 * @Author       : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-02-18 18:49:45
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
#include "./BSP/FILTER/filter.h"
AverageFilter filter_ash5031_xdata;
AverageFilter filter_ash5031_ydata;


/*滑动均值滤波器初始化*/
void average_init(AverageFilter *filter)
{
  for (int i = 0; i < WINDOW_SIZE; ++i)
  {
    filter->window_float[i] = 0;
    filter->window_int[i] = 0;
  }
  filter->index = 0;
}

// 初始化低通滤波器
void initLowPassFilter(LowPassFilter *filter, double cutoffFrequency, double samplingFrequency) 
{
    // 根据截止频率和采样频率计算滤波系数alpha
    filter->alpha = cutoffFrequency / (cutoffFrequency + samplingFrequency);
    filter->output = 0.0;
}

/*浮点数滑动均值滤波器*/
float filterValue_float(AverageFilter *filter, float input)
{
  // 更新缓存区
  filter->window_float[filter->index] = input;
  filter->index = (filter->index + 1) % WINDOW_SIZE;

  // 计算平均值
  float sum = 0.0;
  for (int i = 0; i < WINDOW_SIZE; ++i)
  {
    sum += filter->window_float[i];
  }
  float average = sum / WINDOW_SIZE;

  return average;
}

/*整数滑动均值滤波器*/
int8_t filterValue_int8(AverageFilter *filter, int8_t input)
{
  // 更新缓存区
  filter->window_int[filter->index] = input;
  filter->index = (filter->index + 1) % WINDOW_SIZE;

  // 计算平均值
  int32_t sum = 0.0;
  for (int i = 0; i < WINDOW_SIZE; ++i)
  {
    sum += filter->window_int[i];
  }
  int8_t average = (int8_t)(sum / WINDOW_SIZE);

  return average;
}
/*一介低通滤波器*/
double lowPassFilter(LowPassFilter *filter, double input) 
{
    // 滤波计算公式：output[n] = alpha * input[n] + (1 - alpha) * output[n-1]
    filter->output = filter->alpha * input + (1 - filter->alpha) * filter->output;
    return filter->output;
}


void lowpass_init(void)
{


}


void filterInit(void)
{
	average_init(&filter_ash5031_xdata);
	average_init(&filter_ash5031_ydata);

}

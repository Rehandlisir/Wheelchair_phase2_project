
#include "./BSP/Common/common.h"

AverageFilter filter_LN;
AverageFilter filter_RN;
AverageFilter filter_Lpwm;
AverageFilter filter_Rpwm;
AverageFilter filter_lspeed;
AverageFilter filter_rspeed;
int32_t Value_limit(int32_t min_value, int32_t current_value, int32_t max_value)
{

  if (current_value < min_value)
    return min_value;
  else if (current_value > max_value)
    return max_value;
  else
    return current_value;
}

float Value_limitf(float min_value, float current_value, float max_value)
{

  if (current_value <= min_value)
    return min_value;
  else if (current_value >= max_value)
    return max_value;
  else
    return current_value;
}

int32_t Value_ResetXzero(int32_t min_value, int32_t current_value, int32_t max_value)
{
  if (current_value < min_value || current_value > max_value)
    return current_value;
  else
    return 0;
}

int32_t Value_ResetYzero(int32_t min_value, int32_t current_value, int32_t max_value)
{
  if (current_value < min_value || current_value > max_value)
    return current_value;
  else
    return 0;
}

int16_t local_slopelimitx(int16_t value, int16_t increvalue,int16_t decreasvalue)//,int32_t increvalue,int32_t decreasvalue)
{
  static int16_t out_last = 0; // 上一次值
  int16_t out;
  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }
  if (value>=0)
  { 
    /***************** 正向递增约束 ******************/
    if (value - out_last >= increvalue)
    {
      out = out_last + increvalue;
    }
    /***************** 负向向递减约束 ******************/
    else if (value- out_last <= -decreasvalue)
    {
      out = out_last - decreasvalue;
    }
    /***************** 不满足增量约束条件直接输出 ******************/
    else
    {
      out = value;
    }
    // out_last = out;
    // return out;
  }
  else 
  {
    /***************** 负向递增约束 ******************/
    if (value - out_last <= -increvalue)
    {
      out = out_last -increvalue;
    } 
    /***************** 正向向递减约束 ******************/
    else if (value - out_last >= decreasvalue) 
    {
      out = out_last +decreasvalue;
    }
   else
   {
      out = value;
   }
  }
  out_last = out;
  return out;
}

int16_t local_slopelimity(int16_t value, int16_t increvalue,int16_t decreasvalue)//,int32_t increvalue,int32_t decreasvalue)
{
  static int16_t out_last = 0; // 上一次值
  int16_t out;
  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }
  if (value>=0)
  { 
    /***************** +向递增约束 ******************/
    if (value - out_last > increvalue)
    {
      out = out_last + increvalue;
    }
    /***************** -向递减约束 ******************/
    else if (value- out_last < -decreasvalue)
    {
      out = out_last - decreasvalue;
    }
    /***************** 不满足增量约束条件直接输出 ******************/
    else
    {
      out = value;
    }
  }
  else
  {
    /***************** 负向递+ 约束 ******************/
    if (value - out_last < -increvalue)
    {
      out = out_last -increvalue;
    } 
    /***************** 正向递- 约束 ******************/
    else if (value - out_last > decreasvalue) 
    {
      out = out_last + decreasvalue;
    }
    else
    {
      out = value;
    }
  }
  out_last = out;
  return out;
}

int32_t remote_slopelimitx(int32_t value, int32_t increvalue,int32_t decreasvalue)
{
  static int32_t out_last = 0; // 上一次值
  int32_t out;
  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }
  if (value>0)
  { 
    /***************** 正向递增约束 ******************/
    if (value - out_last >= increvalue)
    {
      out = out_last + increvalue;
    }
    /***************** 负向向递减约束 ******************/
    else if (value- out_last <= -decreasvalue)
    {
      out = out_last - decreasvalue;
    }
    /***************** 不满足增量约束条件直接输出 ******************/
    else
    {
      out = value;
    }
    // out_last = out;
    // return out;
  }
  else if (value<0)
  {
    /***************** 负向递增约束 ******************/
    if (value - out_last <= -increvalue)
    {
      out = out_last -increvalue;
    } 
    /***************** 正向向递减约束 ******************/
    else if (value - out_last >= decreasvalue) 
    {
      out = out_last + decreasvalue;
    }
   else
   {
      out = value;
   }
  }
  else
  {
      out = value;
  }
  out_last = out;
  return out;
}

int32_t remote_slopelimity(int32_t value, int32_t increvalue,int32_t decreasvalue)
{
  static int32_t out_last = 0; // 上一次值
  int32_t out;
  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }
  if (value>0)
  { 
    /***************** 正向递增约束 ******************/
    if (value - out_last >= increvalue)
    {
      out = out_last + increvalue;
    }
    /***************** 负向向递减约束 ******************/
    else if (value- out_last <= -decreasvalue)
    {
      out = out_last - decreasvalue;
    }
    /***************** 不满足增量约束条件直接输出 ******************/
    else
    {
      out = value;
    }
    // out_last = out;
    // return out;
  }
  else if (value<0)
  {
    /***************** 负向递增约束 ******************/
    if (value - out_last <= -increvalue)
    {
      out = out_last -increvalue;
    } 
    /***************** 正向向递减约束 ******************/
    else if (value - out_last >= decreasvalue) 
    {
      out = out_last + decreasvalue;
    }
   else
   {
      out = value;
   }
  }
  else
  {
      out = value;
  }
  out_last = out;
  return out;
}

double slopelimitLDuty(double value, double increvalue,double decreasvalue)
{
  static double out_last = 0; // 上一次值
  double out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }
  /***************** 正向递增约束 ******************/
  if ((value - out_last) >= increvalue)
  {
    out = out_last + increvalue;
  }
  /***************** 负向向递减约束 ******************/
  else if ((value - out_last) <= (-decreasvalue))
  {
    out = out_last - decreasvalue;
  }
  /***************** 不满足增量约束条件直接输出 ******************/
  else
  {
    out = value;
  }
  out_last = out;

  return out;
}

double slopelimitRDuty(double value, double increvalue,double decreasvalue)
{
  static double out_last = 0; // 上一次值
  double out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }
  /***************** 正向递增约束 ******************/
  if ((value - out_last) >= increvalue)
  {
    out = out_last + increvalue;
  }
  /***************** 负向向递减约束 ******************/
  else if ((value - out_last) <= (-decreasvalue))
  {
    out = out_last - decreasvalue;
  }
  /***************** 不满足增量约束条件直接输出 ******************/
  else
  {
    out = value;
  }
  out_last = out;

  return out;
}



// 初始化滤波器
void initializeFilter(AverageFilter *filter)
{
  for (int i = 0; i < WINDOW_SIZE; ++i)
  {
    filter->window[i] = 0;
    filter->window_int[i] = 0;
  }
  filter->index = 0;
}

void filterInit(void)
{
	initializeFilter(&filter_LN); 
	initializeFilter(&filter_RN); 
	initializeFilter(&filter_Lpwm);
	initializeFilter(&filter_Rpwm);
  initializeFilter(&filter_lspeed);
	initializeFilter(&filter_rspeed);
}

// 浮点数算术平均滤波函数
float filterValue_float(AverageFilter *filter, float input)
{
  // 更新缓存区
  filter->window[filter->index] = input;
  filter->index = (filter->index + 1) % WINDOW_SIZE;

  // 计算平均值
  float sum = 0.0;
  for (int i = 0; i < WINDOW_SIZE; ++i)
  {
    sum += filter->window[i];
  }
  float average = sum / WINDOW_SIZE;

  return average;
}

// 整形数算术平均滤波函数
int32_t filterValueX_int32(AverageFilter *filter, int16_t input)
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
  int16_t average = (int16_t)(sum / WINDOW_SIZE);

  return average;
}

int32_t filterValueY_int32(AverageFilter *filter, int16_t input)
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
  int16_t average = (int16_t)(sum / WINDOW_SIZE);

  return average;
}

/************************ 滤波器初始化 alpha *****************************/

uint16_t low_pass_filter1(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //上一次滤波值
  uint16_t out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

uint16_t low_pass_filter2(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //上一次滤波值
  uint16_t out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

uint16_t low_pass_filter3(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //上一次滤波值
  uint16_t out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
uint16_t low_pass_filter4(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //上一次滤波值
  uint16_t out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
uint16_t low_pass_filter5(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //上一次滤波值
  uint16_t out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
uint16_t low_pass_filter6(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //上一次滤波值
  uint16_t out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
uint16_t low_pass_filter7(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //上一次滤波值
  uint16_t out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

int16_t low_pass_filter8(int16_t value,float alpha)
{
  static int16_t out_last = 0; //上一次滤波值
  int16_t out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
int16_t low_pass_filter9(int16_t value,float alpha)
{
  static int16_t out_last = 0; //上一次滤波值
  int16_t out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

/******************************低通滤波器 ***********************************/
 LowPassFilter lowpassl_speed;
 LowPassFilter lowpassr_speed;
 LowPassFilter lowpassl_volatage;
 LowPassFilter lowpassr_volatage;
 LowPassFilter lowpassl_current;
 LowPassFilter lowpassr_current;
 LowPassFilter lowpassy_port;
 LowPassFilter lowpassx_port;
void lowpass_init(void)
{
  initLowPassFilter(&lowpassl_speed,2,1000);
  initLowPassFilter(&lowpassr_speed,5,1000);
  initLowPassFilter(&lowpassl_volatage,4,1000);
  initLowPassFilter(&lowpassr_volatage,4,1000);
  initLowPassFilter(&lowpassl_current,4,1000);
  initLowPassFilter(&lowpassr_current,4,1000);
  initLowPassFilter(&lowpassy_port,25,1000);
  initLowPassFilter(&lowpassx_port,300,1000);

}
// 初始化低通滤波器
void initLowPassFilter(LowPassFilter *filter, double cutoffFrequency, double samplingFrequency) 
{
    // 根据截止频率和采样频率计算滤波系数alpha
    filter->alpha = cutoffFrequency / (cutoffFrequency + samplingFrequency);
    filter->output = 0.0;
}

// 执行低通滤波操作
double lowPassFilter(LowPassFilter *filter, double input) 
{
    // 滤波计算公式：output[n] = alpha * input[n] + (1 - alpha) * output[n-1]
    filter->output = filter->alpha * input + (1 - filter->alpha) * filter->output;
    return filter->output;
}
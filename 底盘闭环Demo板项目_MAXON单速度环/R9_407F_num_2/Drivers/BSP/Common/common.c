
#include "./BSP/Common/common.h"

/*���Ʒ��Ⱥ���*/
int16_t Value_limit(int16_t min_value, int16_t current_value, int16_t max_value)
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

/*������ֵ�˲�����ʼ��*/
void average_init(AverageFilter *filter)
{
  for (int i = 0; i < WINDOW_SIZE; ++i)
  {
    filter->window_float[i] = 0;
    filter->window_int[i] = 0;
  }
  filter->index = 0;
}

// ��ʼ����ͨ�˲���
void initLowPassFilter(LowPassFilter *filter, double cutoffFrequency, double samplingFrequency) 
{
    // ���ݽ�ֹƵ�ʺͲ���Ƶ�ʼ����˲�ϵ��alpha
    filter->alpha = cutoffFrequency / (cutoffFrequency + samplingFrequency);
    filter->output = 0.0;
}

/*������������ֵ�˲���*/
float filterValue_float(AverageFilter *filter, float input)
{
  // ���»�����
  filter->window_float[filter->index] = input;
  filter->index = (filter->index + 1) % WINDOW_SIZE;

  // ����ƽ��ֵ
  float sum = 0.0;
  for (int i = 0; i < WINDOW_SIZE; ++i)
  {
    sum += filter->window_float[i];
  }
  float average = sum / WINDOW_SIZE;

  return average;
}

/*����������ֵ�˲���*/
int16_t filterValue_int16(AverageFilter *filter, int16_t input)
{
  // ���»�����
  filter->window_int[filter->index] = input;
  filter->index = (filter->index + 1) % WINDOW_SIZE;

  // ����ƽ��ֵ
  int32_t sum = 0.0;
  for (int i = 0; i < WINDOW_SIZE; ++i)
  {
    sum += filter->window_int[i];
  }
  int16_t average = (int16_t)(sum / WINDOW_SIZE);

  return average;
}
/*һ���ͨ�˲���*/
double lowPassFilter(LowPassFilter *filter, double input) 
{
    // �˲����㹫ʽ��output[n] = alpha * input[n] + (1 - alpha) * output[n-1]
    filter->output = filter->alpha * input + (1 - filter->alpha) * filter->output;
    return filter->output;
}


/*��ֵ�˲�������*/
AverageFilter filter_LN;
AverageFilter filter_RN;
AverageFilter filter_lspeed;
AverageFilter filter_rspeed;
AverageFilter filter_ADCX;
AverageFilter filter_ADCY;
/*��ͨ�˲�������*/
LowPassFilter lowpassl_speed;
LowPassFilter lowpassr_speed;
LowPassFilter lowpassl_volatage;
LowPassFilter lowpassr_volatage;
LowPassFilter lowpassl_current;
LowPassFilter lowpassr_current;
LowPassFilter lowpassy_ADC;
LowPassFilter lowpassx_ADC;
LowPassFilter lowpass_lspeedTarget;
LowPassFilter lowpass_rspeedTarget;
LowPassFilter filter_Lpwm;
LowPassFilter filter_Rpwm;

void lowpass_init(void)
{
  initLowPassFilter(&lowpassl_speed,4,1000);
  initLowPassFilter(&lowpassr_speed,4,1000);
  initLowPassFilter(&lowpassl_volatage,4,1000);
  initLowPassFilter(&lowpassr_volatage,4,1000);
  initLowPassFilter(&lowpassl_current,4,1000);
  initLowPassFilter(&lowpassr_current,4,1000);
  initLowPassFilter(&lowpassy_ADC,10,100);
  initLowPassFilter(&lowpassx_ADC,10,100);
  initLowPassFilter(&lowpass_lspeedTarget,2,100);
  initLowPassFilter(&lowpass_rspeedTarget,2,100);
  initLowPassFilter(&filter_Lpwm,50,1000);
  initLowPassFilter(&filter_Rpwm,50,1000);
}


void filterInit(void)
{
	average_init(&filter_LN);
	average_init(&filter_RN);
	average_init(&filter_ADCX);
	average_init(&filter_ADCY);
	lowpass_init();
	
}




// ִ�е�ͨ�˲�����
/************************ �˲�����ʼ�� alpha *****************************/

uint16_t low_pass_filter1(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //��һ���˲�ֵ
  uint16_t out;

  /***************** �����һ�ν��룬��� out_last ��ֵ ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** һ���˲� *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;
  return out;
}

uint16_t low_pass_filter2(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //��һ���˲�ֵ
  uint16_t out;

  /***************** �����һ�ν��룬��� out_last ��ֵ ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** һ���˲� *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

uint16_t low_pass_filter3(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //��һ���˲�ֵ
  uint16_t out;

  /***************** �����һ�ν��룬��� out_last ��ֵ ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** һ���˲� *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
uint16_t low_pass_filter4(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //��һ���˲�ֵ
  uint16_t out;

  /***************** �����һ�ν��룬��� out_last ��ֵ ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** һ���˲� *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
uint16_t low_pass_filter5(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //��һ���˲�ֵ
  uint16_t out;

  /***************** �����һ�ν��룬��� out_last ��ֵ ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** һ���˲� *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
uint16_t low_pass_filter6(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //��һ���˲�ֵ
  uint16_t out;

  /***************** �����һ�ν��룬��� out_last ��ֵ ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** һ���˲� *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
uint16_t low_pass_filter7(uint16_t value,float alpha)
{
  static uint16_t out_last = 0; //��һ���˲�ֵ
  uint16_t out;

  /***************** �����һ�ν��룬��� out_last ��ֵ ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** һ���˲� *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

int16_t low_pass_filter8(int16_t value,float alpha)
{
  static int16_t out_last = 0; //��һ���˲�ֵ
  int16_t out;

  /***************** �����һ�ν��룬��� out_last ��ֵ ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** һ���˲� *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
int16_t low_pass_filter9(int16_t value,float alpha)
{
  static int16_t out_last = 0; //��һ���˲�ֵ
  int16_t out;

  /***************** �����һ�ν��룬��� out_last ��ֵ ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** һ���˲� *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}
/****************************** END OF FILE ***********************************/

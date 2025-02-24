#include "./BSP/actuator.h"
/* ****************************************************** 定时器2/3基础配置**************************************** */
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
ACTUATOR_HandleTypeDef actuator_data;
void MX_TIM2_Init(void) {
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;               // 72MHz/(3+1)=18MHz 
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 899;                // 18MHz/900=20kHz 
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  
  HAL_TIM_PWM_Init(&htim2);
 
  // 通道3配置 
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;                  // 初始占空比50%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
 
  // 通道4配置 
  sConfigOC.Pulse = 0;                  // 初始占空比20%
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
 
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}
 
void MX_TIM3_Init(void) {
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3;               // 72MHz/(3+1)=18MHz 
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 899;                // 18MHz/900=20kHz 
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  
  HAL_TIM_PWM_Init(&htim3);
 
  // 通道1配置 
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;                  // 初始占空比50%
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
 
  // 通道2配置 
  sConfigOC.Pulse = 0;                  // 初始占空比20%
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
 
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}
 

/* *****************************************************************硬件层初始化******************************************************** */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_pwm->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
//    __HAL_AFIO_REMAP_TIM2_PARTIAL1();     // 启用部分重映射（若使用PB10/PB11）
 
    // PA2(TIM2_CH3), PA3(TIM2_CH4)
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
	  if(htim_pwm->Instance == TIM3) 
		{
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    // PA6(TIM3_CH1), PA7(TIM3_CH2)
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

/********************************************************************占空比设定封装*****************************************************************************************/
void Actuator1_SetDutyRatio(PWM_Channel ch, float ratio) {
  uint32_t pulse = (uint32_t)(htim2.Init.Period * ratio);
  __HAL_TIM_SET_COMPARE(&htim2, ch, pulse);
}

void Actuator2_SetDutyRatio(PWM_Channel ch, float ratio) {
  uint32_t pulse = (uint32_t)(htim3.Init.Period * ratio);
  __HAL_TIM_SET_COMPARE(&htim3, ch, pulse);
}

void actuator_init(void)
{
	MX_TIM2_Init();
	MX_TIM3_Init();
  actautor_detect_Init();
  sleepseting();
}


void actautor_detect_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  //低功耗配置接口GPIO_PIN_15--1  GPIO_PIN_14--2
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // 推杆报错检测 GPIO_PIN_13 --1  GPIO_PIN_12 --2
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
void sleepseting(void)
{
  //设置为 非睡眠模式
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
}

int8_t fault_state_detect(void)
{
  // 读取错误状态接口
  return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13));
 
}
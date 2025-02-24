#include "./BSP/time.h"
#include "./BSP/API_Schedule.h"
#include "./BSP/actuator.h"
TIM_HandleTypeDef htim4;
/*��ʱ������= (PSC+1)��(ARR+1)/TIM_CLK*/

void b_time4_Init(void) {
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;          // Ԥ��Ƶֵ 
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9;                // �Զ�����ֵ 
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  
  HAL_TIM_Base_Init(&htim4);
  HAL_TIM_Base_Start_IT(&htim4);        // ������ʱ�����ж� 
}


// stm32f1xx_hal_msp.c 
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  if (htim_base->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();        // ʹ��TIM6ʱ�� 
    
    // ����NVIC�ж� 
    HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
}


// stm32f1xx_it.c 
void TIM4_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim4);           // HAL���жϴ������ 
}
 
// �жϻص����� 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM4) {
      
					OS_IT_RUN();

          actuator_data.fault_status = fault_state_detect();
          
  }
}
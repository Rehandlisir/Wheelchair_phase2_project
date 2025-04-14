
#ifndef __ACTUATOR_H
#define __ACTUATOR_H

#include "./SYSTEM/sys/sys.h"

typedef enum {
	
	PWM_CH1 = TIM_CHANNEL_1,
  PWM_CH2 = TIM_CHANNEL_2,
  PWM_CH3 = TIM_CHANNEL_3,
  PWM_CH4 = TIM_CHANNEL_4 
} PWM_Channel;


typedef struct 
{
  uint8_t fault_status; //FAULT_MCU
  uint8_t over_current_flage; //SNSOUT_MCU
  float current; //A_SOUT1
  float pwm_duty; //PWM_DUTY1

}ACTUATOR_HandleTypeDef;
extern ACTUATOR_HandleTypeDef actuator_data;
#define ACTUATOR_TIMES 200  

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void Actuator1_SetDutyRatio(PWM_Channel ch, float ratio);
void Actuator2_SetDutyRatio(PWM_Channel ch, float ratio);
void actuator_init(void);
void actautor_detect_Init(void);
void sleepseting(void);
int8_t fault_state_detect(void);
#endif
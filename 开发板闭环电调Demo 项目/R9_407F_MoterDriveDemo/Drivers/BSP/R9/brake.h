/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/Code/R9_V2/R9_407F_num_2/Drivers/BSP/R9/brake.h
 * @Description  :  brake control
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-08-29 15:52:12
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/

#ifndef __BRAKE_H
#define __BRAKE_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 离合器状态读取 */

#define BRAKE1_GPIO_PORT                  GPIOD
#define BRAKE1_GPIO_PIN                   GPIO_PIN_8
#define BRAKE1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)             /*  */

#define BRAKE2_GPIO_PORT                  GPIOD
#define BRAKE2_GPIO_PIN                   GPIO_PIN_9
#define BRAKE2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)             /*  */

// 0 ： Drive  1：Push
#define LEFT_BREAK_STATE        HAL_GPIO_ReadPin(BRAKE1_GPIO_PORT,BRAKE1_GPIO_PIN)    // 高电平Drive

#define RIGHT_BRAKE_STATE       HAL_GPIO_ReadPin(BRAKE2_GPIO_PORT,BRAKE2_GPIO_PIN)     // 高电平Drive


/*离合器PWM控制接口TIME9 CH1 CH2*/
#define BRAKE_TIM9_PWM_CH_PORT           GPIOE
#define BRAKE_TIM9_PWM_CH1_GPIO          GPIO_PIN_5
#define BRAKE_TIM9_PWM_CH2_GPIO          GPIO_PIN_6
#define BRAKE_TIM9_PWM                   TIM9						/* TIM3 */
#define BRAKE_TIM9_PWM_CH1 TIM_CHANNEL_1			/* 通道Y,  1<= Y <=4 */
#define BRAKE_TIM9_PWM_CH2 TIM_CHANNEL_2			/* 通道Y,  1<= Y <=4 */
#define BRAKE_TIM9_PWM_CH1_CCRX TIM9->CCR1		/* 通道1的输出比较寄存器 */
#define BRAKE_TIM9_PWM_CH2_CCRX TIM9->CCR2		/* 通道2的输出比较寄存器 */

#define BRAKE_TIM9_PWM_CH_GPIO_AF GPIO_AF3_TIM9 /* 端口复用到TIM9 */
#define BRAKE_TIM9_PWM_CH_GPIOE_CLK_ENABLE() \
	do                                       \
	{                                        \
		__HAL_RCC_GPIOE_CLK_ENABLE();        \
	} while (0) /* PE口时钟使能 */

#define BRAKE_TIM9_PWM_CHY_CLK_ENABLE() \
	do                                 \
	{                                  \
		__HAL_RCC_TIM9_CLK_ENABLE();   \
	} while (0) /* TIM9 时钟使能 */


typedef struct 
{
    uint16_t  detect_time; /*检测计时 该设置需大于 200ms*/
    uint8_t detect_falge; /*需检测标志位*/
   
}STRUCT_BRAKE;

extern STRUCT_BRAKE struc_brake;

/******************************************************************************************/


/* */
void brakeTime9_init(uint16_t arr, uint16_t psc) ;
void brakestatedetec_init(void);
void brake_init(uint16_t arr, uint16_t psc);
void brake_pwm_set(uint16_t ccr1,uint16_t ccr2);
void brake(uint8_t flage);                                                                                        
#endif                                                                                                                       /*��ת��ƿ���*/

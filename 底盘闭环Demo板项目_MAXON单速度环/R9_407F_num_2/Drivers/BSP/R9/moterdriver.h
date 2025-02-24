/**
 * @FilePath     : /底盘闭环Demo板项目_MAXON单速度环/R9_407F_num_2/Drivers/BSP/R9/moterdriver.h
 * @Description  :  Moter Drive Header
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-02-14 15:01:19
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/

#ifndef __MOTERDRIVER_H
#define __MOTERDRIVER_H
//#include "./R9/stm32f4xx_hal_gpio_ex.h"
#include "./SYSTEM/sys/sys.h"
#include "./BSP/CAN/can.h"
#include "./BSP/R9/Slavemodbus.h"
#include "math.h"
#include "stdio.h"
#include "./BSP/R9/getadcdata.h"
#include "./BSP/Communicationheartbeat/Comheartbeat.h"
/**********************************底盘L 电机1 驱动输出 TIME1 CH1 CH2 CH1N CH2N  *************************************/
#define ATIM_TIM1_CPLM_CH1_GPIO_PORT            GPIOA
#define ATIM_TIM1_CPLM_CH1_GPIO_PIN             GPIO_PIN_8
#define ATIM_TIM1_CPLM_CH1_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */
#define ATIM_TIM1_CPLM_CH2_GPIO_PORT            GPIOE
#define ATIM_TIM1_CPLM_CH2_GPIO_PIN             GPIO_PIN_11
#define ATIM_TIM1_CPLM_CH2_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */
/* 互补输出通道引脚 */
#define ATIM_TIM1_CPLM_CH1N_GPIO_PORT           GPIOA
#define ATIM_TIM1_CPLM_CH1N_GPIO_PIN            GPIO_PIN_7
#define ATIM_TIM1_CPLM_CH1N_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */
#define ATIM_TIM1_CPLM_CH2N_GPIO_PORT           GPIOE
#define ATIM_TIM1_CPLM_CH2N_GPIO_PIN            GPIO_PIN_10
#define ATIM_TIM1_CPLM_CH2N_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */
/* 刹车输入引脚 */
// #define ATIM_TIM1_CPLM_BKIN_GPIO_PORT           GPIOE
// #define ATIM_TIM1_CPLM_BKIN_GPIO_PIN            GPIO_PIN_15
// #define ATIM_TIM1_CPLM_BKIN_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

// #define LEFT_SOFTBRAKE(x)                                                                                                                                \
//     do                                                                                                                                         \
//     {                                                                                                                                          \
//         x ? HAL_GPIO_WritePin(ATIM_TIM1_CPLM_BKIN_GPIO_PORT, ATIM_TIM1_CPLM_BKIN_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(ATIM_TIM1_CPLM_BKIN_GPIO_PORT, ATIM_TIM1_CPLM_BKIN_GPIO_PIN, GPIO_PIN_RESET); \
//     } while (0) /*  */
/*TIM1 REMAP设置*/
#define ATIM_TIM1_CPLM_CHY_GPIO_AF             GPIO_AF1_TIM1
/* 互补输出使用的定时器 */
#define ATIM_TIM1_CPLM                          TIM1
#define ATIM_TIM1_CPLM_CH1                      TIM_CHANNEL_1
#define ATIM_TIM1_CPLM_CH2                      TIM_CHANNEL_2
#define ATIM_TIM1_CPLM_CH1_CCR1                 ATIM_TIM1_CPLM->CCR1
#define ATIM_TIM1_CPLM_CH2_CCR2                 ATIM_TIM1_CPLM->CCR2
#define ATIM_TIM1_CPLM_CLK_ENABLE()             do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)    /* TIM1 时钟使能 */

/**********************************底盘R 电机2驱动输出 TIME8 CH1 CH2 CH1N CH2N*************************************/
/* 输出通道引脚 */
#define ATIM_TIM8_CPLM_CH1_GPIO_PORT            GPIOC
#define ATIM_TIM8_CPLM_CH1_GPIO_PIN             GPIO_PIN_6
#define ATIM_TIM8_CPLM_CH1_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

#define ATIM_TIM8_CPLM_CH2_GPIO_PORT            GPIOC
#define ATIM_TIM8_CPLM_CH2_GPIO_PIN             GPIO_PIN_7
#define ATIM_TIM8_CPLM_CH2_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

/* 互补输出通道引脚 */
#define ATIM_TIM8_CPLM_CH1N_GPIO_PORT           GPIOA
#define ATIM_TIM8_CPLM_CH1N_GPIO_PIN            GPIO_PIN_5
#define ATIM_TIM8_CPLM_CH1N_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

#define ATIM_TIM8_CPLM_CH2N_GPIO_PORT           GPIOB
#define ATIM_TIM8_CPLM_CH2N_GPIO_PIN            GPIO_PIN_0
#define ATIM_TIM8_CPLM_CH2N_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */
/* 刹车输入引脚 */
// #define ATIM_TIM8_CPLM_BKIN_GPIO_PORT           GPIOA
// #define ATIM_TIM8_CPLM_BKIN_GPIO_PIN            GPIO_PIN_6
// #define ATIM_TIM8_CPLM_BKIN_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */
// #define RIGHT_SOFTBRAKE(x)                                                                                                                     \
//     do                                                                                                                                         \
//     {                                                                                                                                          \
//         x ? HAL_GPIO_WritePin(ATIM_TIM8_CPLM_BKIN_GPIO_PORT, ATIM_TIM8_CPLM_BKIN_GPIO_PIN,GPIO_PIN_SET) : HAL_GPIO_WritePin(ATIM_TIM8_CPLM_BKIN_GPIO_PORT, ATIM_TIM8_CPLM_BKIN_GPIO_PIN, GPIO_PIN_RESET); \
//     } while (0) /*  */
/* TIM8 REMAP设置*/
#define ATIM_TIM8_CPLM_CHY_GPIO_AF             GPIO_AF3_TIM8

/* 互补输出使用的定时器 */
#define ATIM_TIM8_CPLM                          TIM8
#define ATIM_TIM8_CPLM_CH1                      TIM_CHANNEL_1
#define ATIM_TIM8_CPLM_CH2                      TIM_CHANNEL_2
#define ATIM_TIM8_CPLM_CH1_CCR1                 ATIM_TIM8_CPLM->CCR1
#define ATIM_TIM8_CPLM_CH2_CCR2                 ATIM_TIM8_CPLM->CCR2
#define ATIM_TIM8_CPLM_CLK_ENABLE()             do{ __HAL_RCC_TIM8_CLK_ENABLE(); }while(0)    /* TIM1 时钟使能 */





void MoterL_pwm_chy_init(uint16_t arr, uint16_t psc);				   // 左轮电机
void atim_tim1_cplm_pwm_set(uint16_t ccr1,uint16_t ccr2);
void LeftMoterMove(double lduty_cycle);
void LeftMoterStop(void);

void MoterR_pwm_chy_init(uint16_t arr, uint16_t psc);				   // 右轮电机
void atim_tim8_cplm_pwm_set(uint16_t ccr1,uint16_t ccr2);
void RightMoterMove(double rduty_cycle);
void RightMoterStop(void);
void car_move(double lduty_cycle, double rduty_cycle);

void MoterdriveInit(void);



#endif

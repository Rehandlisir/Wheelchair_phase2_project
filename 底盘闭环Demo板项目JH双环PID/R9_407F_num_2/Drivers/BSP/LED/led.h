/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/第二阶段新项目/底盘闭环Demo板项目/R9_407F_num_2/Drivers/BSP/LED/led.h
 * @Description  :  LED Control
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2024-11-14 11:45:55
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/

#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"
#include "./BSP/Communicationheartbeat/Comheartbeat.h"
#define LED1_GPIO_PORT GPIOB
#define LED1_GPIO_PIN GPIO_PIN_10
#define LED1_GPIO_CLK_ENABLE()        \
    do                                \
    {                                 \
        __HAL_RCC_GPIOB_CLK_ENABLE(); \
    } while (0) /*  */

#define LED2_GPIO_PORT GPIOE
#define LED2_GPIO_PIN GPIO_PIN_15
#define LED2_GPIO_CLK_ENABLE()        \
    do                                \
    {                                 \
        __HAL_RCC_GPIOE_CLK_ENABLE(); \
    } while (0) /* PF��ʱ��ʹ�� */



/******************************************************************************************/

/* 指示LED */
#define LED1(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0) /* LED1 = RED */

#define LED2(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0) /* LED1 = GREEN */

#define LED1_TOGGLE()                                      \
    do                                                     \
    {                                                      \
        HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN); \
    } while (0) 
#define LED2_TOGGLE()                                      \
    do                                                     \
    {                                                      \
        HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_GPIO_PIN); \
    } while (0) 

/* */
void led_init(void);
#endif /**/

/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/闭环电调Demo/R9_407F_MoterDriveDemo/Drivers/BSP/LED/led.h
 * @Description  :  LED Control
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-10-23 13:42:54
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/

#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"
#include "./BSP/Communicationheartbeat/Comheartbeat.h"
#define LED0_GPIO_PORT GPIOF
#define LED0_GPIO_PIN GPIO_PIN_9
#define LED0_GPIO_CLK_ENABLE()        \
    do                                \
    {                                 \
        __HAL_RCC_GPIOF_CLK_ENABLE(); \
    } while (0) /*  */

#define LED1_GPIO_PORT GPIOF
#define LED1_GPIO_PIN GPIO_PIN_10
#define LED1_GPIO_CLK_ENABLE()        \
    do                                \
    {                                 \
        __HAL_RCC_GPIOF_CLK_ENABLE(); \
    } while (0) /* PF��ʱ��ʹ�� */



/******************************************************************************************/

/* 指示LED */
#define LED0(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(LED0_GPIO_PORT, LED0_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0) /* LED0 = RED */

#define LED1(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0) /* LED1 = GREEN */

#define LED0_TOGGLE()                                      \
    do                                                     \
    {                                                      \
        HAL_GPIO_TogglePin(LED0_GPIO_PORT, LED0_GPIO_PIN); \
    } while (0) 
#define LED1_TOGGLE()                                      \
    do                                                     \
    {                                                      \
        HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN); \
    } while (0) 



/******************************************************************************************/



/* */
void led_init(void);
#endif /**/

/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/Code/R9_407E/R9_407_F/Drivers/BSP/LED/led.h
 * @Description  :  LED Control
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-07-30 14:50:14
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/

#ifndef __LED_H
#define __LED_H

#include "./SYSTEM/sys/sys.h"
#include "./BSP/Communicationheartbeat/Comheartbeat.h"
#define LED2_GPIO_PORT GPIOF
#define LED2_GPIO_PIN GPIO_PIN_1
#define LED2_GPIO_CLK_ENABLE()        \
    do                                \
    {                                 \
        __HAL_RCC_GPIOF_CLK_ENABLE(); \
    } while (0) /*  */

#define LED1_GPIO_PORT GPIOF
#define LED1_GPIO_PIN GPIO_PIN_0
#define LED1_GPIO_CLK_ENABLE()        \
    do                                \
    {                                 \
        __HAL_RCC_GPIOF_CLK_ENABLE(); \
    } while (0) /* PF��ʱ��ʹ�� */

/*前 - 左转向*/
#define LEFT_FRONT_TURE_GPIO_PORT GPIOC
#define LEFT_FRONT_TURE_GPIO_PIN GPIO_PIN_13
#define LEFT_FRONT_TURE_GPIO_CLK_ENABLE() \
    do                                    \
    {                                     \
        __HAL_RCC_GPIOC_CLK_ENABLE();     \
    } while (0) /* PC13 ????? */
/*前 - 右转向*/
#define RIGHT_FRONT_TURE_GPIO_PORT GPIOD
#define RIGHT_FRONT_TURE_GPIO_PIN GPIO_PIN_4
#define RIGHT_FRONT_TURE_GPIO_CLK_ENABLE() \
    do                                     \
    {                                      \
        __HAL_RCC_GPIOE_CLK_ENABLE();      \
    } while (0)
/*前 - 主灯*/
#define FRONT_MAIN_GPIO_PORT GPIOG
#define FRONT_MAIN_GPIO_PIN GPIO_PIN_12
#define FRONT_MAIN_GPIO_CLK_ENABLE()  \
    do                                \
    {                                 \
        __HAL_RCC_GPIOG_CLK_ENABLE(); \
    } while (0)


/*后 -左转向*/
#define LEFT_BACK_TURE_GPIO_PORT GPIOE
#define LEFT_BACK_TURE_GPIO_PIN GPIO_PIN_3
#define LEFT_BACK_TURE_GPIO_CLK_ENABLE() \
    do                                   \
    {                                    \
        __HAL_RCC_GPIOE_CLK_ENABLE();    \
    } while (0) /* PC13 ????? */
/*后 -右转向*/
#define RIGHT_BACK_TURE_GPIO_PORT GPIOE
#define RIGHT_BACK_TURE_GPIO_PIN GPIO_PIN_2
#define RIGHT_BACK_TURE_GPIO_CLK_ENABLE() \
    do                                    \
    {                                     \
        __HAL_RCC_GPIOE_CLK_ENABLE();     \
    } while (0)
/*后 - 主灯*/
#define BACK_MAIN_GPIO_PORT GPIOG
#define BACK_MAIN_GPIO_PIN GPIO_PIN_11
#define BACK_MAIN_GPIO_CLK_ENABLE()   \
    do                                \
    {                                 \
        __HAL_RCC_GPIOG_CLK_ENABLE(); \
    } while (0)

/******************************************************************************************/

/* 指示LED */
#define LED2(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0) /* LED0 = RED */

#define LED1(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0) /* LED1 = GREEN */

#define LED2_TOGGLE()                                      \
    do                                                     \
    {                                                      \
        HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_GPIO_PIN); \
    } while (0) 
#define LED1_TOGGLE()                                      \
    do                                                     \
    {                                                      \
        HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN); \
    } while (0) 

/*前 -左转向控制*/
#define LEFT_FRONT_TURE(x)                                                                                                                                                                 \
    do                                                                                                                                                                                     \
    {                                                                                                                                                                                      \
        x ? HAL_GPIO_WritePin(LEFT_FRONT_TURE_GPIO_PORT, LEFT_FRONT_TURE_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(LEFT_FRONT_TURE_GPIO_PORT, LEFT_FRONT_TURE_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0) 
/*前-右转向控制*/
#define RIGHT_FRONT_TURE(x)                                                                                                                                                                    \
    do                                                                                                                                                                                         \
    {                                                                                                                                                                                          \
        x ? HAL_GPIO_WritePin(RIGHT_FRONT_TURE_GPIO_PORT, RIGHT_FRONT_TURE_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(RIGHT_FRONT_TURE_GPIO_PORT, RIGHT_FRONT_TURE_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0)
/*前 -主灯控制*/
#define FRONT_MAIN(x)                                                                                                                                                  \
    do                                                                                                                                                                 \
    {                                                                                                                                                                  \
        x ? HAL_GPIO_WritePin(FRONT_MAIN_GPIO_PORT, FRONT_MAIN_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(FRONT_MAIN_GPIO_PORT, FRONT_MAIN_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0)
/*前 -左转向控制*/
#define LEFT_FRONT_TURE_TOGGLE()                                                 \
    do                                                                           \
    {                                                                            \
        HAL_GPIO_TogglePin(LEFT_FRONT_TURE_GPIO_PORT, LEFT_FRONT_TURE_GPIO_PIN); \
    } while (0) /*????? ?? */
/*前-右转向控制*/   
#define RIGHT_FRONT_TURE_TOGGLE()                                                  \
    do                                                                             \
    {                                                                              \
        HAL_GPIO_TogglePin(RIGHT_FRONT_TURE_GPIO_PORT, RIGHT_FRONT_TURE_GPIO_PIN); \
    } while (0) /*????? ?? */


/*后-左转向控制*/
#define LEFT_BACK_TURE(x)                                                                                                                                                              \
    do                                                                                                                                                                                 \
    {                                                                                                                                                                                  \
        x ? HAL_GPIO_WritePin(LEFT_BACK_TURE_GPIO_PORT, LEFT_BACK_TURE_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(LEFT_BACK_TURE_GPIO_PORT, LEFT_BACK_TURE_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0) /* ????? */
/*后-右转向控制*/
#define RIGHT_BACK_TURE(x)                                                                                                                                                                 \
    do                                                                                                                                                                                     \
    {                                                                                                                                                                                      \
        x ? HAL_GPIO_WritePin(RIGHT_BACK_TURE_GPIO_PORT, RIGHT_BACK_TURE_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(RIGHT_BACK_TURE_GPIO_PORT, RIGHT_BACK_TURE_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0)
/*后-主灯控制*/
#define BACK_MAIN(x)                                                                                                                                                \
    do                                                                                                                                                              \
    {                                                                                                                                                               \
        x ? HAL_GPIO_WritePin(BACK_MAIN_GPIO_PORT, BACK_MAIN_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(BACK_MAIN_GPIO_PORT, BACK_MAIN_GPIO_PIN, GPIO_PIN_RESET);  \
    } while (0)

#define MainBulbState HAL_GPIO_ReadPin(FRONT_MAIN_GPIO_PORT,FRONT_MAIN_GPIO_PIN);
#define LeftBulbState HAL_GPIO_ReadPin(LEFT_FRONT_TURE_GPIO_PORT,LEFT_FRONT_TURE_GPIO_PIN);
#define RightBulbState HAL_GPIO_ReadPin(RIGHT_FRONT_TURE_GPIO_PORT,RIGHT_FRONT_TURE_GPIO_PIN);

// #define KEYCONTRONL 0

/******************************************************************************************/

typedef enum
{
    None,
    idle_state,
    open_leftbling,
    open_rightbling,
    open_doublebling,
    open_mainbulb,

    close_leftbling,
    close_rightbling,
    close_doublebling,
    close_mainbulb
} Led_State;

/* */
void led_init(void);
void led_beepControl(void); /*  */
void led_beepControlRK3588(void);
#endif /**/

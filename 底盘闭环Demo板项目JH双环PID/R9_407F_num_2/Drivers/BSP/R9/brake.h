/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/第二阶段新项目/底盘闭环Demo板项目/R9_407F_num_2/Drivers/BSP/R9/brake.h
 * @Description  :  brake control
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2024-11-14 15:54:54
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/

#ifndef __BRAKE_H
#define __BRAKE_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 离合器状态读取 */

#define BRAKE1_GPIO_PORT                  GPIOG
#define BRAKE1_GPIO_PIN                   GPIO_PIN_12
#define BRAKE1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)             /*  */

#define BRAKE2_GPIO_PORT                  GPIOG
#define BRAKE2_GPIO_PIN                   GPIO_PIN_13
#define BRAKE2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)             /*  */

// 0 ： Drive  1：Push
#define LEFT_BREAK_STATE        HAL_GPIO_ReadPin(BRAKE1_GPIO_PORT,BRAKE1_GPIO_PIN)    // 高电平Drive

#define RIGHT_BRAKE_STATE       HAL_GPIO_ReadPin(BRAKE2_GPIO_PORT,BRAKE2_GPIO_PIN)     // 高电平Drive


/*离合器驱动控制接口*/
#define BRAKEDRIVE_GPIO_PORT                  GPIOG
#define BRAKEDRIVE_GPIO_PIN                   GPIO_PIN_11
#define BRAKEDRIVE_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)

#define BRAKEDRIVE(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(BRAKEDRIVE_GPIO_PORT, BRAKEDRIVE_GPIO_PIN, GPIO_PIN_RESET) : HAL_GPIO_WritePin(BRAKEDRIVE_GPIO_PORT, BRAKEDRIVE_GPIO_PIN, GPIO_PIN_SET); \
    } while (0) /* LED1 = RED */

/******************************************************************************************/


/* */
void brake_init(void);
void brake(uint8_t flage);                                                                                        
#endif                                                                                                                       /*��ת��ƿ���*/

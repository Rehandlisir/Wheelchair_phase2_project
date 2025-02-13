/**
 * @FilePath     : /R9_V2_2号机最新/R9_407F_num_2/Drivers/BSP/R9/brake.h
 * @Description  :  brake control
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-02-08 16:52:35
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/

#ifndef __BRAKE_H
#define __BRAKE_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 离合器状态读取 */

#define BRAKE_GPIO_PORT                  GPIOE
#define BRAKE_GPIO_PIN                   GPIO_PIN_5
#define BRAKE_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)             
#define BRAKE(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(BRAKE_GPIO_PORT, BRAKE_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(BRAKE_GPIO_PORT, BRAKE_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0)    /*x ==1 松开抱闸器  否则 锁住抱闸器*/  
	
void brake_init(void);	
#endif                                                                                                                       

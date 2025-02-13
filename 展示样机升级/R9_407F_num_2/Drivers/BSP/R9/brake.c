/**
 * @FilePath     : /R9_V2_2????/R9_407F_num_2/Drivers/BSP/R9/brake.c
 * @Description  :  Brake Control and Detection
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-02-08 17:30:07
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/
#include "./BSP/R9/brake.h"
/**
 * @brief        :
 *                 
 * @return        {*}
**/

void brake_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;                         
    BRAKE_GPIO_CLK_ENABLE();
    gpio_init_struct.Pin = BRAKE_GPIO_PIN;                       
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                    
    gpio_init_struct.Pull = GPIO_PULLUP;                       
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              
    HAL_GPIO_Init(BRAKE_GPIO_PORT, &gpio_init_struct); 

}

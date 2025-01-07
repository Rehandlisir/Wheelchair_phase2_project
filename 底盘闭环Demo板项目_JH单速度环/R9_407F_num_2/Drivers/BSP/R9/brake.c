/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/第二阶段新项目/底盘闭环Demo板项目/R9_407F_num_2/Drivers/BSP/R9/brake.c
 * @Description  :  Brake Control and Detection
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2024-11-14 15:56:34
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/
#include "./BSP/R9/brake.h"

#include "./SYSTEM/delay/delay.h"

void brake_init(void)
{
    
     /*离合器驱动*/
	GPIO_InitTypeDef gpio_init_struct;
	BRAKEDRIVE_GPIO_CLK_ENABLE();
	gpio_init_struct.Pin = BRAKEDRIVE_GPIO_PIN;			  
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;	  
	gpio_init_struct.Pull = GPIO_PULLUP;			  
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;	  
	HAL_GPIO_Init(BRAKEDRIVE_GPIO_PORT, &gpio_init_struct); 
	/*离合器*/
    brake(1);
}


void brake(uint8_t flage)
{
    BRAKEDRIVE(flage);
}


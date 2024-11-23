/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/Code/R9_V2/R9_407F_num_2/Drivers/BSP/LED/led.c
 * @Description  :  LED Control
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-08-27 15:07:22
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
#include "./BSP/LED/led.h"
#include "./SYSTEM/delay/delay.h"
void led_init(void) 
{

	GPIO_InitTypeDef gpio_init_struct;

	/*前后转向灯*/
	LED0_GPIO_CLK_ENABLE(); 
	LED1_GPIO_CLK_ENABLE(); 
	gpio_init_struct.Pin = LED0_GPIO_PIN;			  
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;	  
	gpio_init_struct.Pull = GPIO_PULLUP;			  
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;	  
	HAL_GPIO_Init(LED0_GPIO_PORT, &gpio_init_struct); 
	gpio_init_struct.Pin = LED1_GPIO_PIN;			  
	HAL_GPIO_Init(LED1_GPIO_PORT, &gpio_init_struct);
	/*指示灯初始化*/
	LED0(1); 
	LED1(1); 
}
/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/R9CODE/R9_V2_2号机最新/R9_V2/R9_407F_num_2/Drivers/BSP/LED/led.c
 * @Description  :  LED Control
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2024-11-01 11:11:47
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/R9/Slavemodbus.h"
void led_init(void) 
{
    /*指示灯*/
	GPIO_InitTypeDef gpio_init_struct;
	LED1_GPIO_CLK_ENABLE(); 
	LED2_GPIO_CLK_ENABLE(); 
	gpio_init_struct.Pin = LED1_GPIO_PIN;			  
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;	  
	gpio_init_struct.Pull = GPIO_PULLUP;			  
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;	  
	HAL_GPIO_Init(LED1_GPIO_PORT, &gpio_init_struct); 
	gpio_init_struct.Pin = LED2_GPIO_PIN;			  
	HAL_GPIO_Init(LED2_GPIO_PORT, &gpio_init_struct);
	/*指示灯初始化*/
	LED1(1); 
	LED2(1); 
}
/**
 * @description: 上位机控制 LED灯
 * @return {*}
 */
void led_beepControlRK3588(void)
{
	;
}

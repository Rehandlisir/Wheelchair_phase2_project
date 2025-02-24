#ifndef __LED_H
#define __LED_H

	#include "./SYSTEM/sys/sys.h"

	#define LED1_GPIO_PORT GPIOB
	#define LED1_GPIO_PIN GPIO_PIN_11
	#define LED1_GPIO_CLK_ENABLE()        \
			do                                \
			{                                 \
					__HAL_RCC_GPIOB_CLK_ENABLE(); \
			} while (0) /*  */

	#define LED1_TOGGLE()                                      \
			do                                                     \
			{                                                      \
					HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_GPIO_PIN); \
			} while (0) 
	
			
			
			
	#define LED1(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0) /* LED0 = RED */
		
		
		
void led_init(void);
			
#endif
			
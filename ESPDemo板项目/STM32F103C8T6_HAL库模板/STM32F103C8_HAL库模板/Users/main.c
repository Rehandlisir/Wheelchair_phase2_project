/**
 ******************************************************************************
 * @file     main.c
 * @author   ����ԭ���Ŷ�(ALIENTEK)
 * @version  V1.0
 * @date     2020-08-20
 * @brief    �½�����ʵ��-HAL��汾 ʵ��
 * @license  Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ******************************************************************************
 * @attention
 * 
 * ʵ��ƽ̨:����ԭ�� STM32F103 ������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 ******************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"

void led_init(void);                       /* LED��ʼ���������� */

int main(void)
{
    HAL_Init();                             /* ��ʼ��HAL�� */
    sys_stm32_clock_init(RCC_PLL_MUL6);     /* ����ʱ��, 72Mhz */
    delay_init(72);                         /* ��ʱ��ʼ�� */
    led_init();                             /* LED��ʼ�� */
    
    while(1)
    { 
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);    /* PA8��1 */ 
//        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);  /* PD2��0 */ 
        delay_ms(1000);
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);  /* PA8��0 */
//        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);    /* PD2��1 */
        delay_ms(1000); 
    }
}

/**     
 * @brief       ��ʼ��LED���IO��, ��ʹ��ʱ��
 * @param       ��
 * @retval      ��
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;
    __HAL_RCC_GPIOC_CLK_ENABLE();                          /* IO��PAʱ��ʹ�� */
//    __HAL_RCC_GPIOD_CLK_ENABLE();                          /* IO��PDʱ��ʹ�� */

    gpio_initstruct.Pin = GPIO_PIN_13;                      /* LED0���� */
    gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;            /* ������� */
    gpio_initstruct.Pull = GPIO_PULLUP;                    /* ���� */
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;          /* ���� */
    HAL_GPIO_Init(GPIOC, &gpio_initstruct);                /* ��ʼ��LED0���� */

//    gpio_initstruct.Pin = GPIO_PIN_2;                      /* LED1���� */
//    HAL_GPIO_Init(GPIOD, &gpio_initstruct);                /* ��ʼ��LED1���� */
}

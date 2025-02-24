
#include "./BSP/led.h"

void led_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();                          /* IO��PAʱ��ʹ�� */
    gpio_initstruct.Pin = GPIO_PIN_11;                      /* LED0���� */
    gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;            /* ������� */
    gpio_initstruct.Pull = GPIO_PULLUP;                    /* ���� */
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;          /* ���� */
    HAL_GPIO_Init(GPIOB, &gpio_initstruct);                /* ��ʼ��LED0���� */
		LED1(1);
}

#include "./BSP/led.h"

void led_init(void)
{
    GPIO_InitTypeDef gpio_initstruct;
    __HAL_RCC_GPIOB_CLK_ENABLE();                          /* IO口PA时钟使能 */
    gpio_initstruct.Pin = GPIO_PIN_11;                      /* LED0引脚 */
    gpio_initstruct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_initstruct.Pull = GPIO_PULLUP;                    /* 上拉 */
    gpio_initstruct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
    HAL_GPIO_Init(GPIOB, &gpio_initstruct);                /* 初始化LED0引脚 */
		LED1(1);
}
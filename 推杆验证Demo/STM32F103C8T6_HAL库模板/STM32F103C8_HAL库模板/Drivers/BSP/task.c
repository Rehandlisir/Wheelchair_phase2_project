/*
 * @Author: lisir lisir@rehand.com
 * @Date: 2024-06-07 16:01:18
 * @LastEditors: lisir lisir@rehand.com
 * @LastEditTime: 2024-08-15 11:00:21
 * @FilePath: \R9_407F_NO2\R9_407F_Num_2\R9_407F_num_2\Drivers\BSP\task.c
 * @Description: 主任务列表
 */
#include "./BSP/task.h"

/**
 * @description:
 * @return {*}
 */
void Hard_devInit(void)
{

    HAL_Init();                         /* 初始化HAL库 */
    sys_stm32_clock_init(RCC_PLL_MUL6); /* 设置时钟, 72Mhz */
    delay_init(72);                     /* 延时初始化 */
    led_init();
    b_time4_Init();
    actuator_init();
    usart_init(115200);
}

void TASK_led(void)
{
//    LED1_TOGGLE();
}

void TASK_Actor_excute(void)
{
    static float acctemp ;
    static uint16_t acct ;
    static uint8_t accdoneflage ;
    if(actuator_data.fault_status==0)  //低电平表示有故障切换到IDLE状态
    {
        actor_cmd = IDLE;
        LED1(1);
    }
    else
    {
        LED1(0);

    }
   
    switch (actor_cmd)
    {
        
    case IDLE:
        Actuator1_SetDutyRatio(TIM_CHANNEL_3, 0);
        Actuator1_SetDutyRatio(TIM_CHANNEL_4, 0);
        Actuator2_SetDutyRatio(TIM_CHANNEL_1, 0);
        Actuator2_SetDutyRatio(TIM_CHANNEL_2, 0);
        acct = 0;
        accdoneflage = 0;
        acctemp = 0.0;
        break;

    case actuator1_UP:
        if ((acct < ACTUATOR_TIMES) && (accdoneflage == 0)) // 0.5 --- 0.75  200ms
        {
            acct++;
            acctemp = 4.687500000000011e-12* pow(acct, 5.0) -2.343750000000005e-09 * pow(acct, 4.0) +3.125000000000006e-07* pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        Actuator1_SetDutyRatio(TIM_CHANNEL_3, 0);
        Actuator1_SetDutyRatio(TIM_CHANNEL_4, acctemp);
        break;
    case actuator2_UP:
        if ((acct < ACTUATOR_TIMES) && (accdoneflage == 0)) // 0 --- 0.95  200ms
        {
            acct++;
            acctemp = 4.687500000000011e-12* pow(acct, 5.0) - 2.343750000000005e-09 * pow(acct, 4.0) +3.125000000000006e-07* pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        Actuator2_SetDutyRatio(TIM_CHANNEL_1,  0);
        Actuator2_SetDutyRatio(TIM_CHANNEL_2, acctemp);

        break;
    case coordinated_UP:
        if ((acct < ACTUATOR_TIMES) && (accdoneflage == 0)) // 0 --- 0.95  200ms
        {
            acct++;
            acctemp = 4.687500000000011e-12* pow(acct, 5.0) - 2.343750000000005e-09 * pow(acct, 4.0) +3.125000000000006e-07* pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        
        Actuator1_SetDutyRatio(TIM_CHANNEL_3, 0);
        Actuator1_SetDutyRatio(TIM_CHANNEL_4, acctemp);
        Actuator2_SetDutyRatio(TIM_CHANNEL_1,  0);
        Actuator2_SetDutyRatio(TIM_CHANNEL_2, acctemp);

        break;
    case actuator1_DOWN:
        if ((acct < ACTUATOR_TIMES) && (accdoneflage == 0)) // 0 --- 0.95  200ms
        {
            acct++;
            acctemp = 4.687500000000011e-12* pow(acct, 5.0) - 2.343750000000005e-09 * pow(acct, 4.0) +3.125000000000006e-07* pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        Actuator1_SetDutyRatio(TIM_CHANNEL_3, acctemp);
        Actuator1_SetDutyRatio(TIM_CHANNEL_4, 0);

        break;
    case actuator2_DOWN:
        if ((acct < ACTUATOR_TIMES) && (accdoneflage == 0)) // 0 --- 0.95  200ms
        {
            acct++;
            acctemp = 4.687500000000011e-12* pow(acct, 5.0) - 2.343750000000005e-09 * pow(acct, 4.0) +3.125000000000006e-07* pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        Actuator2_SetDutyRatio(TIM_CHANNEL_1, acctemp);
        Actuator2_SetDutyRatio(TIM_CHANNEL_2, 0);

        break;
    case coordinated_DOWN:
        if ((acct < ACTUATOR_TIMES) && (accdoneflage == 0)) // 0 --- 0.95  200ms
        {
            acct++;
            acctemp = 4.687500000000011e-12* pow(acct, 5.0) - 2.343750000000005e-09 * pow(acct, 4.0) +3.125000000000006e-07* pow(acct, 3) + 0.5;
        }
        else
        {
            accdoneflage = 1;
            acct = 0;
        }
        Actuator1_SetDutyRatio(TIM_CHANNEL_3, acctemp);
        Actuator1_SetDutyRatio(TIM_CHANNEL_4, 0);
        Actuator2_SetDutyRatio(TIM_CHANNEL_1, acctemp);
        Actuator2_SetDutyRatio(TIM_CHANNEL_2, 0);
        break;
    default:
        break;
    }
   
   
}

void TASK_Actor_velovityPlan(void)
{
}

void TASK_actuatorCMD(void)
{
}

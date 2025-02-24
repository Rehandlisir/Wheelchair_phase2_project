/*
 * @Author: lisir lisir@rehand.com
 * @Date: 2024-06-07 16:01:18
 * @LastEditors: lisir lisir@rehand.com
 * @LastEditTime: 2024-06-14 15:14:08
 * @FilePath: \MDK-ARMc:\Users\fu\Desktop\Code\CodeV1.1\R9_407_V1.1\R9_407_V1.1\Drivers\BSP\task.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __TASK_H__
#define __TASK_H__
#include "./SYSTEM/sys/sys.h"
#include "stdio.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/led.h"
#include "./BSP/actuator.h"
#include "./BSP/time.h"
#include"./math.h"
#include"./stdlib.h"
//任务列表
void Hard_devInit(void);
void TASK_led(void);
void TASK_Actor_excute(void);
void TASK_Actor_velovityPlan(void);
void TASK_actuatorCMD(void);

#endif

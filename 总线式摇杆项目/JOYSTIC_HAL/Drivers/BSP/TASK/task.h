
#ifndef __TASK_H__
#define __TASK_H__
#include "./SYSTEM/sys/sys.h"
#include "stdio.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/TIMER/btim.h"
#include "./BSP/MLX90393/mlx90393.h"
#include "./BSP/WDG/wdg.h"
#include "./BSP/CAN/can.h"
#include "./BSP/MLX90393_CAN/mlx90393can_send.h"

//任务列表
void Hard_devInit(void);
void Task_GetMlx90393(void);
void Task_CanjoysticRun(void);
void Task_R9DataScope(void);
#endif

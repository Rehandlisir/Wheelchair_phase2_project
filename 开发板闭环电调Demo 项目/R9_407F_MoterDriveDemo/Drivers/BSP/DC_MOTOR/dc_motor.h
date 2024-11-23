/**
 ****************************************************************************************************
 * @file        dc_motor.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       直流有刷电机控制代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com/forum.php
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __DCMOTOR_H
#define __DCMOTOR_H

#include "./SYSTEM/sys/sys.h"

/*************************************    基础驱动    *****************************************************/

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc);          /* 高级定时器 互补输出 初始化函数 */
void dcmotor_init(void);                                           /* 直流有刷电机初始化 */
void dcmotor_start(void);                                          /* 开启电机 */
void dcmotor_stop(void);                                           /* 关闭电机 */  
void dcmotor_dir(uint8_t para);                                    /* 设置电机方向 */
    
#endif



















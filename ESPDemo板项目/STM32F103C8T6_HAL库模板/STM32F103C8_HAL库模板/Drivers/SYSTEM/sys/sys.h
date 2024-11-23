/**
 ****************************************************************************************************
 * @file        sys.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-20
 * @brief       系统初始化代码(包括时钟配置/中断管理/GPIO设置等)
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211103
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __SYS_H
#define __SYS_H

#include "stm32f1xx.h"


/**
 * SYS_SUPPORT_OS用于定义系统文件夹是否支持OS
 * 0,不支持OS
 * 1,支持OS
 */
#define SYS_SUPPORT_OS          0


#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

#define GPIOA_ODR_Addr    (GPIOA_BASE+20) //0x40020014
#define GPIOB_ODR_Addr    (GPIOB_BASE+20) //0x40020414 
#define GPIOC_ODR_Addr    (GPIOC_BASE+20) //0x40020814 
#define GPIOD_ODR_Addr    (GPIOD_BASE+20) //0x40020C14 


#define GPIOA_IDR_Addr    (GPIOA_BASE+16) //0x40020010 
#define GPIOB_IDR_Addr    (GPIOB_BASE+16) //0x40020410 
#define GPIOC_IDR_Addr    (GPIOC_BASE+16) //0x40020810 
#define GPIOD_IDR_Addr    (GPIOD_BASE+16) //0x40020C10 



#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //


/*函数申明*******************************************************************************************/

void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset);             /* 设置中断偏移量 */
void sys_standby(void);                                                         /* 进入待机模式 */
void sys_soft_reset(void);                                                      /* 系统软复位 */
uint8_t sys_clock_set(uint32_t plln);                                           /* 时钟设置函数 */
void sys_stm32_clock_init(uint32_t plln);                                       /* 系统时钟初始化函数 */

/* 以下为汇编函数 */
void sys_wfi_set(void);                                                         /* 执行WFI指令 */
void sys_intx_disable(void);                                                    /* 关闭所有中断 */
void sys_intx_enable(void);                                                     /* 开启所有中断 */
void sys_msr_msp(uint32_t addr);                                                /* 设置栈顶地址 */

#endif












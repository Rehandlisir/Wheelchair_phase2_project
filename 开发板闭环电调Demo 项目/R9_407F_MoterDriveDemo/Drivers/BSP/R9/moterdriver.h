/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/闭环电调Demo/R9_407F_MoterDriveDemo/Drivers/BSP/R9/moterdriver.h
 * @Description  :  Moter Drive Header
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : lisir lisir@rehand.com
 * @LastEditTime : 2024-10-31 09:30:25
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/

#ifndef __MOTERDRIVER_H
#define __MOTERDRIVER_H
#include "./SYSTEM/sys/sys.h"


/******************************* 电机基本驱动  互补输出带死区控制 **************************************/

/* TIMX 互补输出模式 定义 */

/* 主输出通道引脚 */
#define ATIM_TIMX_CPLM_CHY_GPIO_PORT            GPIOA
#define ATIM_TIMX_CPLM_CHY_GPIO_PIN             GPIO_PIN_8
#define ATIM_TIMX_CPLM_CHY_GPIO_CLK_ENABLE()    do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

/* 互补输出通道引脚 */
#define ATIM_TIMX_CPLM_CHYN_GPIO_PORT           GPIOB
#define ATIM_TIMX_CPLM_CHYN_GPIO_PIN            GPIO_PIN_13
#define ATIM_TIMX_CPLM_CHYN_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* PB口时钟使能 */

/* TIMX 引脚复用设置
 * 因为PA8/PB13, 默认并不是TIM1的功能脚, 必须开启复用,PA8/PB13才能用作TIM1的功能
 */
#define ATIM_TIMX_CPLM_CHY_GPIO_AF              GPIO_AF1_TIM1

/* 互补输出使用的定时器 */
#define ATIM_TIMX_CPLM                          TIM1
#define ATIM_TIMX_CPLM_CHY                      TIM_CHANNEL_1
#define ATIM_TIMX_CPLM_CLK_ENABLE()             do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)    /* TIM1 时钟使能 */

/******************************************************************************************/



/*************************************    基本驱动    *****************************************************/

/* 停止引脚操作宏定义 
 * 此引脚控制H桥是否生效以达到开启和关闭电机的效果
 */
#define SHUTDOWN1_Pin                 GPIO_PIN_2
#define SHUTDOWN1_GPIO_Port           GPIOF

#define SHUTDOWN_GPIO_CLK_ENABLE()    do{__HAL_RCC_GPIOF_CLK_ENABLE();}while(0)
/* 电机停止引脚定义 这里默认是接口1 */
#define ENABLE_MOTOR    HAL_GPIO_WritePin(SHUTDOWN1_GPIO_Port,SHUTDOWN1_Pin,GPIO_PIN_SET)
#define DISABLE_MOTOR   HAL_GPIO_WritePin(SHUTDOWN1_GPIO_Port,SHUTDOWN1_Pin,GPIO_PIN_RESET)

/******************************************************************************************/




/*************************************    第三部分    编码器测速    ****************************************************/

#define ROTO_RATIO      128  /* 线数*倍频系数，即32*4=128 */
#define REDUCTION_RATIO 30  /* 减速比30:1 */

/******************************* 第二部分  电机编码器测速 **************************************/

/* 通用定时器 定义  */
#define GTIM_TIMX_ENCODER_CH1_GPIO_PORT         GPIOC
#define GTIM_TIMX_ENCODER_CH1_GPIO_PIN          GPIO_PIN_6
#define GTIM_TIMX_ENCODER_CH1_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)  /* PC口时钟使能 */

#define GTIM_TIMX_ENCODER_CH2_GPIO_PORT         GPIOC
#define GTIM_TIMX_ENCODER_CH2_GPIO_PIN          GPIO_PIN_7
#define GTIM_TIMX_ENCODER_CH2_GPIO_CLK_ENABLE() do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)  /* PC口时钟使能 */

/* TIMX 引脚复用设置
 * 因为PC6/PC7, 默认并不是TIM3的功能脚, 必须开启复用, 才可以用作TIM3的CH1/CH2功能
 */
#define GTIM_TIMX_ENCODERCH1_GPIO_AF            GPIO_AF2_TIM3                                /* 端口复用到TIM3 */
#define GTIM_TIMX_ENCODERCH2_GPIO_AF            GPIO_AF2_TIM3                                /* 端口复用到TIM3 */

#define GTIM_TIMX_ENCODER                       TIM3                                         /* TIM3 */
#define GTIM_TIMX_ENCODER_INT_IRQn              TIM3_IRQn
#define GTIM_TIMX_ENCODER_INT_IRQHandler        TIM3_IRQHandler

#define GTIM_TIMX_ENCODER_CH1                   TIM_CHANNEL_1                                /* 通道1 */
#define GTIM_TIMX_ENCODER_CH1_CLK_ENABLE()      do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)   /* TIM3 时钟使能 */

#define GTIM_TIMX_ENCODER_CH2                   TIM_CHANNEL_2                                /* 通道2 */
#define GTIM_TIMX_ENCODER_CH2_CLK_ENABLE()      do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)   /* TIM3 时钟使能 */

/* 电机参数结构体 */
typedef struct 
{
  uint8_t state;          /*电机状态*/
  float current;          /*电机电流*/
  float volatage;         /*电机电压*/
  float power;            /*电机功率*/
  float speed;            /*电机实际速度*/
  int32_t motor_pwm;      /*设置比较值大小 */
  uint16_t init_current_adc_val; /*电流初始ADC值大小 */
} Motor_TypeDef;

extern Motor_TypeDef g_motor_data;  /*电机参数变量*/


extern TIM_HandleTypeDef g_timx_encode_chy_handle;         /* 定时器x句柄 */
extern TIM_Encoder_InitTypeDef g_timx_encoder_chy_handle;  /* 定时器编码器句柄 */
/* 编码器参数结构体 */
typedef struct 
{
  int encode_old;                  /* 上一次计数值 */
  int encode_now;                  /* 当前计数值 */
  float speed;                     /* 编码器速度 */
} ENCODE_TypeDef;

extern ENCODE_TypeDef g_encode;    /* 编码器参数变量 */
/****************************/
void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc);   /* 高级定时器 互补输出 初始化函数 */                                /* 设置电机方向 */
void dcmotor_init(void);                /* 直流有刷电机初始化 */
void dcmotor_start(void);               /* 开启电机 */
void dcmotor_stop(void);                /* 关闭电机 */  
void dcmotor_dir(uint8_t para);         /* 设置电机方向 */
void dcmotor_speed(double  dutycycle);     /* 设置电机速度 */
void motor_pwm_set(uint8_t para , double  dutycycle);        /* 电机控制 */

void gtim_timx_encoder_chy_init(uint16_t arr, uint16_t psc);
void speed_computer(int32_t encode_now, uint8_t ms);
#endif


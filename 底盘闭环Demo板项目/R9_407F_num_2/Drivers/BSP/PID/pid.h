
#ifndef __PID_H
#define __PID_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* PID相关参数 */

#define  INCR_LOCT_SELECT  1         /* 0：位置式 ，1：增量式 */

#if INCR_LOCT_SELECT

    // /* 增量式PID 速度环参数相关宏 */
    #define  S_KP     0.0035f               /* P参数 0.007*/
    #define  S_KI      0.000025f               /* I参数 */
    #define  S_KD      0.12f               /* D参数*/
    #define  S_SMAPLSE_PID_SPEED  1       /* 采样周期 单位ms*/


    /* 定义电流环（内环）PID参数相关宏 */
    #define  C_KP      0.0010f             /* P参数 */
    #define  C_KI      0.00000012f             /* I参数 */
    #define  C_KD      0.00000001f             /* D参数 */
    #define  C_SMAPLSE_PID_SPEED  50       /* 采样周期 单位ms */
#else

/* 位置式PID参数相关宏 */
    #define  S_KP     0.0035f               /* P参数 0.007*/
    #define  S_KI      0.000025f               /* I参数 */
    #define  S_KD      0.12f               /* D参数*/
    #define  S_SMAPLSE_PID_SPEED  1       /* 采样周期 单位ms*/


    /* 定义电流环（内环）PID参数相关宏 */
    #define  C_KP      1.00f             /* P参数 */
    #define  C_KI      3.00f             /* I参数 */
    #define  C_KD      0.00f             /* D参数 */
    #define  SMAPLSE_PID_SPEED  50       /* 采样周期 单位ms */

#endif



/* PID参数结构体 */
typedef struct
{
    __IO float  SetPoint;            /* 设定目标 */
    __IO float  ActualValue;         /* 期望输出值 */
    __IO float  SumError;            /* 误差累计 */
    __IO float  Proportion;          /* 比例常数 P */
    __IO float  Integral;            /* 积分常数 I */
    __IO float  Derivative;          /* 微分常数 D */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
    uint16_t    smaplse_Pid_detec_time; //PID计算周期计时值
    uint8_t     detect_flage; //PID计算计时值到达标识
} PID_TypeDef;

extern PID_TypeDef  gl_speed_pid;           /* 左轮速度环PID参数结构体 */
extern PID_TypeDef  gl_current_pid;        /* 左轮电流环PID参数结构体 */
extern PID_TypeDef  gr_speed_pid;           /* 右轮速度环PID参数结构体 */
extern PID_TypeDef  gr_current_pid;        /* 右轮电流环PID参数结构体 */
/******************************************************************************************/

void pid_init(void);                 /* pid初始化 */
double increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);//,float max_limit,float min_limit);     /* pid闭环控制 */
void integral_limit( PID_TypeDef *PID , float max_limit, float min_limit );

#endif

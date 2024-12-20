#include "./BSP/PID/pid.h"
#include "./BSP/R9/moterdriver.h"

PID_TypeDef  g_speed_pid;           /* 速度环PID参数结构体 */
PID_TypeDef  g_current_pid;     /* 电流环PID参数结构体 */
/**
 * @brief       pid初始化
 * @param       无
 * @retval      无
 */
void pid_init(void)
{
    /* 初始化速度环PID参数 */
    g_speed_pid.SetPoint = 0;           /* 目标值 */
    g_speed_pid.ActualValue = 0.0;      /* 期望输出值 */
    g_speed_pid.SumError = 0.0;         /* 积分值 */
    g_speed_pid.Error = 0.0;            /* Error[1] */
    g_speed_pid.LastError = 0.0;        /* Error[-1] */
    g_speed_pid.PrevError = 0.0;        /* Error[-2] */
    g_speed_pid.Proportion = S_KP;      /* 比例常数 Proportional Const */
    g_speed_pid.Integral = S_KI;        /* 积分常数 Integral Const */
    g_speed_pid.Derivative = S_KD;      /* 微分常数 Derivative Const */

    /* 初始化电流环PID参数 */
    g_current_pid.SetPoint = 0.0;       /* 目标值 */
    g_current_pid.ActualValue = 0.0;    /* 期望输出值 */
    g_current_pid.SumError = 0.0;       /* 积分值*/
    g_current_pid.Error = 0.0;          /* Error[1]*/
    g_current_pid.LastError = 0.0;      /* Error[-1]*/
    g_current_pid.PrevError = 0.0;      /* Error[-2]*/
    g_current_pid.Proportion = C_KP;    /* 比例常数 Proportional Const */
    g_current_pid.Integral = C_KI;      /* 积分常数 Integral Const */
    g_current_pid.Derivative = C_KD;    /* 微分常数 Derivative Const */
}

/**
 * @brief       pid闭环控制
 * @param       *PID：PID结构体变量地址
 * @param       Feedback_value：当前实际值
 * @retval      期望输出值
 */
double increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* 计算偏差 */
    
#if  INCR_LOCT_SELECT                                                       /* 增量式PID */
    
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))                          /* 比例环节 */
                        + (PID->Integral * PID->Error)                                             /* 积分环节 */
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* 微分环节 */
    
    PID->PrevError = PID->LastError;                                        /* 存储偏差，用于下次计算 */
    PID->LastError = PID->Error;
    
#else                                                                       /* 位置式PID */
    
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error)                       /* 比例环节 */
                       + (PID->Integral * PID->SumError)                    /* 积分环节 */
                       + (PID->Derivative * (PID->Error - PID->LastError)); /* 微分环节 */
    PID->LastError = PID->Error;
    
#endif
    return ((double)(PID->ActualValue));                                   /* 返回计算后输出的数值 */
}

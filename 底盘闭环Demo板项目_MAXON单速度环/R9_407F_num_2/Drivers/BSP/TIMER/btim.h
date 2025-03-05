/**
 ****************************************************************************************************
 * @file        btim.h
 * @author    lis
 * @version     V1.0
 * @date        2024
 * @brief       基本定时器 驱动代码
 * @license     复成医疗
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:rehand 探索者 F407开发板
 * 复成医疗
 * rehand
 *复成医疗
 * 复成医疗
 *
 * 修改说明
 * V1.0 20211015
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __BTIM_H
#define __BTIM_H

#include "./SYSTEM/sys/sys.h"
#include "./BSP/API_Schedule.h"
#include "./BSP/R9/Slavemodbus.h"
#include "./BSP/DAP21/hostdap21.h"
#include "./BSP/Communicationheartbeat/Comheartbeat.h"
#include "./BSP/R9//WheelSpeedMap.h"
#include "./BSP/PID/pid.h"

/******************************************************************************************/
/* 基本定时器 定义 */

/* TIMX 中断定义 
 * 默认是针对TIM6/TIM7
 * 注意: 通过修改这4个宏定义,可以支持TIM1~TIM8任意一个定时器.
 */
 
#define BTIM_TIMX_INT                       TIM6
#define BTIM_TIMX_INT_IRQn                  TIM6_DAC_IRQn
#define BTIM_TIMX_INT_IRQHandler            TIM6_DAC_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)  /* TIM6 时钟使能 */

/******************************************************************************************/
// 过流保护参数（来自用户设定）
#define BOOST_DRIVE_CURRENT       80.0f     // Boost电流阈值 (A)
#define MAX_CURRENT_LIMIT         63.0f     // 正常工作电流阈值 (A)
#define BOOST_DRIVE_TIME_MS       2000      // Boost持续时间 (ms)
#define CURRENT_FOLDBACK_TH       50.0f     // 触发折反的电流阈值 (A) 
#define CURRENT_FOLDBACK_TIME_MS  10000      // 折叠状态触发需超阈值时间 (ms)
#define CURRENT_FOLDBACK_LEVEL    0.5f      // 折叠状态降额比例
#define CURRENT_FOLDBACK          (MAX_CURRENT_LIMIT * CURRENT_FOLDBACK_LEVEL) // 20A
#define COOL_TIME_SEC             (5 * CURRENT_FOLDBACK_TIME_MS)      // 冷却恢复时间 (ms)
#define CONTROL_CYCLE_MS         1       // 控制周期（示例值：1ms）

typedef enum {
  STATE_NORMAL,
  STATE_BOOST,
  STATE_FOLDBACK
} SystemState;

typedef struct {
  SystemState state;
  uint32_t boost_timer;
  uint32_t foldback_timer;
  float I_limit_current ;  // 当前允许的实时电流限值
} ProtectionContext;
 
// IR补偿参数
typedef struct {
  float R_int;           // 电机内阻（在线测量或固定值）
  float V_battery;       // 电池电压（实时测量）
  float duty_cycle;      // 当前输出占空比
  float voltage_moter;         // 电枢电压（在线测量）
} IR_CompensationContext;

extern  ProtectionContext prote_Lmoter;  // 左电机限流器
extern  IR_CompensationContext ircom_Lmoter;  // 左电机IR补偿器
extern  ProtectionContext prote_Rmoter;  // 左电机限流器
extern  IR_CompensationContext ircom_Rmoter;  // 左电机IR补偿器

void btim_timx_int_init(uint16_t arr, uint16_t psc);    /* 基本定时器 定时中断初始化函数 */
void UpdateProtectionState(ProtectionContext* ctx, float I_actual);
float ApplyCurrentLimitedIRCompensation(IR_CompensationContext* irc, 
  ProtectionContext* prot,
  float speed,
  float I_measured);
void ir_compensation_init(void);
int8_t sign(float x);
float min_float(float a, float b) ;
float max_float(float a, float b) ;
#endif


















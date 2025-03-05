/**
 * @FilePath     : /展示样机升级/R9_407F_num_2/Drivers/BSP/demo.c
 * @Description  :  
 * @Author       : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-03-03 15:05:30
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/
// 过流保护参数（来自用户设定）
#define BOOST_DRIVE_CURRENT       100.0f     // Boost电流阈值 (A)
#define MAX_CURRENT_LIMIT         80.0f     // 正常工作电流阈值 (A)
#define BOOST_DRIVE_TIME_MS       2000      // Boost持续时间 (ms)
#define CURRENT_FOLDBACK_TH       63.0f     // 触发折叠的电流阈值 (A) 
#define CURRENT_FOLDBACK_TIME_MS  10000      // 折叠状态触发需超阈值时间 (ms)
#define CURRENT_FOLDBACK_LEVEL    0.6f      // 折叠状态降额比例
#define CURRENT_FOLDBACK          (MAX_CURRENT_LIMIT * CURRENT_FOLDBACK_LEVEL) // 20A
#define COOL_TIME_SEC             (5 * CURRENT_FOLDBACK_TIME_MS / 1000)      // 冷却恢复时间 (s, 25s)

typedef enum {
    STATE_NORMAL,
    STATE_BOOST,
    STATE_FOLDBACK
} SystemState;

typedef struct {
    SystemState state;
    uint32_t boost_timer;
    uint32_t foldback_timer;
    float I_limit_current;  // 当前允许的实时电流限值
} ProtectionContext;

// IR补偿参数
typedef struct {
    float R_int;           // 电机内阻（在线测量或固定值）
    float V_battery;       // 电池电压（实时测量）
    float duty_cycle;      // 当前输出占空比
} IR_CompensationContext;



//================ 核心函数 ================


void UpdateProtectionState(ProtectionContext* ctx, float I_actual) 
{
    //------------ 全局折叠保护触发检测（优先级最高） -------------
    if (ctx->state != STATE_FOLDBACK) {
        /* 折叠保护触发条件监测 */
        if (I_actual >= CURRENT_FOLDBACK_TH) {
            ctx->foldback_timer += CONTROL_CYCLE_MS;
            
            // 如果累计时间达到触发阈值，强制进入折叠保护
            if (ctx->foldback_timer >= CURRENT_FOLDBACK_TIME_MS) {
                ctx->state          = STATE_FOLDBACK;
                ctx->I_limit_current= CURRENT_FOLDBACK;
                ctx->foldback_timer = COOL_TIME_SEC * 1000;    // 初始化冷却倒计时
                ctx->boost_timer    = 0;                       // 清除Boost计时器
                return; // 立即终止状态机，防止其他状态干扰
            }
        } else {
            // 电流低于保护阈值，复位累积计时
            ctx->foldback_timer = 0;
        }
    }

    //------------ 其他状态的常规切换逻辑 -------------
    switch(ctx->state) {
        case STATE_NORMAL:
            /* Boost触发条件检测 */
            if (I_actual > BOOST_DRIVE_CURRENT) {
                ctx->state          = STATE_BOOST;
                ctx->boost_timer    = 0;
                ctx->I_limit_current= BOOST_DRIVE_CURRENT;     // 激活Boost限流
            }
            break;

        case STATE_BOOST:
            /* Boost时间管理 */
            ctx->boost_timer += CONTROL_CYCLE_MS;
            if (ctx->boost_timer >= BOOST_DRIVE_TIME_MS) {
                ctx->state          = STATE_NORMAL; // Boost结束，回归正常
                ctx->I_limit_current= MAX_CURRENT_LIMIT;
            }
            break;

        case STATE_FOLDBACK:
            /* 冷却倒计时管理 */
            if (ctx->foldback_timer > 0) {
                ctx->foldback_timer -= CONTROL_CYCLE_MS;
            } else {
                ctx->state          = STATE_NORMAL; // 冷却完毕，恢复标准限流
                ctx->I_limit_current= MAX_CURRENT_LIMIT;
            }
            break;
    }
}


// 动态IR补偿 + 电流硬约束
float ApplyCurrentLimitedIRCompensation(IR_CompensationContext* irc, 
                                       ProtectionContext* prot,
                                       float V_eff_desired,
                                       float I_measured) {
    // IR补偿公式: V_applied = V_eff需求 + I*R_int
    float V_applied_target = V_eff_desired + I_measured * irc->R_int;

    // 计算IR补偿后的理论占空比
    float duty = V_applied_target / irc->V_battery;

    // Foldback状态下: 动态限制V_eff需求不超过安全反电势
    if(prot->state == STATE_FOLDBACK) {
        // 最大允许反电势 V_eff_max = (I_max_foldback * R_int)
        float V_eff_max = CURRENT_FOLDBACK * irc->R_int;  // 20A * R_int
        V_eff_desired = fminf(V_eff_desired, V_eff_max);
        // 重新计算受限制后的电压需求
        V_applied_target = V_eff_desired + I_measured * irc->R_int;
        duty = V_applied_target / irc->V_battery;
    }

    // 二次限流保护（覆盖所有状态）
    if(I_measured > prot->I_limit_current) {
        // 计算需降低的电流比例：降幅 = (实际电流 - 限流值)/实际电流
        float reduction_ratio = prot->I_limit_current / I_measured;
        duty *= reduction_ratio;
    }

    // 占空比硬约束
    duty = fmaxf(fminf(duty, 1.0f), 0.0f);  
    return duty;
}

/**
 ****************************************************************************************************
 * @file        btim.h
 * @author    lis
 * @version     V1.0
 * @date        2024
 * @brief       ������ʱ�� ��������
 * @license     ����ҽ��
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:rehand ̽���� F407������
 * ����ҽ��
 * rehand
 *����ҽ��
 * ����ҽ��
 *
 * �޸�˵��
 * V1.0 20211015
 * ��һ�η���
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
/* ������ʱ�� ���� */

/* TIMX �ж϶��� 
 * Ĭ�������TIM6/TIM7
 * ע��: ͨ���޸���4���궨��,����֧��TIM1~TIM8����һ����ʱ��.
 */
 
#define BTIM_TIMX_INT                       TIM6
#define BTIM_TIMX_INT_IRQn                  TIM6_DAC_IRQn
#define BTIM_TIMX_INT_IRQHandler            TIM6_DAC_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)  /* TIM6 ʱ��ʹ�� */

/******************************************************************************************/
// �������������������û��趨��
#define BOOST_DRIVE_CURRENT       80.0f     // Boost������ֵ (A)
#define MAX_CURRENT_LIMIT         63.0f     // ��������������ֵ (A)
#define BOOST_DRIVE_TIME_MS       2000      // Boost����ʱ�� (ms)
#define CURRENT_FOLDBACK_TH       50.0f     // �����۷��ĵ�����ֵ (A) 
#define CURRENT_FOLDBACK_TIME_MS  10000      // �۵�״̬�����賬��ֵʱ�� (ms)
#define CURRENT_FOLDBACK_LEVEL    0.5f      // �۵�״̬�������
#define CURRENT_FOLDBACK          (MAX_CURRENT_LIMIT * CURRENT_FOLDBACK_LEVEL) // 20A
#define COOL_TIME_SEC             (5 * CURRENT_FOLDBACK_TIME_MS)      // ��ȴ�ָ�ʱ�� (ms)
#define CONTROL_CYCLE_MS         1       // �������ڣ�ʾ��ֵ��1ms��

typedef enum {
  STATE_NORMAL,
  STATE_BOOST,
  STATE_FOLDBACK
} SystemState;

typedef struct {
  SystemState state;
  uint32_t boost_timer;
  uint32_t foldback_timer;
  float I_limit_current ;  // ��ǰ�����ʵʱ������ֵ
} ProtectionContext;
 
// IR��������
typedef struct {
  float R_int;           // ������裨���߲�����̶�ֵ��
  float V_battery;       // ��ص�ѹ��ʵʱ������
  float duty_cycle;      // ��ǰ���ռ�ձ�
  float voltage_moter;         // �����ѹ�����߲�����
} IR_CompensationContext;

extern  ProtectionContext prote_Lmoter;  // ����������
extern  IR_CompensationContext ircom_Lmoter;  // ����IR������
extern  ProtectionContext prote_Rmoter;  // ����������
extern  IR_CompensationContext ircom_Rmoter;  // ����IR������

void btim_timx_int_init(uint16_t arr, uint16_t psc);    /* ������ʱ�� ��ʱ�жϳ�ʼ������ */
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


















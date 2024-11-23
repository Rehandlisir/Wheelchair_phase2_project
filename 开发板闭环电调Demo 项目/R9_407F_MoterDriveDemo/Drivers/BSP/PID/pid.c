#include "./BSP/PID/pid.h"
#include "./BSP/R9/moterdriver.h"

PID_TypeDef  g_speed_pid;           /* �ٶȻ�PID�����ṹ�� */
PID_TypeDef  g_current_pid;     /* ������PID�����ṹ�� */
/**
 * @brief       pid��ʼ��
 * @param       ��
 * @retval      ��
 */
void pid_init(void)
{
    /* ��ʼ���ٶȻ�PID���� */
    g_speed_pid.SetPoint = 0;           /* Ŀ��ֵ */
    g_speed_pid.ActualValue = 0.0;      /* �������ֵ */
    g_speed_pid.SumError = 0.0;         /* ����ֵ */
    g_speed_pid.Error = 0.0;            /* Error[1] */
    g_speed_pid.LastError = 0.0;        /* Error[-1] */
    g_speed_pid.PrevError = 0.0;        /* Error[-2] */
    g_speed_pid.Proportion = S_KP;      /* �������� Proportional Const */
    g_speed_pid.Integral = S_KI;        /* ���ֳ��� Integral Const */
    g_speed_pid.Derivative = S_KD;      /* ΢�ֳ��� Derivative Const */

    /* ��ʼ��������PID���� */
    g_current_pid.SetPoint = 0.0;       /* Ŀ��ֵ */
    g_current_pid.ActualValue = 0.0;    /* �������ֵ */
    g_current_pid.SumError = 0.0;       /* ����ֵ*/
    g_current_pid.Error = 0.0;          /* Error[1]*/
    g_current_pid.LastError = 0.0;      /* Error[-1]*/
    g_current_pid.PrevError = 0.0;      /* Error[-2]*/
    g_current_pid.Proportion = C_KP;    /* �������� Proportional Const */
    g_current_pid.Integral = C_KI;      /* ���ֳ��� Integral Const */
    g_current_pid.Derivative = C_KD;    /* ΢�ֳ��� Derivative Const */
}

/**
 * @brief       pid�ջ�����
 * @param       *PID��PID�ṹ�������ַ
 * @param       Feedback_value����ǰʵ��ֵ
 * @retval      �������ֵ
 */
double increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* ����ƫ�� */
    
#if  INCR_LOCT_SELECT                                                       /* ����ʽPID */
    
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError))                          /* �������� */
                        + (PID->Integral * PID->Error)                                             /* ���ֻ��� */
                        + (PID->Derivative * (PID->Error - 2 * PID->LastError + PID->PrevError));  /* ΢�ֻ��� */
    
    PID->PrevError = PID->LastError;                                        /* �洢ƫ������´μ��� */
    PID->LastError = PID->Error;
    
#else                                                                       /* λ��ʽPID */
    
    PID->SumError += PID->Error;
    PID->ActualValue = (PID->Proportion * PID->Error)                       /* �������� */
                       + (PID->Integral * PID->SumError)                    /* ���ֻ��� */
                       + (PID->Derivative * (PID->Error - PID->LastError)); /* ΢�ֻ��� */
    PID->LastError = PID->Error;
    
#endif
    return ((double)(PID->ActualValue));                                   /* ���ؼ�����������ֵ */
}

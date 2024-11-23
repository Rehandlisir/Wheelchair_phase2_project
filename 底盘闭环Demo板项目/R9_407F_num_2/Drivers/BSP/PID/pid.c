#include "./BSP/PID/pid.h"
#include "./BSP/R9/moterdriver.h"

PID_TypeDef  gl_speed_pid;           /* �����ٶȻ�PID�����ṹ�� */
PID_TypeDef  gl_current_pid;        /* ���ֵ�����PID�����ṹ�� */
PID_TypeDef  gr_speed_pid;           /* �����ٶȻ�PID�����ṹ�� */
PID_TypeDef  gr_current_pid;        /* ���ֵ�����PID�����ṹ�� */
/**
 * @brief       pid��ʼ��
 * @param       ��
 * @retval      ��
 */
void pid_init(void)
{
/*����*/
    /* ��ʼ���ٶȻ�PID���� */
    gl_speed_pid.SetPoint = 0;           /* Ŀ��ֵ */
    gl_speed_pid.ActualValue = 0.0;      /* �������ֵ */
    gl_speed_pid.SumError = 0.0;         /* ����ֵ */
    gl_speed_pid.Error = 0.0;            /* Error[1] */
    gl_speed_pid.LastError = 0.0;        /* Error[-1] */
    gl_speed_pid.PrevError = 0.0;        /* Error[-2] */
    gl_speed_pid.Proportion = S_KP;      /* �������� Proportional Const */
    gl_speed_pid.Integral = S_KI;        /* ���ֳ��� Integral Const */
    gl_speed_pid.Derivative = S_KD;      /* ΢�ֳ��� Derivative Const */

    /* ��ʼ��������PID���� */
    gl_current_pid.SetPoint = 0.0;       /* Ŀ��ֵ */
    gl_current_pid.ActualValue = 0.0;    /* �������ֵ */
    gl_current_pid.SumError = 0.0;       /* ����ֵ*/
    gl_current_pid.Error = 0.0;          /* Error[1]*/
    gl_current_pid.LastError = 0.0;      /* Error[-1]*/
    gl_current_pid.PrevError = 0.0;      /* Error[-2]*/
    gl_current_pid.Proportion = C_KP;    /* �������� Proportional Const */
    gl_current_pid.Integral = C_KI;      /* ���ֳ��� Integral Const */
    gl_current_pid.Derivative = C_KD;    /* ΢�ֳ��� Derivative Const */
/*����*/
    /* ��ʼ���ٶȻ�PID���� */
    gr_speed_pid.SetPoint = 0;           /* Ŀ��ֵ */
    gr_speed_pid.ActualValue = 0.0;      /* �������ֵ */
    gr_speed_pid.SumError = 0.0;         /* ����ֵ */
    gr_speed_pid.Error = 0.0;            /* Error[1] */
    gr_speed_pid.LastError = 0.0;        /* Error[-1] */
    gr_speed_pid.PrevError = 0.0;        /* Error[-2] */
    gr_speed_pid.Proportion = S_KP;      /* �������� Proportional Const */
    gr_speed_pid.Integral = S_KI;        /* ���ֳ��� Integral Const */
    gr_speed_pid.Derivative = S_KD;      /* ΢�ֳ��� Derivative Const */

    /* ��ʼ��������PID���� */
    gr_current_pid.SetPoint = 0.0;       /* Ŀ��ֵ */
    gr_current_pid.ActualValue = 0.0;    /* �������ֵ */
    gr_current_pid.SumError = 0.0;       /* ����ֵ*/
    gr_current_pid.Error = 0.0;          /* Error[1]*/
    gr_current_pid.LastError = 0.0;      /* Error[-1]*/
    gr_current_pid.PrevError = 0.0;      /* Error[-2]*/
    gr_current_pid.Proportion = C_KP;    /* �������� Proportional Const */
    gr_current_pid.Integral = C_KI;      /* ���ֳ��� Integral Const */
    gr_current_pid.Derivative = C_KD;    /* ΢�ֳ��� Derivative Const */

}

/**
 * @brief       pid�ջ�����
 * @param       *PID��PID�ṹ�������ַ
 * @param       Feedback_value����ǰʵ��ֵ
 * @retval      �������ֵ
 */
double increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value)//,float max_limit,float min_limit)
{
    PID->Error = (float)(PID->SetPoint - Feedback_value);                   /* ����ƫ�� */
#if  INCR_LOCT_SELECT                                                      /* ����ʽPID */
    PID->ActualValue += (PID->Proportion * (PID->Error - PID->LastError)) + 
                        (PID->Integral * PID->Error);                                            /* ���ֻ��� */
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
/**
 * @brief       �����޷�
 * @param       *PID��PID�ṹ�������ַ
 * @param       max_limit�����ֵ
 * @param       min_limit����Сֵ
 * @retval      ��
 */
void integral_limit( PID_TypeDef *PID , float max_limit, float min_limit )
{
    if (PID->SumError >= max_limit)                           /* �����޷� */
    {
        PID->SumError = max_limit;                            /* ���ƻ��� */
    }
    else if (PID->SumError <= min_limit)                      /* �����޷� */
    {
        PID->SumError = min_limit;
    }
}

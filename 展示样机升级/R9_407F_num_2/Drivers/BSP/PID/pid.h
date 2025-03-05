
#ifndef __PID_H
#define __PID_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* PID��ز��� */
#define  SAMPLINGTIMES  10         /* 0��λ��ʽ ��1������ʽ */
#define  INCR_LOCT_SELECT  1         /* 0��λ��ʽ ��1������ʽ */
#if INCR_LOCT_SELECT

    /* ����ʽPID �����ٶȻ�������غ� */
    #define  S_KP_L      0.006250000f               /* P���� 0.007*/                 /* P���� 0.007*/
    #define  S_KI_L      0.000008f                   /* I���� */
    #define  S_KD_L      0.0f                     /* D����*/ 
    /* ����ʽPID �����ٶȻ�������غ� */
    #define  S_KP_R     0.006250000f                   /* P���� 0.007*/
    #define  S_KI_R     0.000008f                 /* I���� */
    #define  S_KD_R     0.0f                   /* D����*/ 
    /*����ʽPID���ֵ�����PID������غ�*/
    #define  C_KP_L      0.00625f           /* P���� */
    #define  C_KI_L      0.000010f            /* I���� */
    #define  C_KD_L      0.00000000f             /* D���� */
    /*����ʽPID���ֵ�����PID������غ�*/
    #define  C_KP_R      0.00625f           /* P���� */
    #define  C_KI_R      0.000010f            /* I���� */
    #define  C_KD_R      0.00000000f             /* D���� */
#else

/* λ��ʽPID������غ� */
    #define  S_KP     0.0035f               /* P���� 0.007*/
    #define  S_KI      0.000025f               /* I���� */
    #define  S_KD      0.12f               /* D����*/
    #define  S_SMAPLSE_PID_SPEED  1       /* �������� ��λms*/


    /* ������������ڻ���PID������غ� */
    #define  C_KP      1.00f             /* P���� */
    #define  C_KI      3.00f             /* I���� */
    #define  C_KD      0.00f             /* D���� */
    #define  SMAPLSE_PID_SPEED  50       /* �������� ��λms */

#endif



/* PID�����ṹ�� */
typedef struct
{
    __IO float  SetPoint;            /* �趨Ŀ�� */
    __IO float  ActualValue;         /* �������ֵ */
    __IO float  SumError;            /* ����ۼ� */
    __IO float  Proportion;          /* �������� P */
    __IO float  Integral;            /* ���ֳ��� I */
    __IO float  Derivative;          /* ΢�ֳ��� D */
    __IO float  Couple_Proportion;   /* ���������ϵ�� */
    __IO float  Error;               /* Error[1] */
    __IO float  LastError;           /* Error[-1] */
    __IO float  PrevError;           /* Error[-2] */
    __IO float  CoupleError;          /*������ */
    uint16_t    smaplse_Pid_detec_time; //PID�������ڼ�ʱֵ
    uint8_t     detect_flage; //PID�����ʱֵ�����ʶ
    float       test_data;
} PID_TypeDef;

extern PID_TypeDef  gl_speed_pid;           /* �����ٶȻ�PID�����ṹ�� */
extern PID_TypeDef  gl_current_pid;        /* ���ֵ�����PID�����ṹ�� */
extern PID_TypeDef  gr_speed_pid;           /* �����ٶȻ�PID�����ṹ�� */
extern PID_TypeDef  gr_current_pid;        /* ���ֵ�����PID�����ṹ�� */
/******************************************************************************************/

void pid_init(void);                 /* pid��ʼ�� */
double increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);
void integral_limit( PID_TypeDef *PID , float max_limit, float min_limit );

#endif

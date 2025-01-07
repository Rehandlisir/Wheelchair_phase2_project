/**
 ****************************************************************************************************
 * @file        btim.c
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
#include "./BSP/TIMER/btim.h"
#include "./BSP/R9/brake.h"
#include "./BSP/CAN/can.h"
#include "./BSP/ADC/adc.h"
// STRUCT_BRAKE struc_brake;
TIM_HandleTypeDef g_timx_handler;         /* ��ʱ��������� */

/**
 * @brief       ������ʱ��TIMX��ʱ�жϳ�ʼ������
 * @note
 *              ������ʱ����ʱ������APB1,��PPRE1 �� 2��Ƶ��ʱ��
 *              ������ʱ����ʱ��ΪAPB1ʱ�ӵ�2��, ��APB1Ϊ42M, ���Զ�ʱ��ʱ�� = 84Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr : �Զ���װֵ��
 * @param       psc : ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    g_timx_handler.Instance = BTIM_TIMX_INT;                      /* ��ʱ��x */
    g_timx_handler.Init.Prescaler = psc;                          /* ��Ƶ */
    g_timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;         /* ��������ģʽ */
    g_timx_handler.Init.Period = arr;                             /* �Զ�װ��ֵ */
    HAL_TIM_Base_Init(&g_timx_handler);
    
    HAL_TIM_Base_Start_IT(&g_timx_handler);                       /* ʹ�ܶ�ʱ��x�Ͷ�ʱ�������ж� */
}

/**
 * @brief       ��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
                �˺����ᱻHAL_TIM_Base_Init()��������
 * @param       ��
 * @retval      ��
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();                     /* ʹ��TIMxʱ�� */
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 1, 1); /* ��ռ1�������ȼ�3 */
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);         /* ����ITMx�ж� */
    }
}

/**
 * @brief       ������ʱ��TIMX�жϷ�����
 * @param       ��
 * @retval      ��
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_handler);  /* ��ʱ���ص����� */
}

/**
 * @brief       �ص���������ʱ���жϷ���������
 * @param       ��
 * @retval      ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    float templv;
    float templI;
    float temprv;
    float temprI;
    if (htim->Instance == BTIM_TIMX_INT)
    {
		OS_IT_RUN();
       /*ҡ��canͨѶ�����ź���0 ����*/
       static uint8_t joy_t;
       static uint8_t pid_t;       
        joy_t++;
        if (joy_t>200)
        {
            CanjoysticbufReceive[4] =0;
            joy_t=0;
        }
        /**************************************��·��� �����ѹ���������ٶȲɼ�***************************************************/
        templv = (g_adc_val[0]-g_adc_val[3])*ADC2VOLATAGE;
        gl_motor_data.volatage =lowPassFilter(&lowpassl_volatage,templv);//
        if (fabs(gl_motor_data.volatage)<0.02)
        {
            gl_motor_data.volatage = 0.0 ;
        }
        /*�����ɼ��ٶȼ���*/
        if (gl_motor_data.volatage>0)
        {
            templI =  pow(g_adc_val[2],2)*ADC2CURRENT_P1+g_adc_val[2]*ADC2CURRENT_P2+ADC2CURRENT_P3;
            gl_motor_data.current =lowPassFilter(&lowpassl_current,templI);
            if (fabs(gl_motor_data.volatage) <= gl_motor_data.current * MOTER_RA)
            {
                gl_motor_data.speed = 0;
            }
            else
            {
                gl_motor_data.speed = (fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else if (gl_motor_data.volatage <0)  
        {
            templI = pow(g_adc_val[1],2)*ADC2CURRENT_P1+g_adc_val[1]*ADC2CURRENT_P2+ADC2CURRENT_P3;
            gl_motor_data.current =lowPassFilter(&lowpassl_current,templI);
            if (fabs(gl_motor_data.volatage) <= gl_motor_data.current * MOTER_RA )
            {
                gl_motor_data.speed = 0;
            }
            else
            {
                gl_motor_data.speed = -(fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else
        {
            gl_motor_data.volatage = 0;
            gl_motor_data.current = 0;
            gl_motor_data.speed = 0;  

        }
        

//    /**************************************��·��� �����ѹ���������ٶȲɼ�***************************************************/
        /*��·��� �����ѹ���������ٶȲɼ�*/
        temprv = (g_adc_val[10]-g_adc_val[9])*ADC2VOLATAGE;
        gr_motor_data.volatage =lowPassFilter(&lowpassr_volatage,temprv);
        if (fabs(gr_motor_data.volatage)<0.02)
        {
            gr_motor_data.volatage = 0.0;
        }
        /*�����ɼ��ٶȼ���*/
        if (gr_motor_data.volatage>0)
        {
            temprI =g_adc_val[8]*g_adc_val[8]*ADC2CURRENT_P1+g_adc_val[8]*ADC2CURRENT_P2+ADC2CURRENT_P3;
            gr_motor_data.current = lowPassFilter(&lowpassr_current,temprI);
            if (fabs(gr_motor_data.volatage) <= gr_motor_data.current * MOTER_RA )
            {
                gr_motor_data.speed = 0;
            }
            else
            {
                gr_motor_data.speed = (fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else if (gr_motor_data.volatage <0)
        {
            gr_motor_data.current =g_adc_val[7]*g_adc_val[7]*ADC2CURRENT_P1+g_adc_val[7]*ADC2CURRENT_P2+ADC2CURRENT_P3;
            gr_motor_data.current = Value_limitf(0,gr_motor_data.current,(fabs(gr_motor_data.volatage)/MOTER_RA));
            gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
            if (fabs(gr_motor_data.volatage) <= gr_motor_data.current * MOTER_RA )
            {
                gr_motor_data.speed = 0;
            }
            else
            {
                gr_motor_data.speed = -(fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else
        {
            gr_motor_data.volatage = 0;
            gr_motor_data.current = 0;
            gr_motor_data.speed = 0;
        }
     /**************���ҵ�� PWM PID�ջ� ����****************************/	
      static uint8_t tl,tr;
      static float stoplpwm_initvalue,stoprpwm_initvalue;
      if (Struc_ActuPra_Int.adcx < -200 || Struc_ActuPra_Int.adcx > 200 || Struc_ActuPra_Int.adcy > 200 || Struc_ActuPra_Int.adcy < -200)
      {
        gl_motor_data.pwm = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
        gr_motor_data.pwm = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed);
        static uint16_t overcurrent_lcnt,overcurrent_rcnt;
        if (gl_motor_data.current> 70 )
        {
            overcurrent_lcnt ++;
            if (overcurrent_lcnt>5000)
            {
                gl_motor_data.pwm =  gl_motor_data.pwm*0.5;    
            }
            if (overcurrent_lcnt>10000)
            {

                overcurrent_lcnt = 0;
                gl_motor_data.pwm = 0;
            }

        }
        else
        {
            overcurrent_lcnt = 0;

        }

        if (gr_motor_data.current >70)
        {
            overcurrent_rcnt ++;
            if (overcurrent_rcnt>5000)
            {
                gr_motor_data.pwm =  gr_motor_data.pwm*0.5;    
            }
            if (overcurrent_rcnt >10000)
            {
                overcurrent_rcnt = 0;
                gr_motor_data.pwm = 0;
            }
        }
        else
        {
           overcurrent_rcnt = 0;
        }

        /*����ռ�ձ�*/
        gl_motor_data.theory_pwm  =fabs(Struc_ActuPra_Out.L_Velocity *0.083);
        gr_motor_data.theory_pwm = fabs(Struc_ActuPra_Out.R_Velocity *0.083);
        /* ռ�ձ�Լ��*/
        // gl_motor_data.pwm = Value_limitf(-4*gl_motor_data.theory_pwm, gl_motor_data.pwm, 4*gl_motor_data.theory_pwm);
        // gr_motor_data.pwm = Value_limitf(-4*gr_motor_data.theory_pwm, gr_motor_data.pwm, 4*gr_motor_data.theory_pwm);
        /*����ƽ���˲�ռ�ձ��˲�����*/
        gl_motor_data.pwm = filterValue_float(&filter_Lpwm,gl_motor_data.pwm);
        gr_motor_data.pwm = filterValue_float(&filter_Rpwm,gr_motor_data.pwm);
        /*ռ�ձ�Լ��*/
        gl_motor_data.pwm = Value_limitf(-0.8, gl_motor_data.pwm, 0.8);
        gr_motor_data.pwm = Value_limitf(-0.8, gr_motor_data.pwm, 0.8);
      }
      else
      {
        gl_motor_data.pwm = 0.0;
        gr_motor_data.pwm = 0.0;
        // LeftMoterMove(gl_motor_data.pwm,0);
        // RightMoterMove(gr_motor_data.pwm,1);
        //����PID ����
        pid_init();
      }
    }		
}

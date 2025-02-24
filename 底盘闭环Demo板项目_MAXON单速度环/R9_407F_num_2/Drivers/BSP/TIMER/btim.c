
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

int8_t sign(float a)
{
    if (a>0)
    {
        return 1;
    }
    else if (a<0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
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
    //    static uint8_t joy_t;
    //    static uint8_t pid_t;       
    //     joy_t++;
    //     if (joy_t>200)
    //     {
    //         CanjoysticbufReceive[4] =0;
    //         joy_t=0;
    //     }
        /**************************************��·��� �����ѹ���������ٶȲɼ�***************************************************/
        gl_motor_data.volatage = (g_adc_val[3]-g_adc_val[0])*ADC2VOLATAGE;
        gl_motor_data.volatage =lowPassFilter(&lowpassl_volatage,gl_motor_data.volatage);//
        if (fabs(gl_motor_data.volatage)<0.2)
        {
            gl_motor_data.volatage = 0.0 ;
        }
        /*�����ɼ��ٶȼ���*/
        if (gl_motor_data.volatage<0)
        {
            gl_motor_data.current = (g_adc_val[2]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
            gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
            // if (fabs(gl_motor_data.volatage) <= gl_motor_data.current * MOTER_RA )
            // {
            //     gl_motor_data.speed = 0;
            // }
            // else
            {
                gl_motor_data.speed = -(fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else if (gl_motor_data.volatage >0)  
        {
            gl_motor_data.current  = (g_adc_val[1]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
            gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current );
            // if (fabs(gl_motor_data.volatage) <= gl_motor_data.current * MOTER_RA )
            // {
            //     gl_motor_data.speed = 0;
            // }
            // else
            {
                gl_motor_data.speed = (fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
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
        gr_motor_data.volatage = (g_adc_val[9]-g_adc_val[10])*ADC2VOLATAGE;
        gr_motor_data.volatage =lowPassFilter(&lowpassr_volatage, gr_motor_data.volatage);
        if (fabs(gr_motor_data.volatage)<0.2)
        {
            gr_motor_data.volatage = 0.0;
        }
        /*�����ɼ��ٶȼ���*/
        if (gr_motor_data.volatage<0)
        {
            gr_motor_data.current =(g_adc_val[8]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
            gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
            // if (fabs(gr_motor_data.volatage) <= gr_motor_data.current * MOTER_RA)
            // {
            //     gr_motor_data.speed = 0;
            // }
            // else
            {
                gr_motor_data.speed = -(fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else if (gr_motor_data.volatage >0)
        {
            gr_motor_data.current =(g_adc_val[7]/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
            gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
            // if (fabs(gr_motor_data.volatage) <= gr_motor_data.current * MOTER_RA )
            // {
            //     gr_motor_data.speed = 0;
            // }
            // else
            {
                gr_motor_data.speed = (fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;
            }
        }
        else
        {
            gr_motor_data.volatage = 0;
            gr_motor_data.current = 0;
            gr_motor_data.speed = 0;
        }
     /**************���ҵ�� PWM PID�ջ� ����****************************/	
        // gl_motor_data.pwm = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
        // gr_motor_data.pwm = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed);
        // /*����ռ�ձ�*/
        // gl_motor_data.theory_pwm  =fabs(Struc_ActuPra_Out.L_Velocity *0.083);
        // gr_motor_data.theory_pwm = fabs(Struc_ActuPra_Out.R_Velocity *0.083);
        // /* ռ�ձ�Լ��*/
        // /*����ƽ���˲�ռ�ձ��˲�����*/
        // gl_motor_data.pwm = filterValue_float(&filter_Lpwm,gl_motor_data.pwm);
        // gr_motor_data.pwm = filterValue_float(&filter_Rpwm,gr_motor_data.pwm);
        // /*ռ�ձ�Լ��*/
        // gl_motor_data.pwm = Value_limitf(-0.8, gl_motor_data.pwm, 0.8);
        // gr_motor_data.pwm = Value_limitf(-0.8, gr_motor_data.pwm, 0.8);
        /*�����ѹƽ�ⷽ�� U= I*Ra + Ke*N*/

        /*����ǰ����ѹ*/
        gl_motor_data.volatageff_target =gl_speed_pid.SetPoint * MOTER_CE;//0.75*sign(gl_speed_pid.SetPoint)*gl_motor_data.current * MOTER_RA + gl_speed_pid.SetPoint * MOTER_CE;
       /*����ǰ����ѹ*/
        gr_motor_data.volatageff_target  =gr_speed_pid.SetPoint * MOTER_CE;//0.75*sign(gr_speed_pid.SetPoint)*gr_motor_data.current * MOTER_RA + gr_speed_pid.SetPoint * MOTER_CE;
       /*�ٶȱջ���ѹ����*/
       gl_motor_data.volatagefb_target = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
       gr_motor_data.volatagefb_target = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed); 
       /*���տ��Ƶ�ѹ*/
        gl_motor_data.volatagecmd_target = gl_motor_data.volatagefb_target+gl_motor_data.volatageff_target;
        gr_motor_data.volatagecmd_target = gr_motor_data.volatagefb_target+gr_motor_data.volatageff_target;
        /*��ѹԼ��-����Ԥ�ⷨ*/
        gl_motor_data.predict_current = (gl_motor_data.volatagecmd_target - gl_speed_pid.SetPoint * MOTER_CE)/MOTER_RA;
        gr_motor_data.predict_current = (gr_motor_data.volatagecmd_target - gr_speed_pid.SetPoint * MOTER_CE)/MOTER_RA;
        if (gl_motor_data.predict_current > 80)
        {
            gl_motor_data.volatagecmd_target = 80*MOTER_RA + gl_speed_pid.SetPoint * MOTER_CE;

        }
        if (gr_motor_data.predict_current > 80)
        {
            gr_motor_data.volatagecmd_target = 80*MOTER_RA + gr_speed_pid.SetPoint * MOTER_CE;
        }
        /*����ռ�ձ����*/
        gl_motor_data.pwm = gl_motor_data.volatagecmd_target/g_r9sys_data.r9_battary_v;
        gr_motor_data.pwm = gr_motor_data.volatagecmd_target/g_r9sys_data.r9_battary_v;
        /*ռ�ձ�Լ��*/
        gl_motor_data.pwm = Value_limitf(-0.85, gl_motor_data.pwm, 0.85);
        gr_motor_data.pwm = Value_limitf(-0.85, gr_motor_data.pwm, 0.85);

    }
	
}

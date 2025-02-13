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
    if (htim->Instance == BTIM_TIMX_INT)
    {
		OS_IT_RUN();			
//		Modbus �ӻ���RK3588��д
		if(slavemodbus.timrun != 0)//����ʱ�䣡=0����
			{
			slavemodbus.timout++;
			if(slavemodbus.timout >=8)
			{
				slavemodbus.timrun = 0;
				slavemodbus.reflag = 1;//�����������
			}
				
			}

		if(modbus_dap21.timrun != 0)//����ʱ�䣡=0����
			{
			modbus_dap21.timout++;
			if(modbus_dap21.timout >=8)
				{
					modbus_dap21.timrun = 0;
					modbus_dap21.reflag = 1;//�����������
				}
			
			}
		
			comheartstate.detect_time++; // �������һ��ͨѶ״̬�����
			if (comheartstate.detect_time > 2000) // ������һ�μ���ȥ��1000ms
			{
				comheartstate.detect_falge = 1;
				comheartstate.detect_time = 0; 
			}
			float templv;
			float templI;
			float temprv;
			float temprI;
			/**************************************��·��� �����ѹ���������ٶȲɼ�***************************************************/
			templv = (adcdata.A2_ADC-adcdata.A1_ADC)*ADC2VOLATAGE;
			gl_motor_data.volatage =lowPassFilter(&lowpassl_volatage,templv);//
			if (fabs(gl_motor_data.volatage)<0.02)
			{
				gl_motor_data.volatage = 0.0 ;
			}
			/*�����ɼ��ٶȼ���*/
			if (gl_motor_data.volatage>0)
			{
				gl_motor_data.current = (adcdata.ASH1_DIFF_AD/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);; 
				gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);

					gl_motor_data.speed = -(fabs(gl_motor_data.volatage) - gl_motor_data.current * MOTER_RA)/MOTER_CE;
			}
			else if (gl_motor_data.volatage <0)  
			{
				gl_motor_data.current =(adcdata.ASH2_DIFF_AD/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);  
				gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
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
			gr_motor_data.volatage= (adcdata.B2_ADC -adcdata.B1_ADC)*ADC2VOLATAGE;
			gr_motor_data.volatage =lowPassFilter(&lowpassr_volatage,gr_motor_data.volatage);
			if (fabs(gr_motor_data.volatage)<0.02)
			{
				gr_motor_data.volatage = 0.0;
			}
			/*�����ɼ��ٶȼ���*/
			if (gr_motor_data.volatage>0)
			{
				gr_motor_data.current = (adcdata.BSH1_DIFF_AD /4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
				gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);

				{
					gr_motor_data.speed = -(fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;
	
				}
				
			}
			else if (gr_motor_data.volatage<0)
			{
				gr_motor_data.current = (adcdata.BSH2_DIFF_AD /4096.0 *3.3- 0.9892)/(2.86799*SAMP_RA);
				gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
				gr_motor_data.speed = (fabs(gr_motor_data.volatage) - gr_motor_data.current * MOTER_RA)/MOTER_CE;

			}
			else
			{
				gr_motor_data.volatage = 0;
				gr_motor_data.current = 0;
				gr_motor_data.speed = 0;
			}
		 /**************���ҵ�� PWM PID�ջ� ����****************************/	
				gl_motor_data.pwm = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
				gr_motor_data.pwm = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed);
				/*����ƽ���˲�ռ�ձ��˲�����*/
				// gl_motor_data.pwm = filterValue_float(&filter_Lpwm,gl_motor_data.pwm);
				// gr_motor_data.pwm = filterValue_float(&filter_Rpwm,gr_motor_data.pwm);
				/*ռ�ձ�Լ��*/
				gl_motor_data.pwm = Value_limitf(-0.8, gl_motor_data.pwm, 0.8);
				gr_motor_data.pwm = Value_limitf(-0.8, gr_motor_data.pwm, 0.8);

    }		
}

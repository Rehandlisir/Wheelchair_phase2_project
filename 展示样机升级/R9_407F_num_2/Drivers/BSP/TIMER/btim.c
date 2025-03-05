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
#include "./BSP/LED/led.h"
TIM_HandleTypeDef g_timx_handler;         /* ��ʱ��������� */

ProtectionContext prote_Lmoter;  // ����������
IR_CompensationContext ircom_Lmoter;  // ����IR������
ProtectionContext prote_Rmoter;  // ����������
IR_CompensationContext ircom_Rmoter;  // ����IR������

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
			templv = (adcdata.A1_ADC-adcdata.A2_ADC)*ADC2VOLATAGE;
			gl_motor_data.volatage =lowPassFilter(&lowpassl_volatage,templv);
			// ȥ����ѹ����
			if (fabs(gl_motor_data.volatage)<0.05)
			{
				gl_motor_data.volatage = 0.0 ;
			}
			/*�����ɼ�������*/
			if(gl_motor_data.volatage<0)
			{
				gl_motor_data.current = sign(gl_motor_data.volatage)*(adcdata.ASH1_DIFF_AD/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);

				gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
			}
			else if(gl_motor_data.volatage>0)
			{
				gl_motor_data.current =sign(gl_motor_data.volatage)*(adcdata.ASH2_DIFF_AD/4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);  
				gl_motor_data.current =lowPassFilter(&lowpassl_current,gl_motor_data.current);
			}
			else
			{
				gl_motor_data.current = 0;
			}
			//ȥ����������
			if (fabs(gl_motor_data.current)<0.2)// || gl_motor_data.current<0.0)
			{
				gl_motor_data.current = 0.0 ;
			}
			/*�����ѹ���ٶȼ���*/
			gl_motor_data.volatage_Ra =gl_motor_data.current * MOTER_RA;
			gl_motor_data.speed = (gl_motor_data.volatage - gl_motor_data.volatage_Ra)/MOTER_CE;

			//ȥ���ٶȲ���
			if (fabs(gl_motor_data.speed)<0.5)
			{
				gl_motor_data.speed = 0.0 ;
			}

	//    /**************************************��·��� �����ѹ���������ٶȲɼ�***************************************************/
			/*��·��� �����ѹ���������ٶȲɼ�*/
			gr_motor_data.volatage= (adcdata.B1_ADC -adcdata.B2_ADC)*ADC2VOLATAGE;
			gr_motor_data.volatage =lowPassFilter(&lowpassr_volatage,gr_motor_data.volatage);
			if (fabs(gr_motor_data.volatage)<0.05)
			{
				gr_motor_data.volatage = 0.0;
			}
			/*�����ɼ�������*/
			if(gr_motor_data.volatage<0)
			{
				gr_motor_data.current =sign(gr_motor_data.volatage)*(adcdata.BSH1_DIFF_AD /4096.0 *3.3 - 0.9892)/(2.86799*SAMP_RA);
				gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);

			}
			else if(gr_motor_data.volatage>0)
			{
				gr_motor_data.current =sign(gr_motor_data.volatage)*(adcdata.BSH2_DIFF_AD /4096.0 *3.3- 0.9892)/(2.86799*SAMP_RA);
				gr_motor_data.current = lowPassFilter(&lowpassr_current,gr_motor_data.current);
			}
			else
			{
				gr_motor_data.current = 0;
			}
			//ȥ����������
			if (fabs(gr_motor_data.current)<0.2)//|| gr_motor_data.current<0.0)
			{
				gr_motor_data.current = 0.0 ;
			}
			/*�����ѹ*/
			gr_motor_data.volatage_Ra = gr_motor_data.current * MOTER_RA;
			/*�ٶȼ���*/
			gr_motor_data.speed = (gr_motor_data.volatage - gr_motor_data.volatage_Ra)/MOTER_CE;
           //ȥ���ٶȲ���
			if (fabs(gr_motor_data.speed)<0.5)
			{
				gr_motor_data.speed = 0.0 ;
			}

			
		//  /**************���ҵ���ٶȱջ�PID ����********************************************************************/	
		// 		gl_motor_data.pwm = increment_pid_ctrl(&gl_speed_pid, gl_motor_data.speed);
		// 		gr_motor_data.pwm = increment_pid_ctrl(&gr_speed_pid, gr_motor_data.speed);
				/*��������*/
				// static uint16_t current_conttimes;
				// static uint8_t stopcarflage;
				// if (fabs(gl_motor_data.current > 60.0) || fabs(gr_motor_data.current > 60.0))
				// {
				// 	current_conttimes++;
				// 	if (current_conttimes > 2000)
				// 	{
				// 		stopcarflage = 1;
				// 		current_conttimes = 0;
				// 	}
				// }
				// if (stopcarflage)
				// {  
				// 	//�������޽���ͣ������
				// 	gl_motor_data.pwm = 0;
				// 	gr_motor_data.pwm = 0;
				// 	LED1(0); // �������ָʾ��
				// }
		// 		/*����ƽ���˲�ռ�ձ��˲�����*/
		// 		// gl_motor_data.pwm = filterValue_float(&filter_Lpwm,gl_motor_data.pwm);
		// 		// gr_motor_data.pwm = filterValue_float(&filter_Rpwm,gr_motor_data.pwm);
		// 		/*ռ�ձ�Լ��*/
		// 		c = Value_limitf(-0.8, gl_motor_data.pwm, 0.8);
		// 		gr_motor_data.pwm = Value_limitf(-0.8, gr_motor_data.pwm, 0.8);

		/************************���� + IR ����**********************************/
		
		// gl_motor_data.pwm = ir_dynamic_compensation(&limiterL,Struc_ActuPra_Out.LN_Velocity, gl_motor_data.volatage, gl_motor_data.current, 24.0);
		// gr_motor_data.pwm = ir_dynamic_compensation(&limiterR,Struc_ActuPra_Out.RN_Velocity, gr_motor_data.volatage, gr_motor_data.current, 24.0);
		//��ʼ���������״̬�ṹ��

		// ��������

		UpdateProtectionState(&prote_Lmoter, gl_motor_data.current);
		UpdateProtectionState(&prote_Rmoter, gr_motor_data.current);
		gl_motor_data.pwm=ApplyCurrentLimitedIRCompensation(&ircom_Lmoter, &prote_Lmoter, Struc_ActuPra_Out.LN_Velocity, gl_motor_data.current);
		gr_motor_data.pwm=ApplyCurrentLimitedIRCompensation(&ircom_Rmoter, &prote_Rmoter, Struc_ActuPra_Out.RN_Velocity, gr_motor_data.current);
	}
}

float min_float(float a, float b) 
{
    // ����NaN����Ч��ֵ����������� 
    if (isnan(a)) return b;  // ��a�Ƿ��򷵻�b 
    if (isnan(b)) return a;  // ��b�Ƿ��򷵻�a 
    return (a < b) ? a : b;  // ���ıȽ��߼� 
}


float max_float(float a, float b) 
{
    // ����NaN����Ч��ֵ����������� 
    if (isnan(a)) return b;  // ��a�Ƿ��򷵻�b 
    if (isnan(b)) return a;  // ��b�Ƿ��򷵻�a 
    return (a < b) ? b : a;  // ���ıȽ��߼� 
}

void UpdateProtectionState(ProtectionContext* ctx, float I_actual) 
{
    //------------ ȫ���۵�����������⣨���ȼ���ߣ� -------------
    if (ctx->state != STATE_FOLDBACK) 
	{
        /* �۵���������������� */
        if (fabs(I_actual) >= CURRENT_FOLDBACK_TH) 
		{
            ctx->foldback_timer += CONTROL_CYCLE_MS;
            
            // ����ۼ�ʱ��ﵽ������ֵ��ǿ�ƽ����۵�����
            if (ctx->foldback_timer >= CURRENT_FOLDBACK_TIME_MS) 
			{
                ctx->state          = STATE_FOLDBACK;
                ctx->I_limit_current= CURRENT_FOLDBACK;
                ctx->foldback_timer = COOL_TIME_SEC;    // ��ʼ����ȴ����ʱ
                ctx->boost_timer    = 0;                       // ���Boost��ʱ��
                return; // ������ֹ״̬������ֹ����״̬����
            }
        } 
		else 
		{
            // �������ڱ�����ֵ����λ�ۻ���ʱ
            ctx->foldback_timer = 0;
        }
    }

    //------------ ����״̬�ĳ����л��߼� -------------
    switch(ctx->state) 
	{
        case STATE_NORMAL:
            /* Boost����������� */
            if (fabs(I_actual) > BOOST_DRIVE_CURRENT) {
                ctx->state          = STATE_BOOST;
                ctx->boost_timer    = 0;
                ctx->I_limit_current= BOOST_DRIVE_CURRENT;     // ����Boost����
            }
            break;

        case STATE_BOOST:
            /* Boostʱ����� */
            ctx->boost_timer += CONTROL_CYCLE_MS;
            if (ctx->boost_timer >= BOOST_DRIVE_TIME_MS) {
                ctx->state          = STATE_NORMAL; // Boost�������ع�����
                ctx->I_limit_current= MAX_CURRENT_LIMIT;
            }
            break;

        case STATE_FOLDBACK:
            /* ��ȴ����ʱ���� */
            if (ctx->foldback_timer > 0) {
                ctx->foldback_timer -= CONTROL_CYCLE_MS;
            } else {
                ctx->state          = STATE_NORMAL; // ��ȴ��ϣ��ָ���׼����
                ctx->I_limit_current= MAX_CURRENT_LIMIT;
				ctx->boost_timer = 0; 
            }
            break;
    }
}

// ��̬IR���� + ����ӲԼ��
float ApplyCurrentLimitedIRCompensation( IR_CompensationContext* irc,ProtectionContext* prot,float speed,float I_measured) 
{
	irc->V_battery = g_r9sys_data.r9_battary_v;
    //-- �׶�1�����ڵ�ǰ����ֵʵʱԼ�����綯������ --
    // ����ϵͳ�����κ�״̬��Normal/Boost/Foldback��������̬������������綯��
    float I_max_current = prot->I_limit_current; // ��ǰ״̬������ֵ��40A��50A��20A��
    float V_eff_max = sign(speed)*I_max_current * irc->R_int;
	float V_eff_clamped;
	float V_eff_desired = speed*MOTER_CE ;
	float I_applied ;
	if (V_eff_desired>0)
	{
		V_eff_clamped= min_float(V_eff_desired, V_eff_max)+0.5;
		if (I_measured > I_max_current)  // Ӧ�õ���Լ��
		{
			I_applied = I_max_current;
		}
		else
		{
			I_applied = I_measured;
		}
	}
	else if(V_eff_desired<0)
	{
		V_eff_clamped= max_float(V_eff_desired, V_eff_max)-0.5;	
		if (I_measured < -I_max_current)// Ӧ�õ���Լ��
		{
			I_applied = -I_max_current;
		}
		else				// Ӧ�õ���Լ��
		{
			I_applied = I_measured;
		}

	}
	else
	{
		V_eff_clamped = 0;
		I_applied = 0;
	}
    //-- �׶�2��ִ��IR��������Ŀ���R --
    float V_applied_target = V_eff_clamped + I_applied * irc->R_int;
	//Ŀ���ѹԼ��
	
	V_applied_target = Value_limitf(-irc->V_battery*0.5, V_applied_target, irc->V_battery*0.5);
    //-- �׶�3��ռ�ձ�ʵʱ������������ --
    float duty = V_applied_target / irc->V_battery;
    // if (fabs(I_measured) > I_max_current) 
	// {
    //     // ��ʵ�ʵ������ޣ���������Ȩռ�ձȣ�ǿ��Լ����
    //     duty *= (I_max_current / I_measured);
    // }
	return duty;
}


void ir_compensation_init(void)
{
	prote_Lmoter.I_limit_current = MAX_CURRENT_LIMIT;
	prote_Lmoter.state = STATE_NORMAL;
	prote_Lmoter.boost_timer = 0;
	prote_Lmoter.foldback_timer = 0;
 
	prote_Rmoter.I_limit_current = MAX_CURRENT_LIMIT;
	prote_Rmoter.state = STATE_NORMAL;
	prote_Rmoter.boost_timer = 0;
	prote_Rmoter.foldback_timer = 0;

	ircom_Lmoter.voltage_moter = gl_motor_data.volatage;
	ircom_Lmoter.R_int = MOTER_RA;
	ircom_Lmoter.V_battery = 24.0;

	ircom_Rmoter.voltage_moter = gr_motor_data.volatage;
	ircom_Rmoter.R_int = MOTER_RA;
	ircom_Rmoter.V_battery = 24.0;

	Struc_ActuPra_Out.LN_Velocity=0;
	Struc_ActuPra_Out.RN_Velocity=0;
}

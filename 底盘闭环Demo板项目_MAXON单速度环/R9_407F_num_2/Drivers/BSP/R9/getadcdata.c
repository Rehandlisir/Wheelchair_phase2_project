
// #include "./BSP/ADC/adc.h"
// #include "./BSP/R9/getadcdata.h"
// #include "./BSP/LED/led.h"
// #include "./SYSTEM/delay/delay.h"
// #include "./BSP/Common/common.h"
// #include "./BSP/R9/Slavemodbus.h"
// #include "./BSP/R9/moterdriver.h"

// /****************************************************************************************************
// * @attention
//  * 针对 R9系统的所有ADC 数据采集 ，
//  * 一、   ADC1 数据采集  包含 8 通道
//  * (1) PA0 座椅推杆电流
//  * (2) PA1 举升推杆电流
//  * (3) PA2 PA3 ADCX ADCY 
//    (4) PC2 PC3 左路/右路电机电流检测
//    (5) PC4     电池电量检测
//    (6) PC5     充电状态检测
//  * 二、   ADC3 数据采集  包含 10通道
//  * (1) 推杆1~6  位置检测  PF5 PF3 PF4 PF6 PF8 PF7
//  * (2) 左路电机电动势检测  PF9/PF10
//  * (3) 右路电机电动势检测  PC0 PC1 
// *
// ****************************************************************************************************
// */
// #define ADC1_DMA_BUF_SIZE 10 * 8			/* ADC1 DMA采集 BUF大小, 应等于ADC通道数的整数倍 */
// uint16_t g_adc1_dma_buf[ADC1_DMA_BUF_SIZE]; /* ADC DMA BUF */

// extern uint8_t g_adc1_dma_sta; /* DMA传输状态标志, 0, 未完成; 1, 已完成 */

// #define ADC3_DMA_BUF_SIZE 20 * 10			/* ADC3 DMA采集 BUF大小, 应等于ADC通道数的整数倍 */
// uint16_t g_adc3_dma_buf[ADC3_DMA_BUF_SIZE]; /* ADC DMA BUF */

// extern uint8_t g_adc3_dma_sta; /* DMA传输状态标志, 0, 未完成; 1, 已完成 */

// ADCDATA adcdata;

// void getadcDataInit(void)
// {
// 	Datareset();
// 	adc1_nch_dma_init((uint32_t)&g_adc1_dma_buf);

// 	adc1_nch_dma_enable(ADC1_DMA_BUF_SIZE); /* 启动ADC1 DMA多通道采集 */

// 	adc3_nch_dma_init((uint32_t)&g_adc3_dma_buf);

// 	adc3_nch_dma_enable(ADC3_DMA_BUF_SIZE); /* 启动ADC1 DMA多通道采集 */

// 	joysticData_calculateInit();
// }

// void getadcData(void)
// {
// 	getadc1Data();
// 	getadc3Data();
// }

// void getadc1Data(void)
// {
// 	uint16_t i, j;
// 	uint16_t adc1_alldata;
// 	uint32_t sum;
// 	static float Batcacu_V;
// 	/* 循环显示通道1~通道8的结果 */
// 	for (j = 0; j < 8; j++) /* 遍历8个通道 */
// 	{
// 		sum = 0;									/* 清零 */
// 		for (i = 0; i < ADC1_DMA_BUF_SIZE / 8; i++) /* 每个通道采集了10次数据,进行10次累加 */
// 		{
// 			sum += g_adc1_dma_buf[(8 * i) + j]; /* 相同通道的转换数据累加 */
// 		}
// 		adc1_alldata = sum / (ADC1_DMA_BUF_SIZE / 8); /* 取平均值 */

// 		if (j == 0)
// 		{
// 			adcdata.chairplate_current = adc1_alldata;// 待补充实际电流转换关系
// 		}

// 		if (j == 1)
// 		{
// 			adcdata.lift_current = adc1_alldata;//待补充实际电流转换关系
// 		}
// 		// printf("adcdata.LED24_CURRENT:%dadcdata.LED12_CURRENT:%d\n",adcdata.LED24_CURRENT,adcdata.LED12_CURRENT);
// 		if (j == 2)
// 		{
// 			if (X_BASEreturn)
// 			{
// 				adcdata.adc_x =adcdata.adc_xbase - adc1_alldata;	//	1800 - adc1_alldata; //	
// 			}
// 			else
// 			{
// 				adcdata.adc_x =adc1_alldata - adcdata.adc_xbase ;	//	1800 - adc1_alldata; //	
// 			}
																  
// 			// adcdata.adc_x = Value_limit(xadc_min, Value_Resetzero(-xadc_Dim, adcdata.adc_x, xadc_Dim), xadc_max); // 设定耦合模糊区域及上下限值限制

// 		}
// 		if (j == 3)
// 		{
// 			adcdata.adc_y =	adc1_alldata - adcdata.adc_ybase;	//	adc1_alldata - 1800;	 //														  
// 			// adcdata.adc_y = Value_limit(yadc_min, Value_Resetzero(-yadc_Dim, adcdata.adc_y, yadc_Dim), yadc_max); // 设定耦合模糊区域及上下限数值限制
			
// 		}
// 		if (j == 4)
// 		{
// 			adcdata.l_current = adc1_alldata;
// 			/*100A 对应 3V ； 33.33333333333333A/V   ， 1240.909090909091D/V  *   0.0268620268620269A/D*/  ; 
// 			adcdata.l_currentAct= adcdata.l_current  * 0.0268620268620269;
// 		}
// 		if (j == 5)
// 		{
// 			adcdata.r_current = adc1_alldata;
// 			/*100A 对应 3V ； 33.33333333333333A/V   ， 1240.909090909091D/V  *   0.0268620268620269A/D*/  ; 
// 			adcdata.r_currentAct= adcdata.r_current  * 0.0268620268620269;
// 		}
// 		if (j == 6)
// 		{
// 			adcdata.bat_v = low_pass_filter7(adc1_alldata,0.08);
			
// 			if (g_slaveReg[22] == 0 && linerun_cmd==iddle_cmd )
// 			{	
// 				// 待更新为 26.2V  对应采样电压 2.015384615384615 V   对应  2500
// 				Batcacu_V =  0.0105 * adcdata.bat_v;
// 				if (Batcacu_V >=26.0)
// 				{
// 					g_slaveReg[1] = 100;
// 				}
// 				else if (Batcacu_V >25.4 && Batcacu_V<26.2)
// 				{
// 					g_slaveReg[1] =85;

// 				}
// 				else if(Batcacu_V >=23 && Batcacu_V<=25.4)
				
// 				{
// 					g_slaveReg[1] = roundToNDecimalPlaces(0.2917*Batcacu_V-6.558,2)*100.0;
				
// 				}
// 				else
// 				{
// 					g_slaveReg[1] =15;

// 				}
			
// 			}
// 			g_slaveReg[1] = (uint8_t)Value_limit(0, g_slaveReg[1], 100); // RK3588 读取 电池电量信息
			
// 		}
// 		if (j == 7)
// 		{
// 			adcdata.chargeI_adc = adc1_alldata; // 待补充优化充电条件
//             /*5A -2.25v  ~   2.25* 65535 /3.3 =49,647.72727272727 ~  5A/49,647.72727272727 =A/D =1.007e-4  , */
//             adcdata.chargeI_adcAct = adcdata.chargeI_adc * 1.007e-4 ;
//             // printf("adcdata.chargeI_adc:%d,adcdata.chargeI_adcAct:%f\n",adcdata.chargeI_adc, adcdata.chargeI_adcAct);
// 			/*实际测量 未充电 电流0.155481 ~ 0.156790 */ 
// 			if (adcdata.chargeI_adcAct > 0.17) // RK3588 读取 电池充电状态信息
// 			{
// 				g_slaveReg[2] = 1;
// 			}
// 			else
// 			{
// 				g_slaveReg[2] = 0;
// 			}
// 		}
// 	}
// 	g_adc1_dma_sta = 0;						/* 清除DMA采集完成状态标志 */
// 	adc1_nch_dma_enable(ADC1_DMA_BUF_SIZE); /* 启动下一次ADC1 DMA多通道采集 */
// }
// /*将一个double 类型的数四舍五入到n位小数*/
// double roundToNDecimalPlaces(double num, int n) 
// {
//     double factor = pow(10, n);
//     return round(num * factor) / factor;
// }

// /**
//  * @brief        : 
//  *  * 二、   ADC3 数据采集  包含 10通道
//  * (1) 推杆1~6  位置检测  PF5 PF3 PF4 PF6 PF8 PF7
//  * (2) 左路电机电动势检测  PF9/PF10
//  * (3) 右路电机电动势检测  PC0 PC1 
//  * @return        {*}
// **/
// void getadc3Data(void)
// {
// 	uint16_t i, j;
// 	uint16_t adc3_alldata;
// 	uint32_t sum;
// 	for (j = 0; j < 10; j++) /* 遍历10个通道 */
// 	{
// 		sum = 0;									 /* 清零 */
// 		for (i = 0; i < ADC3_DMA_BUF_SIZE / 10; i++) /* 每个通道采集了20次数据,进行20次累加 */
// 		{
// 			sum += g_adc3_dma_buf[(10 * i) + j]; /* 相同通道的转换数据累加 */
// 		}
// 		adc3_alldata = sum / (ADC3_DMA_BUF_SIZE / 10); /* 取平均值 */
		

// 		if (j == 0)
// 		{
// 			adcdata.backboard_pos = low_pass_filter1(adc3_alldata,0.08);
// 		}
			
// 		if (j == 1)
// 		{
// 			adcdata.pedestal_pos = low_pass_filter2(adc3_alldata,0.08);
// 		}
// 		if (j == 2)
// 		{	
// 			adcdata.lift_pos = low_pass_filter3(adc3_alldata,0.08);
// 		}
// 		if (j == 3)
// 		{
// 			adcdata.legangle_pos = low_pass_filter4(adc3_alldata,0.08);
// 		}
// 		if (j == 4)
// 		{	
// 			adcdata.leglength_pos = low_pass_filter5(adc3_alldata,0.08);
// 		}
// 		if (j == 5)
// 		{	
// 			adcdata.support_pos = low_pass_filter6(adc3_alldata,0.08);
// 		}
//         /*左右路电机反电动势 3.3~4096   检测电压 = 测量电压 *21   反电动势 = 给定电压 - 电枢电压 */
// 		if (j == 6)
// 		{	
// 			adcdata.A1_ADC = adc3_alldata;
// 			adcdata.A1V = (double) (adcdata.A1_ADC * 3.3 /4096 * 21.0)-0.0169;
// 		}
// 		if (j == 7)
// 		{	
// 			adcdata.A2_ADC = adc3_alldata;
// 			adcdata.A2V =  (double)(adcdata.A2_ADC * 3.3 /4096 * 21.0) -0.0169;
// 		}
// 		if (j == 8)
// 		{
// 			adcdata.B1_ADC = adc3_alldata;
// 			adcdata.B1V =  (double)(adcdata.B1_ADC * 3.3 /4096 * 21.0 )-0.0169;
// 		}
// 		if (j == 9)
// 		{
// 			adcdata.B2_ADC = adc3_alldata;
// 			adcdata.B2V =  (double)(adcdata.B2_ADC * 3.3 /4096 * 21.0)-0.0169;
// 		}
// 		// printf("adcdata.A1V :%f,adcdata.A2V:%f,adcdata.B1V:%f ,adcdata.B2V:%f\n",\
// 		adcdata.A1V,adcdata.A2V ,adcdata.B1V,adcdata.B2V);
// 	}
   

// 	g_adc3_dma_sta = 0;						/* 清除DMA采集完成状态标志 */
// 	adc3_nch_dma_enable(ADC3_DMA_BUF_SIZE); /* 启动下一次ADC1 DMA多通道采集 */
// }

// void joysticData_calculate(void)
// {
// 	uint32_t i, j, k;
// 	uint32_t adc_current;
// 	uint32_t sum, sumbasex = 0, sumbasey = 0;
// 	// delay_ms(2000); //等待2 s 待硬件上电OK
// 	for (k = 0; k < 1000; k++)	/* 转换 70次                  */
// 	{							/* 循环显示通道0~通道5的结果 */
// 		for (j = 2; j < 4; j++) /* 遍历6个通道 */
// 		{
// 			sum = 0;									/* 清零 */
// 			for (i = 0; i < ADC1_DMA_BUF_SIZE / 8; i++) /* 每个通道采集了10次数据,进行10次累加 */
// 			{
// 				sum += g_adc1_dma_buf[(8 * i) + j]; /* 相同通道的转换数据累加 */
// 			}
// 			adc_current = (uint32_t)(sum / (ADC1_DMA_BUF_SIZE / 8)); /* 取平均值 */
// 																	 // 缓存基准值
// 			if (j == 2)
// 				adcdata.adc_xbase = adc_current;

// 			if (j == 3)
// 				adcdata.adc_ybase = adc_current;
// 			// printf("Xbase:%d,Ybase:%d\t\n",adcdata.adc_xbase,adcdata.adc_xbase);
// 		}
// 		sumbasex += adcdata.adc_xbase;
// 		sumbasey += adcdata.adc_ybase;
// 	}
// 	adcdata.adc_xbase = (int32_t)sumbasex / 1000;
// 	adcdata.adc_ybase = (int32_t)sumbasey / 1000;
// 	sumbasex = 0;
// 	sumbasey = 0;
// 	g_adc1_dma_sta = 0;						/* 清除DMA采集完成状态标志 */
// 	adc1_nch_dma_enable(ADC1_DMA_BUF_SIZE); /* 启动下一次ADC1 DMA多通道采集 */
// }

// void joysticData_calculateInit(void)
// {
// 	uint32_t k;
// 	for (k = 0; k < 50; k++)
// 	{
// 		joysticData_calculate();
// 	}
// }

// void Datareset(void)
// {
// 	adcdata.adc_x = 0;
// 	adcdata.adc_y = 0;
// 	adcdata.adc_xbase = 0;
// 	adcdata.adc_ybase = 0;
// 	adcdata.l_current = 0;
// 	adcdata.r_current = 0;
// 	adcdata.A1_ADC = 0;
// 	adcdata.A2_ADC = 0;
// 	adcdata.B1_ADC = 0;
// 	adcdata.B2_ADC = 0;

// 	adcdata.lift_pos = 0;
// 	adcdata.pedestal_pos = 0;
// 	adcdata.backboard_pos = 0;
// 	adcdata.legangle_pos = 0;
// 	adcdata.leglength_pos = 0;
// 	adcdata.support_pos = 0;

// }

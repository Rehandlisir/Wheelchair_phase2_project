/*
 * @Author: lisir lisir@rehand.com
 * @Date: 2024-06-07 16:01:18
 * @LastEditors: lisir lisir@rehand.com
 * @LastEditTime: 2024-08-15 11:00:21
 * @FilePath: \R9_407F_NO2\R9_407F_Num_2\R9_407F_num_2\Drivers\BSP\task.c
 * @Description: 主任务列表
 */
#include "./BSP/task.h"

/**
 * @description: 
 * @return {*}
 */

/**
 * @description: 所有任务初始化代码
 * @return {*}
 */
void Hard_devInit(void)
{

		HAL_Init();                                 //* 初始化HAl库 */
		dcmotor_init();
		sys_stm32_clock_init(336, 8, 2, 7);     /* 初始化时钟频率,168Mhz 主时钟*/
		delay_init(168);                        /*初始化延时时钟频率*/
		usart_init(115200);                     /* 串口通讯波特率 115200 */
		led_init();                             /* 转向灯初始化 */
		btim_timx_int_init(1000 - 1, 84 - 1);   /*定时器中断初始化 产生固定 1ms 的定时器中断 */
		// iwdg_init(IWDG_PRESCALER_64, 1500);      /* 预分频数为64,重载值为1500,溢出时间约为3s */
		filterInit();                    /*初始化滤波器*/
		vSetUpMlx90393();
		atim_timx_cplm_pwm_init(100-1,84-1);
		gtim_timx_encoder_chy_init(0XFFFF, 0);  /* 编码器定时器初始化，不分频直接84M的计数频率 */
		adc_nch_dma_init();
		getcurrent_initADC(); // 获取电流初始AD值
		pid_init();
		printf("Enter initialization\n");
}

void Task_GetMlx90393(void)
{

	vInMeasurementNormal();

}

/**
 * @description: 灯控程序
 * @return {*}
 */
void Task_led_control(void)
{
	LED0_TOGGLE();
	LED1_TOGGLE();

}

/**
 * @description: 
 *  针对 R9系统的所有ADC 数据采集 ，
 *  一 、ADC1 采集7通道数据 包含   
 * (1)  摇杆数据采集         PA2 PA3
 * (2)  抱闸 数据监测        PA4 PA5
 * (3)  底盘电机电流检测     PA6 PA7
 * (4)  电池电压            PC7 pc5
/
 * 二、   ADC3 数据采集  包含
 * (1) 推杆1~6  位置检测  PF5 PF3 PF4 PF6 PF8 PF7
 * (2) 推杆 1~6 的电流检测 PC2 PC3 PC0 PC1 PF9 PF10
 * @return {*}
 */
void Task_GetADC_AllData(void)
{

	get_actualmoterdata();

}	

/**
 * @description: 底盘控制既驱动执行
 * @return {*}
 */
void Task_UnderpanDrive(void)
{
	underpanExcute();
}


void Task_R9DataScope(void)
{
	//  printf("/*R9car:%f,%f,%f,%f,%f,%f*/\n",\
	//  adcdata.A1V,adcdata.A2V,\
	//  adcdata.B1V,adcdata.B2V,\
	//  adcdata.l_currentAct,\
	//  adcdata.r_currentAct);

	// printf("moterspeed %.1f RPM\r\n",g_motor_data.speed);                  /* 打印速度值 */
	// printf("voltage:%.2fV,current:%fmA\r\n", moterdata.power_voltage, moterdata.moter_current); 
	// printf("/*R9car:%.1f,%.2f,%f*/\n",g_speed_pid.SetPoint ,fabs(g_motor_data.speed),Struc_ActuPra_Out.L_Dutycycle);
	printf("%f,%f,%f,%f\t\n",g_current_pid.SetPoint ,fabs(g_motor_data.speed),Struc_ActuPra_Out.L_Dutycycle*100, g_motor_data.current);
}
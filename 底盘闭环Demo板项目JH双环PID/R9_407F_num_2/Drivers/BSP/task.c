/*
 * @Author: lisir lisir@rehand.com
 * @Date: 2024-06-07 16:01:18
 * @LastEditors: lisir lisir@rehand.com
 * @LastEditTime: 2024-08-15 11:00:21
 * @FilePath: \R9_407F_NO2\R9_407F_Num_2\R9_407F_num_2\Drivers\BSP\task.c
 * @Description: 主任务列表
 */
#include "./BSP/task.h"

// extern ADCDATA adcdata;

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
	MoterdriveInit();
	sys_stm32_clock_init(168, 6, 2, 4);     /* 初始化时钟频率,168Mhz 主时钟*/
	delay_init(168);                        /*初始化延时时钟频率*/
	usart_init(115200);                     /* 串口通讯波特率 115200 */
	led_init();                             /* 转向灯初始化 */
	btim_timx_int_init(10 - 1, 8400 - 1);   /*定时器中断初始化 产生固定 1ms 的定时器中断 */
	brake_init();                           /*抱闸初始化*/   
	can_init(CAN_SJW_1TQ, CAN_BS2_6TQ, CAN_BS1_7TQ, 6, CAN_MODE_NORMAL);  /* CAN初始化, 正常模式, 波特率500Kbps */
	iwdg_init(IWDG_PRESCALER_64, 1500);      /* 预分频数为64,重载值为1500,溢出时间约为3s */
	filterInit();                    /*初始化滤波器*/
	lowpass_init();
	adc3_nch_dma_init();
	pid_init();
	// vSetUpMlx90393();
}

void Task_GetMlx90393(void)
{

	// vInMeasurementNormal();
	CanRead_joystic_excute();

}

/**
 * @description: 灯控程序
 * @return {*}
 */
void Task_led_control(void)
{
	LED1_TOGGLE(); 
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

	// printf("ASH1_AD_F:%d,ASH2_AD_F:%d,\t\n",g_adc_val[7],g_adc_val[8]);

/*闭环采样数据 ：1 左轮规划速度 2 左轮实际速度  3左轮电枢电压  4 左轮电枢电流 
5 右轮规划速度 6 右轮实际速度  7右轮电枢电压  8 右轮电枢电流  9 左轮占空比  10 右轮占空比*/
printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n\t",gl_speed_pid.SetPoint,gl_motor_data.speed,gl_motor_data.volatage,gl_motor_data.current,\
gr_speed_pid.SetPoint,gr_motor_data.speed,gr_motor_data.volatage,gr_motor_data.current,gl_motor_data.pwm,gr_motor_data.pwm,gl_motor_data.pidspeed_out);

// printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n\t",gl_current_pid.SetPoint,gl_motor_data.speed,gl_motor_data.volatage,gl_motor_data.current,\
// gr_current_pid.SetPoint,gr_motor_data.speed,gr_motor_data.volatage,gr_motor_data.current,gl_motor_data.pwm,gr_motor_data.pwm);


}




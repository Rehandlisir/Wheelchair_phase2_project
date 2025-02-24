
#include "./BSP/TASK/task.h"
#include "math.h"

void Hard_devInit(void)
{

	HAL_Init();                                 //* 初始化HAl库 */
	sys_stm32_clock_init();     /* 初始化时钟频率,168Mhz 主时钟*/
	delay_init(168);                        /*初始化延时时钟频率*/
	usart_init(115200);                     /* 串口通讯波特率 115200 */
	btim_timx_int_init(1000 - 1, 168 - 1);   /*定时器中断初始化 产生固定 1ms 的定时器中断 */
	MX_FDCAN1_Init();/* CAN初始化, 正常模式, 波特率500Kbps */
	// iwdg_init(IWDG_PRESCALER_64, 2000);      /* 时间计算(大概):Tout=((4 * 2^prer) * rlr) / 32 (ms). 约4秒*/
	filterInit();
	AS5013_Init();
}

void Task_GetAsh5013(void)
{
	As5013_excute();
}

/**
 * @description: 底盘控制既驱动执行
 * @return {*}
 */
void Task_CanjoysticRun(void)
{
	ASH5013_CAN_Trans(0x0002,As5013data.x_raw,As5013data.y_raw);
	
}
void Task_R9DataScope(void)
{

	printf("%d,%d\t\n",As5013data.x_raw,As5013data.y_raw);//,L_Velocity,R_Velocity);
}




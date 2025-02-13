#include "./BSP/Exception_handling/exception_handling.h"
void ex_handl_Init(void)
{
	linearActuatordec_init();
}
void ex_handl_brake(uint16_t cnt_times)
{
//	static uint16_t cntL,cntR;
//	// 左路电机离合器判定
//	if (LEFT_BREAK_STATE)
//	{
//        cntL++;
//		if (cntL> cnt_times) // 连续cnt_times次都是高电平 判定离合器断开
//		{
//			g_slaveReg[4] = 1;
//			cntL = 0;
//		}
//	}
//	else
//	{
//		cntL = 0;
//		g_slaveReg[4] = 1;
//	}
//	// 右路电机离合器判定
//	if (RIGHT_BRAKE_STATE)
//	{
//        cntR++;
//		if (cntR> cnt_times) // 连续cnt_times次都是高电平 判定离合器断开
//		{
//			g_slaveReg[27] = 1;
//			cntR = 0;
//		}
//	}
//	else
//	{
//		cntR = 0;
//		g_slaveReg[27] = 1;
//	}	
//	// printf("LEFT_BREAK_STATE:%d,RIGHT_BRAKE_STATE:%d\n",LEFT_BREAK_STATE,RIGHT_BRAKE_STATE);
}

void ex_handl_LRmoter(void)
{
	
	
	
	;
	
	
}
void linearActuatordec_init(void)
{
	;
}


void ex_handl_linearActuator(void)
{	
	
	CanCmdled(LED_GREEN,LED_GREEN,LED_GREEN,LED_GREEN,LED_GREEN);
}

void ex_handl_battary(void)
{
	
	;
	
}

void ex_handl_joystic(void)
{
	
	
	;
	
}

void ex_handl_indicatorlight(void)
{
	
	;
}

void ex_handl_excute(void)
{
	
	ex_handl_brake(200);
	
	ex_handl_LRmoter();
	
	ex_handl_linearActuator();
	
	ex_handl_battary();
	
	ex_handl_joystic();
	
}

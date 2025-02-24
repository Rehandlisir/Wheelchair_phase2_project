/*
 * @Author: lisir lisir@rehand.com
 * @Date: 2024-06-07 16:01:18
 * @LastEditors: lisir lisir@rehand.com
 * @LastEditTime: 2024-08-15 11:00:21
 * @FilePath: \R9_407F_NO2\R9_407F_Num_2\R9_407F_num_2\Drivers\BSP\task.c
 * @Description: 主任务列表
 */
#include "./BSP/task.h"

extern ADCDATA adcdata;

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
		getadcDataInit();                      /*ADC数据采集初始化*/
		Host_ModbusDap21_Init();              /*与DYPA21通讯*/
		SlaveModbus_Init();                  /*与RK3588作为从机通讯*/
		can_init(CAN_SJW_1TQ, CAN_BS2_6TQ, CAN_BS1_7TQ, 6, CAN_MODE_NORMAL);  /* CAN初始化, 正常模式, 波特率500Kbps */
		iwdg_init(IWDG_PRESCALER_64, 1500);      /* 预分频数为64,重载值为1500,溢出时间约为3s */
		filterInit();                    /*初始化滤波器*/
//		vSetUpMlx90393();
		g_slaveReg[0] = 0x68;//本机设备作为Modbus从机时的设备ID
		printf("Enter initialization\n");
}

void Task_GetMlx90393(void)
{

//	vInMeasurementNormal();
	CanRead_joystic_excute();

}

/**
 * @description: 灯控程序
 * @return {*}
 */
void Task_led_control(void)
{
	led_beepControlRK3588();
//	led_beepControl();
	LED2_TOGGLE();

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

	getadcData();

}	
void Task_KineCacu(void)
{
	legKinematics();
}
/**
 * @description: 底盘控制既驱动执行
 * @return {*}
 */
void Task_Velocitymaping(void)
{
	mapingExcute();
}

void Task_Moter_Run(void)
{
	moter_run();
}

/**
 * @description: 推杆控制及驱动
 * @return {*}
 */
void Task_linearactuatorDrive(void)
{	
//	linearactuatorTest(); 

}

/**
 * @description: MPU6050 执行
 * @return {*}
 */
void Task_gyroscopeData(void)
{
//	MPU6050Excute();
}

/**
 * @description: 与RK3588 通讯程序执行
 * @return {*}
 */
void Task_ModbusSlaveExecute (void)
{	
	SlaveModbus_Event();//Modbus事件处理函数(执行读或者写的判断)--从机地址0x01	
}

/**
 * @description: 超声波测距程序，目前单从没问题
 * @return {*}
 */
void Task_ultrasonicreadExecute1 (void)
{
			HostDap21_Read03_slave(0x01,0x0101,0x0001);//参数1从机地址，参数2起始地址，参数3寄存器个数
			
			if(modbus_dap21.Host_send_flag)
			{

				modbus_dap21.Host_send_flag=0;//清空发送结束数据标志位

				HOST_ModbusDap21RX();//接收数据进行处理
			}

}

void Task_ultrasonicreadExecute2 (void)
{
			HostDap21_Read03_slave(0x02,0x0100,0x0001);//参数1从机地址，参数2起始地址，参数3寄存器个数			
			if(modbus_dap21.Host_send_flag)
			{
				modbus_dap21.Host_send_flag=0;//清空发送结束数据标志位
				HOST_ModbusDap21RX();//接收数据进行处理
			}
			
			// 仅在驾驶过程或未在充电 且按键板未在操作中传输有效数据
//			if((g_slaveReg[10] || g_slaveReg[11]) && g_slaveReg[2]==0 && linerun_cmd==iddle_cmd)
//			{
//				g_slaveReg[7] = dap21Data.dyplength2 ; /*超声波数据通过MOdbus上传至上位机进行显示/应用*/   
//			}	
//			else 
//			{

//				g_slaveReg[7] = 2000 ; /*超声波数据通过MOdbus上传至上位机进行显示/应用*/   
//			}

	
}


/**
 * @description: 按键板Can通讯实现
 * @return {*}
 */
void Task_CanKeyRun(void)
{
      CanRead_KEY_excute();
}
void Task_ex_handl(void)
{

//	ex_handl_excute();

}

void Task_Comsdetect(void)
{	

	if (comheartstate.detect_falge)
	{
		ComheartReset();
		comheartstate.detect_falge =0;	
	}
	ComheartDetect(1000);
	
}

void Task_R9DataScope(void)
{
	printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n\t",gl_speed_pid.SetPoint,gl_motor_data.speed,gl_motor_data.volatage,gl_motor_data.current,\
	gr_speed_pid.SetPoint,gr_motor_data.speed,gr_motor_data.volatage,gr_motor_data.current,gl_motor_data.pwm,gr_motor_data.pwm);
}
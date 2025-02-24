#include "./BSP/AS5013/AS5013.h"
I2C_HandleTypeDef hi2c1; // 全局I2C句柄
AS5013_Data As5013data;

void AS5013_IIC_Init(void)
{
	__HAL_RCC_I2C1_CLK_ENABLE();
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing =0x00707CBB; // APB1=168MHz时的CubeMX生成值 400KHz通讯频率 0x30F3A7F0
	hi2c1.Init.OwnAddress1 = 0;		// 主模式无需地址
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&hi2c1);
}



void AS5013_GPIO_Init(void)
{
	// IIC GPIO
	// 1. 使能GPIO时钟
	__HAL_RCC_GPIOA_CLK_ENABLE(); // PA15(SCL)
	__HAL_RCC_GPIOB_CLK_ENABLE(); // PB7(SDA)
	// 2. 配置SCL(PA15)和SDA(PB7)
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // 复用开漏
	GPIO_InitStruct.Pull = GPIO_NOPULL;		// 依赖外部上拉
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1; // 确认AF4支持PA15/PB7
	// SCL (PA15)
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	// SDA (PB7)
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	// RST GPIO
	GPIO_InitStruct.Pin = AS5031_RST_GPIO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(AS5031_RST_PORT, &GPIO_InitStruct);
}

void MX_GPIO_Init(void)
{
	// 配置PA4为外部中断
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // 下降沿触发
	GPIO_InitStruct.Pull = GPIO_PULLUP;			 // 上拉防误触
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	// 设置中断优先级
	HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0); // EXTI4对应PA4
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}


void AS5013_Init(void)
{
	AS5013_GPIO_Init();
	AS5013_IIC_Init();
	MX_GPIO_Init();
	/*硬件复位*/
	AS5031REST(0);
	delay_ms(1);
	AS5031REST(1);
	// 软复位
	uint8_t ctrl_reg1 = 0x02; // Soft reset bit[1]
	HAL_I2C_Mem_Write(&hi2c1, AS5013_I2C_ADDRESS, AS5013_CONTROL_REG_1, I2C_MEMADD_SIZE_8BIT, &ctrl_reg1, 1, 100);
	delay_ms(1);
	// 配置为Idle Mode并开启中断（寄存器0Fh: Idle=1）
	uint8_t config_mode = 0x84; // Bit7=1 (Idle Mode 0x80  Idle下启用中断模式 0x84)
	HAL_I2C_Mem_Write(&hi2c1, AS5013_I2C_ADDRESS, AS5013_CONTROL_REG_1, I2C_MEMADD_SIZE_8BIT, &config_mode, 1, 100);
	delay_ms(1);
	// 配置Control Register 2（磁极方向）
	uint8_t ctrl_reg2 = 0x84; // inv_spinning=1（根据磁铁方向调整若反向了调整为0x84）
	HAL_I2C_Mem_Write(&hi2c1, AS5013_I2C_ADDRESS, AS5013_CONTROL_REG_2, I2C_MEMADD_SIZE_8BIT, &ctrl_reg2, 1, 100);
	delay_ms(1);
    // 配置缩放 因子
	uint8_t scal_reg = 0x27;
	HAL_I2C_Mem_Write(&hi2c1, AS5013_I2C_ADDRESS, AS5013_SCALING_CONTROL_REG, I2C_MEMADD_SIZE_8BIT, &scal_reg, 1, 100);
	delay_ms(1);
}

// 读取AS5013数据
void Read_XY(int8_t *x, int8_t *y)
{
	// 读取X寄存器
	HAL_I2C_Mem_Read(&hi2c1, AS5013_I2C_ADDRESS, AS5013_X_REG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)x, 1, 100);
	// 读取Y寄存器（自动清除中断）
	HAL_I2C_Mem_Read(&hi2c1, AS5013_I2C_ADDRESS, AS5013_Y_RESET_INT_REG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)y, 1, 100);
}
void ReadAs5013ID(uint8_t *ID)
{
	HAL_I2C_Mem_Read(&hi2c1, AS5013_I2C_ADDRESS, AS5013_ID_CODE_REG, I2C_MEMADD_SIZE_8BIT, (uint8_t *)ID, 1, 100);
}


// EXTI4中断处理
void EXTI4_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_FLAG(GPIO_PIN_4))
	{
		
		
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4); // 清除中断标志
	}
}

void As5013_excute(void)
{
	ReadAs5013ID(&As5013data.As5013ID);
	
	Read_XY(&As5013data.x_raw, &As5013data.y_raw);
	//均值滤波
	As5013data.x_raw =filterValue_int8(&filter_ash5031_xdata,As5013data.x_raw);
	As5013data.y_raw =filterValue_int8(&filter_ash5031_ydata,As5013data.y_raw);
	
//	printf("%d,%d\t\n",As5013data.x_raw,As5013data.y_raw);
}
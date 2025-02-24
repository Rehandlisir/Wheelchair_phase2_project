/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/第二阶段新项目/总线式摇杆项目/JOYSTIC_HAL/Drivers/BSP/CAN/can.c
 * @Description  :  CAN
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2024-11-11 16:46:43
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/

#include "./BSP/CAN/can.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/AS5013/AS5013.h"
FDCAN_RxHeaderTypeDef fdcan1_RxHeader;
FDCAN_TxHeaderTypeDef fdcan1_TxHeader;
FDCAN_HandleTypeDef hfdcan1;
/**
 * @brief       CAN初始化
 * @param       tsjw    : 重新同步跳跃时间单元.范围: 1~3;
 * @param       tbs2    : 时间段2的时间单元.范围: 1~8;
 * @param       tbs1    : 时间段1的时间单元.范围: 1~16;
 * @param       brp     : 波特率分频器.范围: 1~1024;
 *   @note      以上4个参数, 在函数内部会减1, 所以, 任何一个参数都不能等于0
 *              CAN挂在APB1上面, 其输入时钟频率为 Fpclk1 = PCLK1 = 168Mhz
 *              tq     = brp * tpclk1;
 *              波特率 = Fpclk1 / ((tbs1 + tbs2 + 1) * brp);
 *             
 *              我们设置 则CAN波特率为:
 *              168MHZ / ((6+ 7 + 1) * 24) = 500Kbps
 *
 * @param       mode    : CAN_MODE_NORMAL,  普通模式;
                          CAN_MODE_LOOPBACK,回环模式;
 * @retval      0,  初始化成功; 其他, 初始化失败;
 */
uint8_t MX_FDCAN1_Init(void)
{
	FDCAN_FilterTypeDef FDCAN1_RXFilter;
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 24;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 6;
  hfdcan1.Init.NominalTimeSeg2 = 7;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    return 1;
  }
	FDCAN1_RXFilter.IdType=FDCAN_STANDARD_ID;                       //标准ID
	FDCAN1_RXFilter.FilterIndex=0;                                  //滤波器索引                   
	FDCAN1_RXFilter.FilterType=FDCAN_FILTER_MASK;                   //滤波器类型
	FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	FDCAN1_RXFilter.FilterID1=0x0000;                               //32位ID
	FDCAN1_RXFilter.FilterID2=0x0000;                               //如果FDCAN配置为传统模式的话，这里是32位掩码
	if(HAL_FDCAN_ConfigFilter(&hfdcan1,&FDCAN1_RXFilter)!=HAL_OK) //滤波器初始化
	{
		 return 1;
	}
    HAL_FDCAN_Start(&hfdcan1);                               //开启FDCAN
    HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
}
 
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{
 
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      ;
    }
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 
  //   /* FDCAN1 interrupt Init */
  //   HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
  //   HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  // /* USER CODE BEGIN FDCAN1_MspInit 1 */
 
  /* USER CODE END FDCAN1_MspInit 1 */
  }
}
 

/* USER CODE BEGIN 1 */
//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8),可设置为FDCAN_DLC_BYTES_2~FDCAN_DLC_BYTES_8				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
uint8_t FDCAN1_Send_Msg(uint32_t id,uint8_t* msg,uint32_t length)
{	
		
  fdcan1_TxHeader.Identifier = id;
  fdcan1_TxHeader.IdType = FDCAN_STANDARD_ID;
  fdcan1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  fdcan1_TxHeader.DataLength = length;
  fdcan1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  fdcan1_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  fdcan1_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  fdcan1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  fdcan1_TxHeader.MessageMarker = 0;//0x52; //由于网上借鉴该函数，我也不太明白为什么是0x52，不过我测试改成0好像也没问题    
  if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&fdcan1_TxHeader,msg)!=HAL_OK) 
  {
    return 1;//发送
  }



}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//其他,接收的数据长度;
uint8_t FDCAN1_Receive_Msg(uint8_t *buf, uint16_t *Identifier,uint16_t *len)
{	
    
  if(HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&fdcan1_RxHeader,buf)!=HAL_OK)return 0;//接收数据
  *Identifier = fdcan1_RxHeader.Identifier;
  *len=fdcan1_RxHeader.DataLength>>16;
  return fdcan1_RxHeader.DataLength>>16;	
}
 
void ASH5013_CAN_Trans(uint8_t id,int8_t x,int8_t y)
{
	int8_t src_data[8] = {x,y,id,0,0,0,0,0}; // 原始数据 
  uint8_t send_data[8];
	for (int i = 0; i < sizeof(src_data); i++) 
	{
    send_data[i] = (uint8_t)src_data[i]; // 强制类型转换 
	}
	
  FDCAN1_Send_Msg(id,send_data,FDCAN_DLC_BYTES_8); // id 0x0002

}
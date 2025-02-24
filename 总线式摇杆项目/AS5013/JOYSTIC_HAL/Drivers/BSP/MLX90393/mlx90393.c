#include "./BSP/MLX90393/mlx90393.h"
#include "./SYSTEM/delay/delay.h"

MLX90393Data mlxdata;
#define MachineNo_2 //�ڶ�̨�� 
AverageFilter filter_mlx_xdata;
AverageFilter filter_mlx_ydata;

void MLX90393_SDA_OUT(void)
{
	
	 /*    HAL��ʹ�ã�HAL��ע��Ҫ�ѳ�ʼ�������ľ�̬���ȥ��    */
	__HAL_RCC_GPIOB_CLK_ENABLE();   //ʹ��GPIOBʱ��
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_7;                    //ʹ��STM3cubemx�Ƕ���õ�SDA_PinΪGPIO_PIN_8�ı�ǩ
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
}

void MLX90393_SDA_IN(void)
	
{
		__HAL_RCC_GPIOB_CLK_ENABLE();   //ʹ��GPIOBʱ��
		GPIO_InitTypeDef GPIO_InitStruct;
		GPIO_InitStruct.Pin = GPIO_PIN_7    ;                    //ʹ��STM3cubemx�Ƕ���õ�SDA_PinΪGPIO_PIN_8�ı�ǩ
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
}



void MLX90393_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
   /*B7*/
    __HAL_RCC_GPIOB_CLK_ENABLE();   //ʹ��GPIOBʱ��
	 __HAL_RCC_GPIOA_CLK_ENABLE();   //ʹ��GPIOAʱ��
	
	
    GPIO_Initure.Pin=GPIO_PIN_7;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;     //����
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    /*A15*/
    GPIO_Initure.Pin=GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
    MLX90393_SDA(1);
    MLX90393_SCL(1);  
}

//����MLX90393��ʼ�ź�
void MLX90393_Start(void)
{
	MLX90393_SDA_OUT();     //sda�����
	MLX90393_SDA(1);	  	  
	MLX90393_SCL(1);
	delay_us(4);
 	MLX90393_SDA(0);//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	MLX90393_SCL(0);//ǯסI2C���ߣ�׼�����ͻ�������� 
	delay_us(2);
}	  
//����MLX90393ֹͣ�ź�
void MLX90393_Stop(void)
{
	MLX90393_SDA_OUT();//sda�����
	MLX90393_SCL(0);
	MLX90393_SDA(0);//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	MLX90393_SCL(1); 	
	MLX90393_SDA(1);//����I2C���߽����ź�
    delay_us(4);
    MLX90393_NAck(); //ֹͣ����Ҫ����Ӧ��
							   	
}
//�ȴ�Ӧ���źŵ��� ��һ��Nack�ź�
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t MLX90393_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	// unsigned char staturs;
	MLX90393_SDA_IN();      //SDA����Ϊ����   
    MLX90393_SDA(1);delay_us(1);
	MLX90393_SCL(1);delay_us(1);
    while (MLX90393_READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime>250)
        {
          MLX90393_Stop();  
          return 1;  
        }
    }
	MLX90393_SCL(0);//ʱ�����0 	
    return 0;
} 
//����ACKӦ�� дһ�� ACK�ź�  һ���������ݶ�ȡ��ɺ��ٲ�����
void MLX90393_Ack(void)
{
	MLX90393_SCL(0);
	MLX90393_SDA_OUT();
	MLX90393_SDA(0);
	delay_us(2);
	MLX90393_SCL(1);
	delay_us(2);
	MLX90393_SCL(0);
	
}
//������ACKӦ��		    
void MLX90393_NAck(void)
{
	MLX90393_SCL(0);
	MLX90393_SDA_OUT();
	MLX90393_SDA(1);
	delay_us(2);
	MLX90393_SCL(1);
	delay_us(2);
	MLX90393_SCL(0);
}					 				     
//MLX90393����һ���ֽ�
//���شӻ�����Ӧ��
//1������Ӧ��ʧ��
//0������Ӧ��ɹ�			  
uint8_t MLX90393_Send_Byte(uint8_t WriteByte)
{     
    uint8_t i;
    uint8_t staturs;
    MLX90393_SDA_OUT();  //Set-->SDA As OutPutMode
    MLX90393_SCL(0);//����ʱ�ӿ�ʼ���ݴ���
    for(i = 0; i < 8; i++)
    {
        MLX90393_SDA ( (WriteByte&0x80)>>7);
        WriteByte<<=1;
        delay_us(2);
        MLX90393_SCL(1);
        delay_us(2);
        MLX90393_SCL(0);
        delay_us(2);
    }
    staturs = MLX90393_Wait_Ack();//д�����ݺ�Ҫ��ȴ� ACK�ź�
    return (staturs);
}	
		    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t MLX90393_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MLX90393_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        MLX90393_SCL(0); 
        delay_us(2);
		MLX90393_SCL(1);
        receive<<=1;
        if(MLX90393_READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        MLX90393_NAck();//����nACK
    else
        MLX90393_Ack(); //����ACK   
    return receive;
}

//��90393дһ������ ucSlaveAdd Ϊд��ַ
unsigned char ucMLX90393WriteCommand(unsigned char ucWriteCmd, unsigned char ucSlaveAdd)
{
    unsigned char ucReadStatusData;//��ȡ������ ״̬��
    MLX90393_Start();
    MLX90393_Send_Byte(ucSlaveAdd);
    MLX90393_Send_Byte(ucWriteCmd);
    MLX90393_Start();
    ucSlaveAdd |= 0x01; //read data slave address
    MLX90393_Send_Byte(ucSlaveAdd);
    ucReadStatusData = MLX90393_Read_Byte(0); 
    /*IIC stop*/
    MLX90393_Stop();
    return (ucReadStatusData);
}

//�������ķ�ʽ д�Ĵ��� ucSlaveAdd Ϊд������ַ ucRegAddress Ϊ�Ĵ�����ַ
unsigned char ucMLX90393WriteReg(unsigned char ucRegAddress, unsigned short usWriteData, unsigned char ucSlaveAdd)
{
    unsigned char ucReadStatusData;//��ȡ������ ״̬��
    MLX90393_Start();
    MLX90393_Send_Byte(ucSlaveAdd);
    MLX90393_Send_Byte(MLX90393_WriteRegCommand);
    MLX90393_Send_Byte(usWriteData >> 8);
    MLX90393_Send_Byte(usWriteData);
    MLX90393_Send_Byte(ucRegAddress << 2);
    MLX90393_Start();
    ucSlaveAdd |= 0x01; //read data slave address
    MLX90393_Send_Byte(ucSlaveAdd);
    ucReadStatusData = MLX90393_Read_Byte(0);
   
    /*IIC stop*/
    MLX90393_Stop();
    return(ucReadStatusData);
}

//�������ķ�ʽ ���Ĵ���
unsigned short usMlx90393ReadReg(unsigned char RegAddress, unsigned char SlaveAdd)
{
    
    unsigned int Data;
    RegAddress = RegAddress << 2;
    MLX90393_Start();
    MLX90393_Send_Byte(SlaveAdd);
    MLX90393_Send_Byte(MLX90393_ReadRegCommand);
    MLX90393_Send_Byte(RegAddress);

    MLX90393_Start();
    SlaveAdd	|= 0x01; 			//read data slave address
    MLX90393_Send_Byte(SlaveAdd);

    mlxdata.statusByte = MLX90393_Read_Byte(1);
 
    Data = MLX90393_Read_Byte(1);
 
    Data = Data << 8;
    Data |= MLX90393_Read_Byte(1);
    /*IIC stop*/
    MLX90393_Stop();
    return (Data);
}


//�ж�״̬�Ĵ����Ƿ����
unsigned char ucCheckReadMlxStatusErro(unsigned char ucMlxStatusData, unsigned char ucCmdMode)
{
    if(0xFF == ucMlxStatusData)return 1;//���һ�����е�CLK�����SDA����ʱ��������ߵ�ƽ��˵������λ�Ǵ��
    else if(0x10 & ucMlxStatusData)return 1;//���״̬�Ĵ����д�����ڵ�
    if(MlxInReg_Mode == ucCmdMode)//����Ƕ��Ĵ������е�����
    {
        if(0x00 == ucMlxStatusData)return 0;
        else if(0x10 & ucMlxStatusData)return 1;
    }
    else
    {
        if(0x00 == ucMlxStatusData)return 1;//���һ�����е�CLK�����SDA����ʱ��������ߵ�ƽ��˵������λ�Ǵ��
    }
    return 0;
}

//д������״̬�Ĵ����Ƿ��ǳ���ģ������������ǿ�ƶ��д  ���ʱ��Ϊ ��MaxCmdErroTimes�����
void vCmdMlxAndWaiteErroTimes(unsigned char command, unsigned char SlaveAdd, unsigned char ucErroType)
{
    unsigned char ucReadCycCnt = 0;
    for(ucReadCycCnt = 0; ucReadCycCnt < MaxCmdErroTimes; ucReadCycCnt++)//�������������
    {
        mlxdata.statusByte = ucMLX90393WriteCommand(command, SlaveAdd);//дһ������
        if(ucCheckReadMlxStatusErro(mlxdata.statusByte, MlxInReg_Mode))
        {
             delay_ms(10);
        }
        else return;
    }
    if(ucReadCycCnt > 4) mlxdata.ucMlx90393ErroType = ucErroType;
}



//д�Ĵ��������״̬�Ĵ����Ƿ��ǳ���ģ������������ǿ�ƶ��д  ���ʱ��Ϊ ��MaxCmdErroTimes�����
void vCmdMlxWriteRegAndWaite(uint16_t RegAddress, unsigned short usWriteData , unsigned char SlaveAdd, unsigned char ucErroType)
{
    unsigned char ucReadCycCnt = 0;
    for(ucReadCycCnt = 0; ucReadCycCnt < MaxCmdErroTimes; ucReadCycCnt++)
    {
        mlxdata.statusByte = ucMLX90393WriteReg(RegAddress, usWriteData, SlaveAdd);
        if(ucCheckReadMlxStatusErro(mlxdata.statusByte, MlxInReg_Mode))
        {
				delay_ms(10);
        }
        else return;
    }
    if(ucReadCycCnt > 4) mlxdata.ucMlx90393ErroType = ucErroType;
}



//��90393���ϵ��ʼ������
void vSetUpMlx90393(void)
{
    filter_init(); // �˲���ʼ��
    #define IfMemoryRecallOkWaiteTime_ms 5//�����ͬ�߼�����������֮��ĵȴ���ʱ
    #define IfReadNextRegWaite_ms        2//ÿ����ͬ����֮���ʱ����
    // unsigned char ucReadMlxRegNumIndex = 0;//��ȡоƬ���Ĵ���������
    // unsigned char ucReadCycCnt = 0;//ѭ����������
    unsigned short usReg0_3DataArr[3] = {0};
    mlxdata.ucMlx90393ErroType = 0;//��� ����Ϣ

    MLX90393_IIC_Init(); // IIC ��ʼ��
		

    vCmdMlxAndWaiteErroTimes(MLX90393_ExitCommand, MLX90393slaveAdd_011, MlxErro_InCmdExitErro);//�������˳����в���

    delay_ms(2); //�ȴ� 1ms

    vCmdMlxAndWaiteErroTimes(MLX90393_ResetCommand, MLX90393slaveAdd_011, MlxErro_InCmdExitErro);//��λоƬ

    delay_ms(2); //�ȴ� 2ms 

    if((UseSetMlxReg0Data != usReg0_3DataArr[0]) || (UseSetMlxReg1Data != usReg0_3DataArr[1]) ||
            (UseSetMlxReg2Data != usReg0_3DataArr[2]))//���������Ԥ�ڲ�һ������������д��оƬ
    {
		// delay_ms(1);
        vCmdMlxWriteRegAndWaite(0x00, UseSetMlxReg0Data, MLX90393slaveAdd_011, MlxErro_InWriteRegErro);   // д�Ĵ��� 0x00
        if(mlxdata.ucMlx90393ErroType)return;
        else
        {
			delay_ms(2);
            vCmdMlxWriteRegAndWaite(0x01, UseSetMlxReg1Data, MLX90393slaveAdd_011, MlxErro_InWriteRegErro);// д�Ĵ��� 0x01
        }
        if(mlxdata.ucMlx90393ErroType)return;
        else
        {
			delay_ms(2);
            vCmdMlxWriteRegAndWaite(0x02, UseSetMlxReg2Data, MLX90393slaveAdd_011, MlxErro_InWriteRegErro);// д�Ĵ��� 0x02
        }
        if(mlxdata.ucMlx90393ErroType)return;
        else
        {
			delay_ms(2);
            vCmdMlxAndWaiteErroTimes(MLX90393_MemStoreCommand, MLX90393slaveAdd_011, MlxErro_InStoreRegErro);
        }
		delay_ms(15);
    }
	

		
}
void filter_init(void)
{
	//�˲���ʼ��
	initializeFilter(&filter_mlx_xdata);
	initializeFilter(&filter_mlx_ydata);
}


void vInMeasurementNormal(void) 
{
  unsigned char bufx[2], bufy[2];
  mlxdata.statusByte = ucMLX90393WriteCommand(MLX90393_StartMeasurement,MLX90393slaveAdd_011);//дһ�������
	MLX90393_Start(); // д��ʼλ
	MLX90393_Send_Byte(MLX90393slaveAdd_011); //д�ӻ���ַ 
	MLX90393_Send_Byte(MLX90393_ReadMeausreCommand); //д-������ ֻ��X,Y����ֵ
	MLX90393_Start(); // ��д��ʼλ
	MLX90393_Send_Byte(MLX90393slaveAddAndRead_011); //������ ֻ��X,Y����ֵ
	mlxdata.statusByte = MLX90393_Read_Byte(1); // ��ȡ״̬λ������Ӧ��
	mlxdata.xhdata = MLX90393_Read_Byte(1) ;// ��ȡX ��8λ��λ������Ӧ��
	mlxdata.xldata = MLX90393_Read_Byte(1) ;// ��ȡX ��8λ��λ������Ӧ��
	mlxdata.yhdata = MLX90393_Read_Byte(1) ;// ��ȡY ��8λ��λ������Ӧ��
	mlxdata.yldata = MLX90393_Read_Byte(0) ;// ��ȡY ��8λ��λ��������Ӧ��
	//�Ͱ�λ�͸߰�λ���Ϊ 16λ ����ת�����
    bufx[0] = mlxdata.xldata;
    bufx[1] = mlxdata.xhdata;
    bufy[0] = mlxdata.yldata;
    bufy[1] = mlxdata.yhdata;	
	mlxdata.xdata = ((*((uint8_t *)bufx+ 1)<< 8))| *(uint8_t *)bufx;
	mlxdata.ydata = ((*((uint8_t *)bufy+ 1)<< 8))| *(uint8_t *)bufy;
	MLX90393_Stop();// ��ȡ��ɷ�ֹͣ�źţ�
	//��� ��Ҫ�ٴ�дһ�� ����ģʽ������
	if(ucCheckReadMlxStatusErro(mlxdata.statusByte, MlxInNoneReg_Mode))
	{
			mlxdata.usMlx90393StatusErroTimes++;//���״̬λ������ �ۼӴ���֡������
	}


	if(ucCheckReadMlxStatusErro(mlxdata.statusByte, MlxInNoneReg_Mode))
	{
			mlxdata.usMlx90393StatusErroTimes++;//���״̬λ������ �ۼӴ���֡������
	}


}

void mlx_90393_offset(void)
{
	
	 mlxdata.xdata =  mlxdata.xdata - mlxdata.x_offset;
	
	 mlxdata.ydata = mlxdata.ydata - mlxdata.y_offset;
	

    /*ҡ����Ч���ݶξ�ֵ�˲��˲�*/
    mlxdata.xdata  = filterValue_int32(&filter_mlx_xdata, mlxdata.xdata);
    mlxdata.ydata = filterValue_int32(&filter_mlx_ydata, mlxdata.ydata);
	
}


void mlx_90393_offsetcacu(void)
{
	
	uint16_t t;
	
	for(t=0;t<100;t++) 
	{ 
		vInMeasurementNormal();
		mlxdata.x_offset = mlxdata.xdata; 
		mlxdata.y_offset = mlxdata.ydata; 
//    mlxdata.xdata= mlxdata.xdata - mlxdata.x_offset;
//    mlxdata.ydata= mlxdata.ydata - mlxdata.y_offset;
		delay_ms(2);
		mlxdata.mlxcaculate_end = 0;
//		printf("mlxdata.x_offset:%d,mlxdata.y_offset:%d\n",mlxdata.x_offset,mlxdata.y_offset);
	}
	mlxdata.mlxcaculate_end = 1;
	
}

//�޷�
int32_t Value_limit(int32_t min_value, int32_t current_value, int32_t max_value)
{

  if (current_value < min_value)
    return min_value;
  else if (current_value > max_value)
    return max_value;
  else
    return current_value;
}

// ��ʼ���˲���
void initializeFilter(AverageFilter *filter)
{
  for (int i = 0; i < WINDOW_SIZE; ++i)
  {
    filter->window[i] = 0;
    filter->window_int[i] = 0;
  }
  filter->index = 0;
}

int32_t filterValue_int32(AverageFilter *filter, int16_t input)
{
  // ���»�����
  filter->window_int[filter->index] = input;
  filter->index = (filter->index + 1) % WINDOW_SIZE;

  // ����ƽ��ֵ
  int32_t sum = 0.0;
  for (int i = 0; i < WINDOW_SIZE; ++i)
  {
    sum += filter->window_int[i];
  }
  int16_t average = (int16_t)(sum / WINDOW_SIZE);

  return average;
}

int32_t Value_Resetzero(int32_t min_value, int32_t current_value, int32_t max_value)
{
  if (current_value < min_value || current_value > max_value)
    return current_value;
  else
    return 0;
}


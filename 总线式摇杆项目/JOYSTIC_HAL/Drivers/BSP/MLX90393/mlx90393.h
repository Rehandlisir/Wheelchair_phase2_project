#ifndef __MLX90393_H_
#define __MLX90393_H_

#include "./SYSTEM/sys/sys.h"

//IO接口定义
//#define MLX90393_SDA_IN()  {GPIOB->CRL &=0xF0FFFFFF;GPIOB->CRL|=(uint32_t)4<<28;}	//PB7 输入模式
//#define MLX90393_SDA_OUT() {GPIOB->CRL &=0xF0FFFFFF;GPIOB->CRL|=(uint32_t)3<<28;} //PB7 输出模式
//IO电平操作

#define MLX90393_READ_SDA  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)

#define MLX90393_SDA(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); \
    } while (0) 
				  
		

#define MLX90393_SCL(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET) : HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); \
    } while (0) 
				  

// MLX90393 IIC地址  0001 100R/W   0：W  1：R				  
#define MLX90393slaveAddAndRead_011     0x19   // 读地址
#define MLX90393slaveAdd_011            0x18   // 写地址

// MLX 90393指令
#define MLX90393_BurstReadCommand   0x16        //-yx-Start Burst Mode
#define MLX90393_WeekUpCommand      0x26        //-yx-Start Wake-up on Change Mode
#define MLX90393_SingleReadCommand  0x36        //-yx-Start Single Measurement Mode
#define MLX90393_ReadMeausreCommand 0x46        //-yx-Read Measurement
#define MLX90393_ReadRegCommand     0x50        //Read Register
#define MLX90393_WriteRegCommand    0x60        //Write Register
#define MLX90393_ExitCommand        0x80        //Exit Mode
#define MLX90393_MemRecallCommand   0xd0        //Memory Recall
#define MLX90393_MemStoreCommand    0xe0        //Memory Store
#define MLX90393_ResetCommand       0xf0        //Reset
#define MLX90393_StartMeasurement   0x36        //-yx-




// 寄存器配置设置

#define UseSetMlxReg0Data           0X801C

#define UseSetMlxReg1Data           0xE180
#define UseSetMlxReg2Data           0x01E5

#define MaxCmdErroTimes   10//??????????
#define IfMlxStatusByteErroWaiteNextCmdTime_ms   10 //???MLX ?????????????????????????????????????

#define MlxErro_InCallMemoryCall   0x01
#define MlxErro_InReadRegErro      0x02
#define MlxErro_InCmdExitErro      0x03
#define MlxErro_InWriteRegErro     0x04
#define MlxErro_InStoreRegErro     0x05
#define MlxErro_CheckUseFulDataErro 0x06
#define MlxInReg_Mode              1
#define MlxInNoneReg_Mode          0
#define MlxCycOprtationIcDelayTime 100//?????????????IC????????

#define MlxErro_StatusByteErro     0xFF

#define FALSE                      0
#define TRUE                       1


union uniRecUsefulMemoryData__
{
    unsigned short usRecUsefulMemoryData[10];
    struct strMlxAdjust__
    {
        unsigned short usMiddleX : 16;
        unsigned short usMiddleY : 16;
        unsigned short usLeftX : 16;
        unsigned short usLeftY : 16;
        unsigned short usRightX : 16;
        unsigned short usRightY : 16;
        unsigned short usUpX : 16;
        unsigned short usUpY : 16;
        unsigned short usDownX : 16;
        unsigned short usDownY : 16;
    } strMlxAdjust;
};




typedef  struct
{
	union uniRecUsefulMemoryData__  uniRecUsefulMemoryData;
	uint8_t xldata;
	uint8_t xhdata;
	uint8_t yldata;
	uint8_t yhdata;
  uint8_t statusByte;
  	int16_t xdata_source;
	int16_t ydata_source;
	int16_t xdata;
	int16_t ydata;
  uint16_t x_offset ;
  uint16_t y_offset ;
	unsigned char ucMlx90393ErroType;
	unsigned short usMlx90393StatusErroTimes;
  uint8_t mlxcommstatus;
	
} MLX90393Data;

#define WINDOW_SIZE 10
typedef struct {
    double window[WINDOW_SIZE];
	int16_t window_int[WINDOW_SIZE];
    uint8_t index;
} AverageFilter;

extern AverageFilter filter_mlx_xdata;
extern AverageFilter filter_mlx_ydata;

extern MLX90393Data mlxdata;

#define MAX_XDATA 3500
#define MIN_XDATA -3500
#define MAX_YDATA 3500
#define MIN_YDATA -3500
#define YADC_DIM 500  
#define XADC_DIM 500  
void MLX90393_SDA_OUT(void);
void MLX90393_SDA_IN(void);
void MLX90393_IIC_Init(void);                //��ʼ��MLX90393��IO��				 
void MLX90393_Start(void);				//����MLX90393��ʼ�ź�
void MLX90393_Stop(void);	  			//����MLX90393ֹͣ�ź�
uint8_t MLX90393_Send_Byte(uint8_t WriteByte);
uint8_t MLX90393_Read_Byte(unsigned char ack);//MLX90393��ȡһ���ֽ�
uint8_t MLX90393_Wait_Ack(void); 				//MLX90393�ȴ�ACK�ź�
void MLX90393_Ack(void);					//MLX90393����ACK�ź�
void MLX90393_NAck(void);				//MLX90393������ACK�ź�						  
void vInMeasurementNormal(void) ;
unsigned char ucMLX90393WriteCommand(unsigned char ucWriteCmd, unsigned char ucSlaveAdd);
unsigned short usMlx90393ReadReg(unsigned char RegAddress, unsigned char SlaveAdd);
unsigned char ucMLX90393WriteReg(unsigned char ucRegAddress, unsigned short usWriteData, unsigned char ucSlaveAdd);
unsigned char ucCheckReadMlxStatusErro(unsigned char ucMlxStatusData, unsigned char ucCmdMode);
void vCmdMlxWriteRegAndWaite(uint16_t RegAddress, unsigned short usWriteData , unsigned char SlaveAdd, unsigned char ucErroType);
void vCmdMlxAndWaiteErroTimes(unsigned char command, unsigned char SlaveAdd, unsigned char ucErroType);
void vSetUpMlx90393(void);

int32_t Value_limit(int32_t min_value, int32_t current_value, int32_t max_value);
void initializeFilter(AverageFilter *filter);
int32_t filterValue_int32(AverageFilter *filter, int16_t input);
void mlx_90393_offsetcacu(void);
void mlx_90393_offset(void);
int32_t Value_Resetzero(int32_t min_value, int32_t current_value, int32_t max_value);

#endif

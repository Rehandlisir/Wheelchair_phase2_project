#ifndef __AS5013_H
#define __AS5013_H

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/FILTER/filter.h"

// AS5013�Ĵ�������
#define AS5013_I2C_ADDRESS (0x41 << 1)            //������ַ ����I2C��Ҫ������һλ оƬ����Դ��ַΪ0X41
#define AS5013_ID_CODE_REG (0x0c)                 //ID�洢�Ĵ���
#define AS5013_ID_VERSION_REG (0x0d)              //ID�汾�Ĵ���
#define AS5013_SILICONE_REVISION_REG (0x0e)		  //����汾�Ĵ���
/*�ؼ�����-��ȡ �Ĵ���*/
#define AS5013_CONTROL_REG_1 (0x0f)               //���ƼĴ���1
#define AS5013_X_REG (0x10)                       //X�Ĵ���
#define AS5013_Y_RESET_INT_REG (0x11)             //Y�Ĵ������Զ�����жϣ�
#define AS5013_X_LEFT_THRESHOLD_REG (0x12)        //X�������޼Ĵ���
#define AS5013_X_RIGHT_THRESHOLD_REG (0x13)       //X�����Ҽ��޼Ĵ���
#define AS5013_Y_TOP_THRESHOLD_REG (0x14) 				//Y�����ϼ��޼Ĵ���
#define AS5013_Y_BOTTOM_THRESHOLD_REG (0x15)			//Y�����¼��޼Ĵ���

#define AS5013_C4_NEG_H4_REG (0x16)			//C4�����Ĵ����������λ��Чֵ
#define AS5013_C4_NEG_L8_REG (0x17)			//C4�����Ĵ��������8λ��Чֵ
#define AS5013_C4_POS_H4_REG (0x18)			//C4�����Ĵ����������λ��Чֵ
#define AS5013_C4_POS_L8_REG (0x19)			//C4�����Ĵ��������8λ��Чֵ

#define AS5013_C3_NEG_H4_REG (0x1A)			//C3�����Ĵ����������λ��Чֵ
#define AS5013_C3_NEG_L8_REG (0x1B)			//C3�����Ĵ��������8λ��Чֵ
#define AS5013_C3_POS_H4_REG (0x1C)			//C3�����Ĵ����������λ��Чֵ
#define AS5013_C3_POS_L8_REG (0x1D)			//C3�����Ĵ��������8λ��Чֵ

#define AS5013_C2_NEG_H4_REG (0x1E)			//C2�����Ĵ����������λ��Чֵ
#define AS5013_C2_NEG_L8_REG (0x1F)			//C2�����Ĵ��������8λ��Чֵ
#define AS5013_C2_POS_H4_REG (0x20)			//C2�����Ĵ����������λ��Чֵ
#define AS5013_C2_POS_L8_REG (0x21)			//C2�����Ĵ��������8λ��Чֵ

#define AS5013_C1_NEG_H4_REG (0x22)			//C1�����Ĵ����������λ��Чֵ
#define AS5013_C1_NEG_L8_REG (0x23)			//C1�����Ĵ��������8λ��Чֵ
#define AS5013_C1_POS_H4_REG (0x24)			//C1�����Ĵ����������λ��Чֵ
#define AS5013_C1_POS_L8_REG (0x25)			//C1�����Ĵ��������8λ��Чֵ

#define AS5013_C5_NEG_H4_REG (0x26)			//C5�����Ĵ����������λ��Чֵ
#define AS5013_C5_NEG_L8_REG (0x27)			//C5�����Ĵ��������8λ��Чֵ
#define AS5013_C5_POS_H4_REG (0x28)			//C5�����Ĵ����������λ��Чֵ
#define AS5013_C5_POS_L8_REG (0x29)			//C5�����Ĵ��������8λ��Чֵ

#define AS5013_ACG_REG (0x2A)			//�Զ�������ƼĴ���

#define AS5013_M_CONTROL_REG (0x2b)      //���� M_ctrl�������м����Ԫ�� C5 �Ŀ��ƼĴ���������üĴ�����ֵΪ�㣬���ڽ��� XY �������ʱ����ʹ���м����Ԫ����
#define AS5013_J_CONTROL_REG (0x2c)      //���� J_ctrl��˥������-����������Ԫ������������˥�����ƼĴ�����
#define AS5013_SCALING_CONTROL_REG (0x2d)//���� T_ctrl���������Ӷ�����������ţ������� 8 λ����Ĵ�������
#define AS5013_CONTROL_REG_2 (0x2e)               //���ƼĴ���2-д��ֵ 86h����84h �� ��ת��������



// GPIO ���Ŷ���
/*
AS5013        STM32G431
-------------------
VDD  -> 3.3V
VDDp -> 3.3V
VSS  -> GND
SCL  -> PA15
SDA  -> PB7
INTn -> PA4
RST  -> PA8
*/

#define AS5031_SCL_PORT  GPIOA
#define AS5031_SCL_GPIO_PIN  GPIO_PIN_15
#define AS5031_SCL_GPIO_CLK_ENABLE()        \
    do                                \
    {                                 \
        __HAL_RCC_GPIOA_CLK_ENABLE(); \
    } while (0) /*  */
		
#define AS5031_SDA_PORT  GPIOB
#define AS5031_SDA_GPIO_PIN  GPIO_PIN_7
#define AS5031_SDA_GPIO_CLK_ENABLE()        \
    do                                \
    {                                 \
        __HAL_RCC_GPIOB_CLK_ENABLE(); \
    } while (0) /*  */
		
#define AS5031_INT_PORT  GPIOA
#define AS5031_INT_GPIO_PIN  GPIO_PIN_4
#define AS5031_INT_GPIO_CLK_ENABLE()        \
    do                                \
    {                                 \
        __HAL_RCC_GPIOA_CLK_ENABLE(); \
    } while (0) /*  */

		
#define AS5031_RST_PORT  GPIOA
#define AS5031_RST_GPIO_PIN  GPIO_PIN_8
#define AS5031_RST_GPIO_CLK_ENABLE()        \
    do                                \
    {                                 \
        __HAL_RCC_GPIOA_CLK_ENABLE(); \
    } while (0) /*  */
		
		
#define AS5031REST(x)                                                                                                                                \
    do                                                                                                                                         \
    {                                                                                                                                          \
        x ? HAL_GPIO_WritePin(AS5031_RST_PORT, AS5031_RST_GPIO_PIN, GPIO_PIN_SET) : HAL_GPIO_WritePin(AS5031_RST_PORT, AS5031_RST_GPIO_PIN, GPIO_PIN_RESET); \
    } while (0) /* x = 1 ����� ��֮�� */	

		
typedef struct {
    int8_t x_raw;       // ԭʼXֵ��8λ�з��ţ�
    int8_t y_raw;       // ԭʼYֵ��8λ�з��ţ�
		uint8_t As5013ID;
		
} AS5013_Data;

extern I2C_HandleTypeDef hi2c1;   // ȫ��I2C���
extern AS5013_Data As5013data;
// ��������
void AS5013_IIC_Init(void);
void AS5013_GPIO_Init(void);
void MX_GPIO_Init(void);

void ReadAs5013ID(uint8_t *ID);
void Read_XY(int8_t *x, int8_t *y);
void AS5013_Init(void) ;
void As5013_excute(void);

#endif 
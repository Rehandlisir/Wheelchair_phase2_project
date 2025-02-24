#ifndef __AS5013_H
#define __AS5013_H

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/FILTER/filter.h"

// AS5013寄存器定义
#define AS5013_I2C_ADDRESS (0x41 << 1)            //器件地址 由于I2C库要求右移一位 芯片本身源地址为0X41
#define AS5013_ID_CODE_REG (0x0c)                 //ID存储寄存器
#define AS5013_ID_VERSION_REG (0x0d)              //ID版本寄存器
#define AS5013_SILICONE_REVISION_REG (0x0e)		  //硅基版本寄存器
/*关键控制-读取 寄存器*/
#define AS5013_CONTROL_REG_1 (0x0f)               //控制寄存器1
#define AS5013_X_REG (0x10)                       //X寄存器
#define AS5013_Y_RESET_INT_REG (0x11)             //Y寄存器（自动清除中断）
#define AS5013_X_LEFT_THRESHOLD_REG (0x12)        //X死区左极限寄存器
#define AS5013_X_RIGHT_THRESHOLD_REG (0x13)       //X死区右极限寄存器
#define AS5013_Y_TOP_THRESHOLD_REG (0x14) 				//Y死区上极限寄存器
#define AS5013_Y_BOTTOM_THRESHOLD_REG (0x15)			//Y死区下极限寄存器

#define AS5013_C4_NEG_H4_REG (0x16)			//C4霍尔寄存器负向高四位有效值
#define AS5013_C4_NEG_L8_REG (0x17)			//C4霍尔寄存器负向低8位有效值
#define AS5013_C4_POS_H4_REG (0x18)			//C4霍尔寄存器正向高四位有效值
#define AS5013_C4_POS_L8_REG (0x19)			//C4霍尔寄存器正向低8位有效值

#define AS5013_C3_NEG_H4_REG (0x1A)			//C3霍尔寄存器负向高四位有效值
#define AS5013_C3_NEG_L8_REG (0x1B)			//C3霍尔寄存器负向低8位有效值
#define AS5013_C3_POS_H4_REG (0x1C)			//C3霍尔寄存器正向高四位有效值
#define AS5013_C3_POS_L8_REG (0x1D)			//C3霍尔寄存器正向低8位有效值

#define AS5013_C2_NEG_H4_REG (0x1E)			//C2霍尔寄存器负向高四位有效值
#define AS5013_C2_NEG_L8_REG (0x1F)			//C2霍尔寄存器负向低8位有效值
#define AS5013_C2_POS_H4_REG (0x20)			//C2霍尔寄存器正向高四位有效值
#define AS5013_C2_POS_L8_REG (0x21)			//C2霍尔寄存器正向低8位有效值

#define AS5013_C1_NEG_H4_REG (0x22)			//C1霍尔寄存器负向高四位有效值
#define AS5013_C1_NEG_L8_REG (0x23)			//C1霍尔寄存器负向低8位有效值
#define AS5013_C1_POS_H4_REG (0x24)			//C1霍尔寄存器正向高四位有效值
#define AS5013_C1_POS_L8_REG (0x25)			//C1霍尔寄存器正向低8位有效值

#define AS5013_C5_NEG_H4_REG (0x26)			//C5霍尔寄存器负向高四位有效值
#define AS5013_C5_NEG_L8_REG (0x27)			//C5霍尔寄存器负向低8位有效值
#define AS5013_C5_POS_H4_REG (0x28)			//C5霍尔寄存器正向高四位有效值
#define AS5013_C5_POS_L8_REG (0x29)			//C5霍尔寄存器正向低8位有效值

#define AS5013_ACG_REG (0x2A)			//自动增益控制寄存器

#define AS5013_M_CONTROL_REG (0x2b)      //配置 M_ctrl（用于中间霍尔元件 C5 的控制寄存器。如果该寄存器的值为零，则在进行 XY 坐标计算时不会使用中间霍尔元件）
#define AS5013_J_CONTROL_REG (0x2c)      //配置 J_ctrl（衰减因子-用于外侧霍尔元件基于扇区的衰减控制寄存器）
#define AS5013_SCALING_CONTROL_REG (0x2d)//配置 T_ctrl（缩放因子对输入进行缩放，以适配 8 位结果寄存器。）
#define AS5013_CONTROL_REG_2 (0x2e)               //控制寄存器2-写入值 86h或者84h → 反转磁铁极性



// GPIO 引脚定义
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
    } while (0) /* x = 1 输出高 反之低 */	

		
typedef struct {
    int8_t x_raw;       // 原始X值（8位有符号）
    int8_t y_raw;       // 原始Y值（8位有符号）
		uint8_t As5013ID;
		
} AS5013_Data;

extern I2C_HandleTypeDef hi2c1;   // 全局I2C句柄
extern AS5013_Data As5013data;
// 函数声明
void AS5013_IIC_Init(void);
void AS5013_GPIO_Init(void);
void MX_GPIO_Init(void);

void ReadAs5013ID(uint8_t *ID);
void Read_XY(int8_t *x, int8_t *y);
void AS5013_Init(void) ;
void As5013_excute(void);

#endif 
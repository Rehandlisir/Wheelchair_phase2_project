/**
 * @FilePath     : /MDK-ARMc:/Users/fu/Desktop/第二阶段新项目/总线式摇杆项目/JOYSTIC_HAL/Drivers/BSP/MLX90393_CAN/mlx90393can_send.c
 * @Description  :  CAN
 * @Author       : lisir lisir@rehand.com
 * @Version      : 0.0.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2024-11-11 20:31:44
 * @2024 by Rehand Medical Technology Co., LTD, All Rights Reserved.
**/

#include "./BSP/CAN/can.h"
#include "./BSP/MLX90393_CAN/mlx90393can_send.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/MLX90393/mlx90393.h"

void mlx_90393_CAN_Trans(uint8_t id,int16_t x,int16_t y)
{
  uint8_t send_data[8];
  send_data[0] = (uint8_t)(x&0xff);  //x 低八位
  send_data[1] = (uint8_t)((x >> 8)&0xff);//x 高八位
  send_data[2] = (uint8_t)(y & 0xff);  //x 低八位
  send_data[3] = (uint8_t)((y >> 8)&0xff);//x 高八位
  send_data[4] = id;
  send_data[5]= mlxdata.x_offset;
  send_data[6]= mlxdata.y_offset;
  FDCAN1_Send_Msg(id,send_data,FDCAN_DLC_BYTES_8); // id 0x0002

}
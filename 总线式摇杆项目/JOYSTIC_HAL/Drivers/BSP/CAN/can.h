/**
 ****************************************************************************************************
 * @file        can.h
 * @author      R9(V1.1)
 * @version     V1.1
 * @date        2023-06-06
 * @brief       CAN 驱动代码
 * @license    R9
 ****************************************************************************************************
 * @attention

 * 1, 优化can_send_msg函数, 新增发送超时处理机制
 ****************************************************************************************************
 */

#ifndef __CAN_H
#define __CAN_H

#include "./SYSTEM/sys/sys.h"

 
#ifdef __cplusplus
extern "C" {
#endif
 
extern FDCAN_HandleTypeDef hfdcan1;
uint8_t MX_FDCAN1_Init(void);
uint8_t FDCAN1_Send_Msg(uint32_t id,uint8_t* msg,uint32_t length);
uint8_t FDCAN1_Receive_Msg(uint8_t *buf, uint16_t *Identifier,uint16_t *len);
 
#ifdef __cplusplus
}
#endif
 
#endif 
 



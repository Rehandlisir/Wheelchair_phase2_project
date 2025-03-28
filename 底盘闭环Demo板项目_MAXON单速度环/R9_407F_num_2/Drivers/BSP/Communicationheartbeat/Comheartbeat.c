/*
 * @Author: lisir lisir@rehand.com
 * @Date: 2024-08-12 14:01:53
 * @LastEditors: lisir lisir@rehand.com
 * @LastEditTime: 2024-08-14 17:27:43
 * @FilePath: \R9_407F\R9_407F\R9_407_F\Drivers\BSP\Communicationheartbeat\Comheartbeat.c
 * @Description: 
 * 
 * Copyright (c) 2024-2034  , Rehand Medical Technology Co. LTDl, All Rights Reserved. 
 */
#include "./BSP/Communicationheartbeat/Comheartbeat.h"
STRUCT_COMHEART comheartstate;

void  ComheartReset(void)
{
        g_slaveReg[87] = 0;
  
}

/**
 * @brief        : 该函数 需在大于 200ms 定时器下执行一次，
 * @param         {uint16_t} overtcont:
 * @return        1：Success： 通讯成功   0:Fail通讯失败（连续 overtcont 次都为0 则判定上位机通讯失败）
**/
void ComheartDetect(uint16_t overtcont)
{
	static uint16_t comfali_cont;
	if (g_slaveReg[87] == 0) /*读取寄存器g_slaveReg[87]是否被上位机置1了，连续overtcont次未被置1 则判定上位机关机或断线通讯失败*/
    {
        comfali_cont++;
        if (comfali_cont > overtcont)
        {
            comfali_cont = 0;
            comheartstate.com_state = Fail;
            comheartstate.Current_Com = 0;
        }
    }
    else  /*上位机有心跳 通讯是OK的将g_slaveReg[87] 置0*/
    {
        
        comheartstate.com_state = Success;
        comheartstate.Current_Com = 1;
        comfali_cont = 0;
    }
}

/**
 * @FilePath     : /���̱ջ�Demo����Ŀ_MAXON���ٶȻ�/R9_407F_num_2/Drivers/BSP/API_Schedule.c
 * @Description  :  
 * @Author       : lisir
 * @Version      : V1.1
 * @LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime : 2025-01-06 13:59:53
 * @Copyright (c) 2024 by Rehand Medical Technology Co., LTD, All Rights Reserved. 
**/
#include "API_Schedule.h"
#include "./BSP/task.h"
/**
 * @description: �����ʼ����ÿһ���ṹ�嶼����һ��������������ɾ���������������
 * @return {*}
 */
struct TaskStruct TaskST[] =
	{
		{0, 1, 0, Task_GetMlx90393},
		{0, 3, 0, Task_UnderpanDrive},
		{0, 300, 0, Task_led_control},
		{0, 1, 0, Task_R9DataScope},
	};

// ��¼��������
uint8_t TaskCount = sizeof(TaskST) / sizeof(TaskST[0]);
/**
 * @description: ������ѯ
 * @return {*}
 */
void OS_IT_RUN(void)
{
	uint8_t i;
	for (i = 0; i < TaskCount; i++) // ��������ѭ��
	{
		if (!TaskST[i].TaskStatus)
		{
			if (++TaskST[i].TaskTickNow >= TaskST[i].TaskTickMax) // ��ʱ�����ж��Ƿ񵽴ﶨʱʱ��
			{
				TaskST[i].TaskTickNow = 0;
				TaskST[i].TaskStatus = 1;
			}
		}
	}
}
// ����main������ִ�У��Դ���ѭ��������ִ�й��������
void PeachOSRun(void)
{
	Hard_devInit();
	uint8_t j = 0;
	while (1)
	{
		iwdg_feed(); /* ι�� */
		if (TaskST[j].TaskStatus) // �ж�һ�������Ƿ񱻹���
		{
			TaskST[j].FC();			  // ִ�и�������
			TaskST[j].TaskStatus = 0; // ȡ������Ĺ���״̬
		}
		if (++j >= TaskCount) // �൱�ڲ���ѭ���������е�����
			j = 0;
	}
}

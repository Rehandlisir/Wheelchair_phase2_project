
#include "API_Schedule.h"
#include "./BSP/task.h"
/**
 * @description: �����ʼ����ÿһ���ṹ�嶼����һ��������������ɾ���������������
 * @return {*}
 */
struct TaskStruct TaskST[] =
{
		{0, 200, 0, TASK_led},
		{0, 1, 0, TASK_Actor_excute},
			

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

		if (TaskST[j].TaskStatus) // �ж�һ�������Ƿ񱻹���
		{
			TaskST[j].FC();			  // ִ�и�������
			TaskST[j].TaskStatus = 0; // ȡ������Ĺ���״̬
		}
		if (++j >= TaskCount) // �൱�ڲ���ѭ���������е�����
			j = 0;
	}
}

#ifndef __API_SCHEDULE_H__
#define __API_SCHEDULE_H__
#include "./SYSTEM/sys/sys.h"
/*
����洢һ���������е�����
*/
struct TaskStruct
{
	uint16_t	TaskTickNow;//���ڼ�ʱ
	uint16_t	TaskTickMax;//���ü�ʱʱ��
	uint8_t	TaskStatus;//�������б�־λ
	void (*FC)();//������ָ��
};

extern struct TaskStruct TaskST[];	//����Ϊ�ṹ�����ݣ������������ʱ�������

//�����������ĺ�������
void PeachOSRun(void);
void OS_IT_RUN(void);


#endif

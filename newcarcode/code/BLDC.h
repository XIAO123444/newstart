#ifndef _BLDC__H
#define _BLDC__H 

#include "zf_common_headfile.h"
// BLDC���������غ����ͺ궨��
typedef struct
{
    int16 basic_duty;   // ����ռ�ձ�
    int16 encoder_p; // ����������
    int16 max_output; // ������
    int16 min_output; // ��С���
    int16 output;     // ��ǰ���
} BLDC_Param;

void BLDC_init(void);
void BLDC_run(int16 duty);
void BLDC_unlock_UP(void);
void BLDC_unlock_DOWN(void);
void BLDC_STOP(void);

#endif  
#ifndef _BLDC__H
#define _BLDC__H 

#include "zf_common_headfile.h"
// BLDC电机控制相关函数和宏定义
void BLDC_init(void);
void BLDC_run(int16 duty);
void BLDC_unlock_UP(void);
void BLDC_unlock_DOWN(void);
void BLDC_STOP(void);

#endif  
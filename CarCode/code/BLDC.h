#ifndef _BLDC__H
#define _BLDC__H 

#include "zf_common_headfile.h"
// BLDC电机控制相关函数和宏定义
typedef struct
{
    int16 basic_duty;   // 基础占空比
    int16 encoder_p; // 编码器比例
    int16 max_output; // 最大输出
    int16 min_output; // 最小输出
    int16 output;     // 当前输出
} BLDC_Param;

void BLDC_init(void);
void BLDC_run(int16 duty);
void BLDC_unlock_UP(void);
void BLDC_unlock_DOWN(void);
void BLDC_STOP(void);

#endif  
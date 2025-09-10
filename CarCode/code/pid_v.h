#ifndef PID_V_H__
#define PID_V_H__
#include "zf_common_headfile.h"

struct pid_v {  // 完整定义
    float output_max;
    float p;
    float i;
    float d;
    float i_max;
    float d_max;
};

extern struct pid_v PID_V;
 
//int16 pid_control1(int16 target1);
//int16 pid_control2(int16 target2);
int pid_V_comon(int target);

void pidv_init(void);



// 参数增加函数
void pid_add_p(void);
void pid_add_i(void);
void pid_add_d(void);
void pid_add_i_max(void);
void pid_add_d_max(void);
void pid_add_output_max(void);

// 参数减少函数
void pid_sub_p(void);
void pid_sub_i(void);
void pid_sub_d(void);
void pid_sub_i_max(void);
void pid_sub_d_max(void);
void pid_sub_output_max(void);

void    pid_vparam_init(void);
struct  pid_v* PID_vget_param(void);
void    PID_vset_param(struct pid_v* p);




#endif
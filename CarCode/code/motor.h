#ifndef MOTOR_H__
#define MOTOR_H__

#include "zf_common_headfile.h"
#define PWM_L               (TIM5_PWM_CH4_A3)
#define PWM_R               (TIM5_PWM_CH2_A1)


#define DIR_R               (A0)
#define DIR_L               (A2)
void motor_init(void);
void motor_run(int16 a,int16 b);

#endif
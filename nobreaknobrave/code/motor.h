#ifndef MOTOR_H__
#define MOTOR_H__

#include "zf_common_headfile.h"
#define DIR_R               (A0 )
#define PWM_R               (TIM5_PWM_CH2_A1)

#define DIR_L               (A2 )
#define PWM_L               (TIM5_PWM_CH4_A3)
void motor_init(void);
void motor_run(int16 a,int16 b);


void motor(int16 left,int16 right);
#endif
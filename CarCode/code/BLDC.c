#include "BLDC.h"

#define MAX_DUTY            (100 )                                               // 最大占空比输出限制
#define PWM_CH1             (TIM2_PWM_CH1_A15)                                   // PWM1输出端口

#define PWM_CH2             (TIM2_PWM_CH2_B3)                                   // PWM2输出端口

void BLDC_init(void)
{
    // pwm_init(PWM_CH1, 100, 0);                                                 // PWM 通道1 初始化10ms 
    // pwm_set_duty(PWM_CH1, 1000);                     // 计算占空比
    pwm_init(PWM_CH1, 100, 0);                                                 // PWM 通道2 初始化10ms
    pwm_init(PWM_CH2, 100, 0);                                                 // PWM 通道2 初始化10ms

    // pwm_set_duty(PWM_CH1, 1000);                     // 计算占空比
    pwm_set_duty(PWM_CH1, 1000);                     // 计算占空比
    pwm_set_duty(PWM_CH2, 1000);                     // 计算占空比


}

void BLDC_run(int16 duty)
{
        if(0 <= duty)                                                               // 正转
        {
            pwm_set_duty(PWM_CH1, 1000+10*duty);                     // 计算占空比
            pwm_set_duty(PWM_CH2, 1000+10*duty);                     // 计算占空比
        }
        else                                                                        // 不转
        {
            pwm_set_duty(PWM_CH1, 1000);                     // 计算占空比
            pwm_set_duty(PWM_CH2, 1000);                     // 计算占空比
        }
}
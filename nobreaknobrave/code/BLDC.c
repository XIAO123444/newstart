#include "BLDC.h"

#define MAX_DUTY            (100 )                                               // 最大占空比输出限制
#define PWM_CH1             (TIM2_PWM_CH1_A15)                                   // PWM1输出端口

#define PWM_CH2             (TIM2_PWM_CH2_B3)                                   // PWM2输出端口
BLDC_Param bldc_param={
    .basic_duty=400,
    .encoder_p=5,
    .max_output=600,
    .min_output=-400,
};

void BLDC_init(void)
{
    pwm_init(PWM_CH1, 100, 0);                                                 // PWM 通道2 初始化10ms
    pwm_init(PWM_CH2, 100, 0);                                                 // PWM 通道2 初始化10ms

    pwm_set_duty(PWM_CH1, 0);                     // 计算占空比
    pwm_set_duty(PWM_CH2, 0);                     // 计算占空比


}
//电机校准高电平
void BLDC_unlock_UP(void)
{
    pwm_set_duty(PWM_CH1, 2000);                     // 计算占空比
    pwm_set_duty(PWM_CH2, 2000);                     // 计算占空比
}
void BLDC_unlock_DOWN(void)
{
    pwm_set_duty(PWM_CH1, 1000);                     // 计算占空比
    pwm_set_duty(PWM_CH2, 1000);                     // 计算占空比
}
void BLDC_STOP(void)
{
    pwm_set_duty(PWM_CH1, 0);                     // 计算占空比
    pwm_set_duty(PWM_CH2, 0);                     // 计算占空比
}
void BLDC_run(int16 duty)
{
        if(0 <= duty)                                                               // 正转
        {
            pwm_set_duty(PWM_CH1, 1000+duty);                     // 计算占空比
            pwm_set_duty(PWM_CH2, 1000+duty);                     // 计算占空比
        }
        else                                                                        // 不转
        {
            pwm_set_duty(PWM_CH1, 1000);                     // 计算占空比
            pwm_set_duty(PWM_CH2, 1000);                     // 计算占空比
        }
}

#include "BLDC.h"

#define MAX_DUTY            (100 )                                               // ���ռ�ձ��������
#define PWM_CH1             (TIM2_PWM_CH1_A15)                                   // PWM1����˿�

#define PWM_CH2             (TIM2_PWM_CH2_B3)                                   // PWM2����˿�

void BLDC_init(void)
{
    // pwm_init(PWM_CH1, 100, 0);                                                 // PWM ͨ��1 ��ʼ��10ms 
    // pwm_set_duty(PWM_CH1, 1000);                     // ����ռ�ձ�
    pwm_init(PWM_CH1, 100, 0);                                                 // PWM ͨ��2 ��ʼ��10ms
    pwm_init(PWM_CH2, 100, 0);                                                 // PWM ͨ��2 ��ʼ��10ms

    // pwm_set_duty(PWM_CH1, 1000);                     // ����ռ�ձ�
    pwm_set_duty(PWM_CH1, 1000);                     // ����ռ�ձ�
    pwm_set_duty(PWM_CH2, 1000);                     // ����ռ�ձ�


}

void BLDC_run(int16 duty)
{
        if(0 <= duty)                                                               // ��ת
        {
            pwm_set_duty(PWM_CH1, 1000+10*duty);                     // ����ռ�ձ�
            pwm_set_duty(PWM_CH2, 1000+10*duty);                     // ����ռ�ձ�
        }
        else                                                                        // ��ת
        {
            pwm_set_duty(PWM_CH1, 1000);                     // ����ռ�ձ�
            pwm_set_duty(PWM_CH2, 1000);                     // ����ռ�ձ�
        }
}
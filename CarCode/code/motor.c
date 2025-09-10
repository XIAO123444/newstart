#include "motor.h"

void motor_init()
{
    pwm_init(PWM_L, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    pwm_init(PWM_R, 17000, 0);                                                  // PWM ͨ����ʼ��Ƶ�� 17KHz ռ�ձȳ�ʼΪ 0
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL); // ���ַ������ų�ʼ��
    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL); // ���ַ������ų�ʼ��

}

void motor_run(int16 a,int16 b) 
{

    if(0 <= a)                                                           // ��ת
        {
            gpio_set_level(DIR_L, GPIO_HIGH); // �������ַ���Ϊ��ת
            pwm_set_duty(PWM_L, (uint16)(a * (PWM_DUTY_MAX / 10000)));                   // ����ռ�ձ�
                 // ����ռ�ձ� bS
        }
        else                                                                    // ��ת
        {

            gpio_set_level(DIR_L, GPIO_LOW); // �������ַ���Ϊ��ת
            pwm_set_duty(PWM_L, (uint16)(-a * (PWM_DUTY_MAX / 10000)));                   // ����ռ�ձ�
                // ����ռ�ձ�
        } 
 
     if(0 <= b)                                                           // ��ת
        {
            gpio_set_level(DIR_R, GPIO_LOW); // �������ַ���Ϊ��ת
            pwm_set_duty(PWM_R, (uint16)(b * (PWM_DUTY_MAX / 10000)));                   // ����ռ�ձ�
        }
        else// ��ת
        {
 
            gpio_set_level(DIR_R, GPIO_HIGH); // �������ַ���Ϊ��ת
            pwm_set_duty(PWM_R, (uint16)(-b * (PWM_DUTY_MAX / 10000)));                   // ����ռ�ձ�
        } 
}
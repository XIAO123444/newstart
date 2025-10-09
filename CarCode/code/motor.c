#include "motor.h"

void motor_init()
{
    pwm_init(PWM_L, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    pwm_init(PWM_R, 17000, 0);                                                  // PWM 通道初始化频率 17KHz 占空比初始为 0
    gpio_init(DIR_L, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 左轮方向引脚初始化
    gpio_init(DIR_R, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 右轮方向引脚初始化

}

void motor_run(int16 a,int16 b) 
{

    if(0 <= a)                                                           // 正转
        {
            gpio_set_level(DIR_L, GPIO_HIGH); // 设置左轮方向为正转
            pwm_set_duty(PWM_L, (uint16)(a * (PWM_DUTY_MAX / 10000)));                   // 计算占空比
                 // 计算占空比 bS
        }
        else                                                                    // 反转
        {

            gpio_set_level(DIR_L, GPIO_LOW); // 设置左轮方向为反转
            pwm_set_duty(PWM_L, (uint16)(-a * (PWM_DUTY_MAX / 10000)));                   // 计算占空比
                // 计算占空比
        } 
 
     if(0 <= b)                                                           // 正转
        {
            gpio_set_level(DIR_R, GPIO_LOW); // 设置右轮方向为正转
            pwm_set_duty(PWM_R, (uint16)(b * (PWM_DUTY_MAX / 10000)));                   // 计算占空比
        }
        else// 反转
        {
 
            gpio_set_level(DIR_R, GPIO_HIGH); // 设置右轮方向为反转
            pwm_set_duty(PWM_R, (uint16)(-b * (PWM_DUTY_MAX / 10000)));                   // 计算占空比
        } 
}
void Motor_SetLeftSpeed(int16 duty)
{		
	if (duty >= 0)							//如果设置正转的速度值
	{
		gpio_set_level(DIR_L, GPIO_LOW);
		pwm_set_duty(PWM_L, duty );                  
                              
		
	}
	else									//否则，即设置反转的速度值
	{  
		 gpio_set_level(DIR_L, GPIO_HIGH);
        pwm_set_duty(PWM_L, (-duty));
	}
}
void Motor_SetRightSpeed(int16 duty)
{
	if (duty >= 0)							//如果设置正转的速度值
	{
		gpio_set_level(DIR_R, GPIO_HIGH);
		pwm_set_duty(PWM_R, duty);                  // 计算占空比           
	}
	else									//否则，即设置反转的速度值
	{ 
		gpio_set_level(DIR_R, GPIO_LOW);
    pwm_set_duty(PWM_R, (-duty));               // 计算占空
}
	}

void motor(int16 left,int16 right){
	if (left>=PWM_DUTY_MAX){left=PWM_DUTY_MAX;}
	if ((-left)>=PWM_DUTY_MAX){left=-(PWM_DUTY_MAX);}
	if (right>=PWM_DUTY_MAX){right=PWM_DUTY_MAX;}
	if (-right>=PWM_DUTY_MAX){right=-(PWM_DUTY_MAX);}
	Motor_SetRightSpeed(-right);
	Motor_SetLeftSpeed(left);
}
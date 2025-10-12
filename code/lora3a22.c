#include "lora3a22.h"
#include "zf_common_headfile.h"
#include "motor.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "menu.h"
extern car_mode carmode;
// 控制参数结构体
typedef struct {
    int max_pwm;           // PWM上限
    int dead_zone;         // 摇杆死区
    float max_linear_speed;  // 最大线速度比例
    float max_angular_speed; // 最大角速度比例
    float acceleration_factor; // 加速度因子
    float deceleration_factor; // 减速度因子
} ControlParams;

// 控制器状态结构体
typedef struct {
    ControlParams params;
    float left_motor_speed;
    float right_motor_speed;
} TwoWheelController;

// 初始化控制器
void controller_init(TwoWheelController* controller, ControlParams* params) {
    controller->params = *params;
    controller->left_motor_speed = 0.0f;
    controller->right_motor_speed = 0.0f;
}

// 约束函数
float constrain(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// 绝对值函数
float fabs_f(float value) {
    return (value < 0) ? -value : value;
}

// 死区处理和归一化
float apply_deadzone_and_normalize(int stick_value, int dead_zone) {
    if (fabs_f((float)stick_value) < (float)dead_zone) {
        return 0.0f;
    }
    
    // 归一化到[-1, 1]
    float normalized = (float)stick_value / 2000.0f;
    
    // 限制在[-1, 1]范围内
    return constrain(normalized, -1.0f, 1.0f);
}

// 应用平滑曲线（平方根曲线，保持符号）
float apply_smooth_curve(float input) {
    // 使用平方根曲线，保持符号
    if (input >= 0) {
        return sqrtf(input);
    } else {
        return -sqrtf(-input);
    }
}

// 差速转向计算
void calculate_differential_drive(float linear, float angular, 
                                 float max_linear, float max_angular,
                                 float* left_speed, float* right_speed) {
    // 基础速度计算
    float base_left = linear * max_linear;
    float base_right = linear * max_linear;
    
    // 转向调整（差速）
    float turn_adjust = angular * max_angular;
    base_left -= turn_adjust;
    base_right += turn_adjust;
    
    // 限制在[-1, 1]范围内
    *left_speed = constrain(base_left, -1.0f, 1.0f);
    *right_speed = constrain(base_right, -1.0f, 1.0f);
    
    // 防止电机饱和
    float max_speed = fmaxf(fabs_f(*left_speed), fabs_f(*right_speed));
    if (max_speed > 1.0f) {
        *left_speed /= max_speed;
        *right_speed /= max_speed;
    }
}

// 平滑加速度控制
void smooth_acceleration(TwoWheelController* controller, 
                        float target_left, float target_right) {
    // 计算速度差
    float left_diff = target_left - controller->left_motor_speed;
    float right_diff = target_right - controller->right_motor_speed;
    
    // 选择加速度因子（加速或减速）
    float left_accel = (fabs_f(left_diff) > fabs_f(controller->left_motor_speed)) ? 
                      controller->params.acceleration_factor : 
                      controller->params.deceleration_factor;
    
    float right_accel = (fabs_f(right_diff) > fabs_f(controller->right_motor_speed)) ? 
                       controller->params.acceleration_factor : 
                       controller->params.deceleration_factor;
    
    // 应用平滑过渡
    controller->left_motor_speed += left_diff * left_accel;
    controller->right_motor_speed += right_diff * right_accel;
    
    // 接近零时直接归零，避免抖动
    if (fabs_f(controller->left_motor_speed) < 0.01f && fabs_f(target_left) < 0.05f) {
        controller->left_motor_speed = 0.0f;
    }
    if (fabs_f(controller->right_motor_speed) < 0.01f && fabs_f(target_right) < 0.05f) {
        controller->right_motor_speed = 0.0f;
    }
}

// 主控制函数
void update_control(TwoWheelController* controller, int left_stick_y, int right_stick_x) {
    // 1. 应用死区并归一化到[-1, 1]范围
    float linear = apply_deadzone_and_normalize(left_stick_y, controller->params.dead_zone);
    float angular = apply_deadzone_and_normalize(right_stick_x, controller->params.dead_zone);
    
    // 2. 应用非线性曲线让控制更丝滑
    linear = apply_smooth_curve(linear);
    angular = apply_smooth_curve(angular);
    
    // 3. 计算目标速度（差速转向模型）
    float target_left, target_right;
    calculate_differential_drive(linear, angular, 
                                controller->params.max_linear_speed,
                                controller->params.max_angular_speed,
                                &target_left, &target_right);
    
    // 4. 平滑加速度控制
    smooth_acceleration(controller, target_left, target_right);
}

// 获取电机PWM值
int get_left_motor_pwm(TwoWheelController* controller) {
    return (int)(controller->left_motor_speed * controller->params.max_pwm);
}

int get_right_motor_pwm(TwoWheelController* controller) {
    return (int)(controller->right_motor_speed * controller->params.max_pwm);
}
// 设置控制参数
void set_control_params(TwoWheelController* controller, ControlParams* params) {
    controller->params = *params;
}

// 默认参数配置
ControlParams default_params = {
    .max_pwm = 40000,
    .dead_zone = 300,
    .max_linear_speed = 0.8f,
    .max_angular_speed = 0.6f,
    .acceleration_factor = 0.05f,
    .deceleration_factor = 0.2f
};
void remote_speed_control(){
	TwoWheelController controller;
    // 初始化控制器
  controller_init(&controller, &default_params);
	int left_stick_y =0,right_stick_x =0;   //y:前后, x:左右
	if (lora3a22_state_flag == 1)
        {    
            if (lora3a22_finsh_flag == 1)
            {   
							  
								right_stick_x=lora3a22_uart_transfer.joystick[2];
							  left_stick_y=lora3a22_uart_transfer.joystick[1];
							  update_control(&controller, left_stick_y, right_stick_x);
							  int left_pwm = get_left_motor_pwm(&controller);
                int right_pwm = get_right_motor_pwm(&controller);
							  motor(left_pwm,right_pwm);
//							  printf("%d\n",left_pwm);
//							  printf("%d\n",right_pwm);
							if(lora3a22_uart_transfer.key[2]==1){
								carmode=stop;
							}
							  lora3a22_finsh_flag = 0;
						}
}
}
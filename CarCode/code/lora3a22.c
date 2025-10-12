#include "lora3a22.h"
#include "zf_common_headfile.h"
#include "motor.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "menu.h"
extern car_mode carmode;
// ���Ʋ����ṹ��
typedef struct {
    int max_pwm;           // PWM����
    int dead_zone;         // ҡ������
    float max_linear_speed;  // ������ٶȱ���
    float max_angular_speed; // �����ٶȱ���
    float acceleration_factor; // ���ٶ�����
    float deceleration_factor; // ���ٶ�����
} ControlParams;

// ������״̬�ṹ��
typedef struct {
    ControlParams params;
    float left_motor_speed;
    float right_motor_speed;
} TwoWheelController;

// ��ʼ��������
void controller_init(TwoWheelController* controller, ControlParams* params) {
    controller->params = *params;
    controller->left_motor_speed = 0.0f;
    controller->right_motor_speed = 0.0f;
}

// Լ������
float constrain(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// ����ֵ����
float fabs_f(float value) {
    return (value < 0) ? -value : value;
}

// ��������͹�һ��
float apply_deadzone_and_normalize(int stick_value, int dead_zone) {
    if (fabs_f((float)stick_value) < (float)dead_zone) {
        return 0.0f;
    }
    
    // ��һ����[-1, 1]
    float normalized = (float)stick_value / 2000.0f;
    
    // ������[-1, 1]��Χ��
    return constrain(normalized, -1.0f, 1.0f);
}

// Ӧ��ƽ�����ߣ�ƽ�������ߣ����ַ��ţ�
float apply_smooth_curve(float input) {
    // ʹ��ƽ�������ߣ����ַ���
    if (input >= 0) {
        return sqrtf(input);
    } else {
        return -sqrtf(-input);
    }
}

// ����ת�����
void calculate_differential_drive(float linear, float angular, 
                                 float max_linear, float max_angular,
                                 float* left_speed, float* right_speed) {
    // �����ٶȼ���
    float base_left = linear * max_linear;
    float base_right = linear * max_linear;
    
    // ת����������٣�
    float turn_adjust = angular * max_angular;
    base_left -= turn_adjust;
    base_right += turn_adjust;
    
    // ������[-1, 1]��Χ��
    *left_speed = constrain(base_left, -1.0f, 1.0f);
    *right_speed = constrain(base_right, -1.0f, 1.0f);
    
    // ��ֹ�������
    float max_speed = fmaxf(fabs_f(*left_speed), fabs_f(*right_speed));
    if (max_speed > 1.0f) {
        *left_speed /= max_speed;
        *right_speed /= max_speed;
    }
}

// ƽ�����ٶȿ���
void smooth_acceleration(TwoWheelController* controller, 
                        float target_left, float target_right) {
    // �����ٶȲ�
    float left_diff = target_left - controller->left_motor_speed;
    float right_diff = target_right - controller->right_motor_speed;
    
    // ѡ����ٶ����ӣ����ٻ���٣�
    float left_accel = (fabs_f(left_diff) > fabs_f(controller->left_motor_speed)) ? 
                      controller->params.acceleration_factor : 
                      controller->params.deceleration_factor;
    
    float right_accel = (fabs_f(right_diff) > fabs_f(controller->right_motor_speed)) ? 
                       controller->params.acceleration_factor : 
                       controller->params.deceleration_factor;
    
    // Ӧ��ƽ������
    controller->left_motor_speed += left_diff * left_accel;
    controller->right_motor_speed += right_diff * right_accel;
    
    // �ӽ���ʱֱ�ӹ��㣬���ⶶ��
    if (fabs_f(controller->left_motor_speed) < 0.01f && fabs_f(target_left) < 0.05f) {
        controller->left_motor_speed = 0.0f;
    }
    if (fabs_f(controller->right_motor_speed) < 0.01f && fabs_f(target_right) < 0.05f) {
        controller->right_motor_speed = 0.0f;
    }
}

// �����ƺ���
void update_control(TwoWheelController* controller, int left_stick_y, int right_stick_x) {
    // 1. Ӧ����������һ����[-1, 1]��Χ
    float linear = apply_deadzone_and_normalize(left_stick_y, controller->params.dead_zone);
    float angular = apply_deadzone_and_normalize(right_stick_x, controller->params.dead_zone);
    
    // 2. Ӧ�÷����������ÿ��Ƹ�˿��
    linear = apply_smooth_curve(linear);
    angular = apply_smooth_curve(angular);
    
    // 3. ����Ŀ���ٶȣ�����ת��ģ�ͣ�
    float target_left, target_right;
    calculate_differential_drive(linear, angular, 
                                controller->params.max_linear_speed,
                                controller->params.max_angular_speed,
                                &target_left, &target_right);
    
    // 4. ƽ�����ٶȿ���
    smooth_acceleration(controller, target_left, target_right);
}

// ��ȡ���PWMֵ
int get_left_motor_pwm(TwoWheelController* controller) {
    return (int)(controller->left_motor_speed * controller->params.max_pwm);
}

int get_right_motor_pwm(TwoWheelController* controller) {
    return (int)(controller->right_motor_speed * controller->params.max_pwm);
}
// ���ÿ��Ʋ���
void set_control_params(TwoWheelController* controller, ControlParams* params) {
    controller->params = *params;
}

// Ĭ�ϲ�������
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
    // ��ʼ��������
  controller_init(&controller, &default_params);
	int left_stick_y =0,right_stick_x =0;   //y:ǰ��, x:����
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
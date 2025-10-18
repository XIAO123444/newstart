#ifndef __PID_H
#define __PID_H

// 增量式PID控制器结构体
typedef struct{
  float out;         //PID输出
  float kp,ki,kd,kd2;     //PID参数，kd2为陀螺仪补偿参数
	float error0,error1,error2,errorint;    //当前误差，上次误差，上上次误差，误差积分
  float actual,targ;//实际值和目标值
	float maxout,minout;    //输出限幅
}PID_t;
void PID_calculate(int error,PID_t *p);     //基于误差输入的位置式PID计算函数
void increment_pid_update(PID_t *p);        //增量式PID控制器更新函数
void PID_gyro_update();                     //带陀螺仪补偿的位置式PID控制器
void PID_update(PID_t *p);                //位置式PID控制器更新函数
#endif
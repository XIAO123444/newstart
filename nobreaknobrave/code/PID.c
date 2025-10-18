#include "PID.h"

/**
 * @brief 位置式PID控制器更新函数
 * @param p PID控制器结构体指针
 * @note 位置式PID计算公式: Output = Kp*e(k) + Ki*∑e(k) + Kd*(e(k)-e(k-1))
 *        适合需要精确控制位置的系统[2](@ref)
 */
PID_t PID_gyro={
	  .kp=0.03,   //0.3
	  .ki=0.001,  //0.02//0.01
	  .kd=0,
	  .maxout=80,
	  .minout=0,
    .targ=0
};
PID_t PID_angle={
	  .kp=1,   //0.3
	  .ki=0.00,  //0.02//0.01
	  .kd=0,
	  .maxout=5000,
	  .minout=-5000,
    .targ=0
};
PID_t PID_speed={
	  .kp=2,   //0.3
	  .ki=0.009, //0.02//0.01
	  .kd=0,
	  .maxout=1500,
	  .minout=-1500,
	  .targ=0
};
PID_t PID_steer={
	  .kp=0,   //0.3
	  .ki=0, //0.02//0.01
	  .kd=0,
	  .kd2=0.1,
	  .maxout=5000,
	  .minout=-5000,
	  .targ=94,
};
//负压风扇环
PID_t PID_BLDC={
      .kp=40,   //0.3
      .ki=0.8, //0.02//0.01
      .kd=0,
      .kd2=15,
      .maxout=3000,
      .minout=-3000,
    .targ=0
};
void PID_update(PID_t *p) //位置式
{
    // 更新误差历史: 将上一次误差保存为e(k-1)，当前误差变为e(k)
    p->error1 = p->error0;
    // 计算当前误差e(k) = 目标值 - 实际值
    p->error0 = p->targ - p->actual;
    
    // 积分项累加：∑e(k) = ∑e(k-1) + e(k)
    p->errorint += p->error0;
    // 积分限幅：防止积分饱和，提高系统稳定性[2](@ref)
    if(p->errorint > 1000){p->errorint = 1000;}
    if(p->errorint < -1000){p->errorint = -1000;}
    
    // PID计算: P + I + D
    // P项: Kp*e(k), I项: Ki*∑e(k), D项: Kd*(e(k)-e(k-1))[9](@ref)
    p->out = p->kp * p->error0 + p->ki * p->errorint + p->kd * (p->error0 - p->error1);
    
    // 输出限幅：确保控制量在合理范围内
    if(p->out > p->maxout){p->out = p->maxout;}
    if(p->out < p->minout){p->out = p->minout;}
}

/**
 * @brief 增量式PID控制器更新函数
 * @param p PID控制器结构体指针
 * @note 增量式PID计算公式: Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
 *        输出变化量而不是绝对位置，适合执行机构带积分特性的对象[7](@ref)
 */
void increment_pid_update(PID_t *p) {
    // 更新误差历史: e(k-2) = e(k-1), e(k-1) = e(k), 然后计算新的e(k)
    p->error2 = p->error1;
    p->error1 = p->error0;
    p->error0 = p->targ - p->actual;
    
    // 积分项限幅（虽然增量式PID通常不直接使用积分项，但这里保留了积分限制）
    if(p->errorint > 10000){p->errorint = 10000;}
    if(p->errorint < -10000){p->errorint = -10000;}
    
    // 增量式PID计算: Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
    p->out += p->kp * (p->error0 - p->error1) + p->ki * p->error0 + p->kd * (p->error0 - 2 * p->error1 + p->error2);
    
    // 输出限幅
    if(p->out > p->maxout){p->out = p->maxout;}
    if(p->out < p->minout){p->out = p->minout;}
}

/**
 * @brief 基于误差输入的位置式PID计算函数
 * @param error 直接输入的误差值
 * @param p PID控制器结构体指针
 * @note 与PID_update()类似，但直接使用外部计算的误差值作为输入
 *        适用于误差已在外部计算好的场景
 */
void PID_calculate(int error, PID_t *p){
    // 更新误差历史
    p->error1 = p->error0;
    p->error0 = error; // 使用外部计算的误差值
    
    // 积分项累加与限幅
    p->errorint += p->error0;
    if(p->errorint > 1000){p->errorint = 1000;}
    if(p->errorint < -1000){p->errorint = -1000;}
    
    // 标准位置式PID计算
    p->out = p->kp * p->error0 + p->ki * p->errorint + p->kd * (p->error0 - p->error1);
    
    // 输出限幅
    if(p->out > p->maxout){p->out = p->maxout;}
    if(p->out < p->minout){p->out = p->minout;}
}

/**
 * @brief 带陀螺仪补偿的位置式PID控制器
 * @param p PID控制器结构体指针
 * @param gyro 陀螺仪角速度数据
 * @note 在标准PID基础上增加陀螺仪补偿项: Kd2 * gyro
 *        用于平衡控制系统（如平衡车），提高系统抗干扰能力[6](@ref)
 */
void PID_gyro_update(PID_t *p, int gyro){
    // 更新误差历史
    p->error1 = p->error0;
    p->error0 = p->targ - p->actual;
    
    // 积分项累加与限幅
    p->errorint += p->error0;
    if(p->errorint > 1000){p->errorint = 1000;}
    if(p->errorint < -1000){p->errorint = -1000;}
    
    // 带陀螺仪补偿的PID计算: 标准PID + Kd2 * 角速度
    p->out = p->kp * p->error0 + p->ki * p->errorint + p->kd * (p->error0 - p->error1) + p->kd2 * gyro;
    
    // 输出限幅
    if(p->out > p->maxout){p->out = p->maxout;}
    if(p->out < p->minout){p->out = p->minout;}
}
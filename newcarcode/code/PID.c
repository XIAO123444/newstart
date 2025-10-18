#include "PID.h"

/**
 * @brief λ��ʽPID���������º���
 * @param p PID�������ṹ��ָ��
 * @note λ��ʽPID���㹫ʽ: Output = Kp*e(k) + Ki*��e(k) + Kd*(e(k)-e(k-1))
 *        �ʺ���Ҫ��ȷ����λ�õ�ϵͳ[2](@ref)
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
//��ѹ���Ȼ�
PID_t PID_BLDC={
      .kp=40,   //0.3
      .ki=0.8, //0.02//0.01
      .kd=0,
      .kd2=15,
      .maxout=3000,
      .minout=-3000,
    .targ=0
};
void PID_update(PID_t *p) //λ��ʽ
{
    // ���������ʷ: ����һ������Ϊe(k-1)����ǰ����Ϊe(k)
    p->error1 = p->error0;
    // ���㵱ǰ���e(k) = Ŀ��ֵ - ʵ��ֵ
    p->error0 = p->targ - p->actual;
    
    // �������ۼӣ���e(k) = ��e(k-1) + e(k)
    p->errorint += p->error0;
    // �����޷�����ֹ���ֱ��ͣ����ϵͳ�ȶ���[2](@ref)
    if(p->errorint > 1000){p->errorint = 1000;}
    if(p->errorint < -1000){p->errorint = -1000;}
    
    // PID����: P + I + D
    // P��: Kp*e(k), I��: Ki*��e(k), D��: Kd*(e(k)-e(k-1))[9](@ref)
    p->out = p->kp * p->error0 + p->ki * p->errorint + p->kd * (p->error0 - p->error1);
    
    // ����޷���ȷ���������ں���Χ��
    if(p->out > p->maxout){p->out = p->maxout;}
    if(p->out < p->minout){p->out = p->minout;}
}

/**
 * @brief ����ʽPID���������º���
 * @param p PID�������ṹ��ָ��
 * @note ����ʽPID���㹫ʽ: ��u(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
 *        ����仯�������Ǿ���λ�ã��ʺ�ִ�л������������ԵĶ���[7](@ref)
 */
void increment_pid_update(PID_t *p) {
    // ���������ʷ: e(k-2) = e(k-1), e(k-1) = e(k), Ȼ������µ�e(k)
    p->error2 = p->error1;
    p->error1 = p->error0;
    p->error0 = p->targ - p->actual;
    
    // �������޷�����Ȼ����ʽPIDͨ����ֱ��ʹ�û���������ﱣ���˻������ƣ�
    if(p->errorint > 10000){p->errorint = 10000;}
    if(p->errorint < -10000){p->errorint = -10000;}
    
    // ����ʽPID����: ��u(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
    p->out += p->kp * (p->error0 - p->error1) + p->ki * p->error0 + p->kd * (p->error0 - 2 * p->error1 + p->error2);
    
    // ����޷�
    if(p->out > p->maxout){p->out = p->maxout;}
    if(p->out < p->minout){p->out = p->minout;}
}

/**
 * @brief ������������λ��ʽPID���㺯��
 * @param error ֱ����������ֵ
 * @param p PID�������ṹ��ָ��
 * @note ��PID_update()���ƣ���ֱ��ʹ���ⲿ��������ֵ��Ϊ����
 *        ��������������ⲿ����õĳ���
 */
void PID_calculate(int error, PID_t *p){
    // ���������ʷ
    p->error1 = p->error0;
    p->error0 = error; // ʹ���ⲿ��������ֵ
    
    // �������ۼ����޷�
    p->errorint += p->error0;
    if(p->errorint > 1000){p->errorint = 1000;}
    if(p->errorint < -1000){p->errorint = -1000;}
    
    // ��׼λ��ʽPID����
    p->out = p->kp * p->error0 + p->ki * p->errorint + p->kd * (p->error0 - p->error1);
    
    // ����޷�
    if(p->out > p->maxout){p->out = p->maxout;}
    if(p->out < p->minout){p->out = p->minout;}
}

/**
 * @brief �������ǲ�����λ��ʽPID������
 * @param p PID�������ṹ��ָ��
 * @param gyro �����ǽ��ٶ�����
 * @note �ڱ�׼PID���������������ǲ�����: Kd2 * gyro
 *        ����ƽ�����ϵͳ����ƽ�⳵�������ϵͳ����������[6](@ref)
 */
void PID_gyro_update(PID_t *p, int gyro){
    // ���������ʷ
    p->error1 = p->error0;
    p->error0 = p->targ - p->actual;
    
    // �������ۼ����޷�
    p->errorint += p->error0;
    if(p->errorint > 1000){p->errorint = 1000;}
    if(p->errorint < -1000){p->errorint = -1000;}
    
    // �������ǲ�����PID����: ��׼PID + Kd2 * ���ٶ�
    p->out = p->kp * p->error0 + p->ki * p->errorint + p->kd * (p->error0 - p->error1) + p->kd2 * gyro;
    
    // ����޷�
    if(p->out > p->maxout){p->out = p->maxout;}
    if(p->out < p->minout){p->out = p->minout;}
}
#ifndef __PID_H
#define __PID_H

// ����ʽPID�������ṹ��
typedef struct{
  float out;         //PID���
  float kp,ki,kd,kd2;     //PID������kd2Ϊ�����ǲ�������
	float error0,error1,error2,errorint;    //��ǰ���ϴ������ϴ���������
  float actual,targ;//ʵ��ֵ��Ŀ��ֵ
	float maxout,minout;    //����޷�
}PID_t;
void PID_calculate(int error,PID_t *p);     //������������λ��ʽPID���㺯��
void increment_pid_update(PID_t *p);        //����ʽPID���������º���
void PID_gyro_update();                     //�������ǲ�����λ��ʽPID������
void PID_update(PID_t *p);                //λ��ʽPID���������º���
#endif
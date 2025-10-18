#include "encoder.h"

int32 encoder_R;     //�ұ�����
int32 encoder_L;     //�������
int32 encoder_R_last=0; //�ұ������ϴ�ֵ
int32 encoder_L_last=0; //��������ϴ�ֵ
int32 encoder_R_d=0;    //�ұ�������
int32 encoder_L_d=0;    //���������

void Encoder_Init()
{
    encoder_quad_init(TIM3_ENCODER, TIM3_ENCODER_CH1_B4, TIM3_ENCODER_CH2_B5);  //������������ʼ��
	encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_B6, TIM4_ENCODER_CH2_B7);   //�����������ʼ��
 	pit_ms_init(TIM6_PIT, 1);                                                 //Ӳ����ʱ������1ms
	interrupt_set_priority(TIM6_IRQn, 0);                                           
   
}

int Encoder_GetInfo_L(void)
{
     return encoder_L;
}
int Encoder_GetInfo_R(void)
{
    return encoder_R;
}
#include "encoder.h"

int32 encoder_R;     //右编码器
int32 encoder_L;     //左编码器
int32 encoder_R_last=0; //右编码器上次值
int32 encoder_L_last=0; //左编码器上次值
int32 encoder_R_d=0;    //右编码器Δ
int32 encoder_L_d=0;    //左编码器Δ

void Encoder_Init()
{
    encoder_quad_init(TIM3_ENCODER, TIM3_ENCODER_CH1_B4, TIM3_ENCODER_CH2_B5);  //编码器正交初始化
	encoder_quad_init(TIM4_ENCODER, TIM4_ENCODER_CH1_B6, TIM4_ENCODER_CH2_B7);   //编码器方向初始化
 	pit_ms_init(TIM6_PIT, 1);                                                 //硬件定时，周期1ms
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
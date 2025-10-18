#include "flash.h"
#include "steer_pid.h"
#include "menu.h"
#include "pid.h"
#include "BLDC.h"



extern int speed;

extern int16 threshold_up;       //�����ֵ����
extern int16 threshold_down;     //�����ֵ����

extern int16 forwardsight; //Ĭ��ǰհ
extern int16 forwardsight2;//  ֱ���ж�ǰհ��������ע�������ǰհ��ͬ���������ֻ������ֳ����ٵģ�����
extern int16 forwardsight3;//���ǰհ

extern PID_t PID_gyro;          //���ٶȻ�
extern PID_t PID_angle;         //�ǶȻ�
extern PID_t PID_speed;         //�ٶȻ�  
extern PID_t PID_steer;         //ת��
extern PID_t PID_BLDC;          //��ѹ���Ȼ�

extern BLDC_Param bldc_param;
void flash_reset(void)
{
    flash_erase_page(100, 0);                                 // ������һҳ
    flash_erase_page(100, 1);                                 // ������һҳ
    flash_erase_page(100, 2);                                 // ������һҳ
    flash_erase_page(99, 0);                                  // ������һҳ
    
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ��������� FLASH����1
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     flash_save_config1();
// ��ע��Ϣ     0��default 1,2,3,4�ֱ��Ӧ��100,1����100,2����100,3����101,0��
//-------------------------------------------------------------------------------------------------------------------
void flash_save_config(int16 i)
{

        if(flash_check(100+i/4, i%4)){flash_erase_page(100+i/4, i%4);}
        flash_buffer_clear();
        //ֻ��ת��d2
    
        // 100,0   
        //���ٶȻ�
        flash_union_buffer[0].float_type = PID_gyro.kp;
        flash_union_buffer[1].float_type = PID_gyro.ki;    
        flash_union_buffer[2].float_type = PID_gyro.kd;
        flash_union_buffer[3].float_type = PID_gyro.maxout;    
        flash_union_buffer[4].float_type = PID_gyro.minout;
        //�ǶȻ�
        flash_union_buffer[5].float_type = PID_angle.kp;
        flash_union_buffer[6].float_type = PID_angle.ki;
        flash_union_buffer[7].float_type = PID_angle.kd;
        flash_union_buffer[8].float_type = PID_angle.maxout;
        flash_union_buffer[9].float_type = PID_angle.minout;
        //�ٶȻ�
        flash_union_buffer[10].float_type = PID_speed.kp;
        flash_union_buffer[11].float_type = PID_speed.ki;
        flash_union_buffer[12].float_type = PID_speed.kd;
        flash_union_buffer[13].float_type = PID_speed.maxout;
        flash_union_buffer[14].float_type = PID_speed.minout;
        flash_union_buffer[24].float_type=  PID_speed.targ;
        //ת��
        flash_union_buffer[15].float_type = PID_steer.kp;
        flash_union_buffer[16].float_type = PID_steer.ki;
        flash_union_buffer[17].float_type = PID_steer.kd;
        flash_union_buffer[18].float_type = PID_steer.kd2;
        flash_union_buffer[19].float_type = PID_steer.maxout;
        flash_union_buffer[20].float_type = PID_steer.minout;
        flash_union_buffer[25].float_type = PID_steer.kd2;
        //ת��6������
        flash_union_buffer[26].float_type = PID_BLDC.kp;
        flash_union_buffer[27].float_type = PID_BLDC.ki;    
        flash_union_buffer[28].float_type = PID_BLDC.kd;
        flash_union_buffer[29].float_type = PID_BLDC.maxout;
        flash_union_buffer[30].float_type = PID_BLDC.minout;
        flash_union_buffer[31].float_type = PID_BLDC.kd2;

        //��ѹ���Ȼ�5������
        //PID21������
        flash_union_buffer[21].int16_type = forwardsight;
        flash_union_buffer[22].int16_type = forwardsight2;
        flash_union_buffer[23].int16_type = forwardsight3;
        //ǰհ3������
        flash_union_buffer[24].int16_type = bldc_param.basic_duty;
        flash_union_buffer[25].int16_type = bldc_param.encoder_p;
        flash_union_buffer[26].int16_type = bldc_param.max_output;
        flash_union_buffer[27].int16_type = bldc_param.min_output;



        flash_write_page_from_buffer(100+i/4, i%4);        //flashд

}
void flash_save_config_default(void) { flash_save_config(0); }          //Ĭ�Ϸ�������
void flash_save_config_1(void) { flash_save_config(1); }
void flash_save_config_2(void) { flash_save_config(2); }
void flash_save_config_3(void) { flash_save_config(3); }
void flash_save_config_4(void) { flash_save_config(4); }
void flash_load_config(int16 i)
{
    flash_buffer_clear();                                               //����������
    flash_read_page_to_buffer(100+i/4, i%4);                            // �����ݴ� flash ��ȡ��������
    PID_gyro.kp = flash_union_buffer[0].float_type;
    PID_gyro.ki = flash_union_buffer[1].float_type;
    PID_gyro.kd = flash_union_buffer[2].float_type;
    PID_gyro.maxout = flash_union_buffer[3].float_type;
    PID_gyro.minout = flash_union_buffer[4].float_type;
    
    // �ǶȻ�������ȡ
    PID_angle.kp = flash_union_buffer[5].float_type;
    PID_angle.ki = flash_union_buffer[6].float_type;
    PID_angle.kd = flash_union_buffer[7].float_type;
    PID_angle.maxout = flash_union_buffer[8].float_type;
    PID_angle.minout = flash_union_buffer[9].float_type;
    
    // �ٶȻ�������ȡ
    PID_speed.kp = flash_union_buffer[10].float_type;
    PID_speed.ki = flash_union_buffer[11].float_type;
    PID_speed.kd = flash_union_buffer[12].float_type;
    PID_speed.maxout = flash_union_buffer[13].float_type;
    PID_speed.minout = flash_union_buffer[14].float_type;
    PID_speed.targ=flash_union_buffer[24].float_type;//24
    
    // ת�򻷲�����ȡ
    PID_steer.kp = flash_union_buffer[15].float_type;
    PID_steer.ki = flash_union_buffer[16].float_type;
    PID_steer.kd = flash_union_buffer[17].float_type;
    PID_steer.kd2 = flash_union_buffer[18].float_type; // ע�⣺����ʱ��kd2����ȡҲӦ��
    PID_steer.maxout = flash_union_buffer[19].float_type;
    PID_steer.minout = flash_union_buffer[20].float_type;
    PID_steer.kd2=flash_union_buffer[25].float_type;
    // ת��6��������ȡ
    PID_BLDC.kp = flash_union_buffer[26].float_type;
    PID_BLDC.ki = flash_union_buffer[27].float_type;
    PID_BLDC.kd = flash_union_buffer[28].float_type;
    PID_BLDC.maxout = flash_union_buffer[29].float_type;
    PID_BLDC.minout = flash_union_buffer[30].float_type;
    PID_BLDC.kd2 = flash_union_buffer[31].float_type;

    //PID21��������ȡ

    forwardsight=flash_union_buffer[21].int16_type;
    forwardsight2=flash_union_buffer[22].int16_type;
    forwardsight3=flash_union_buffer[23].int16_type;
    //ǰհ3��������ȡ
    bldc_param.basic_duty=flash_union_buffer[24].int16_type;
    bldc_param.encoder_p=flash_union_buffer[25].int16_type;
    bldc_param.max_output=flash_union_buffer[26].int16_type;
    bldc_param.min_output=flash_union_buffer[27].int16_type;        
    //bldc4��������ȡ


} 
void flash_load_config_default(void) { flash_load_config(0); }      //Ĭ�Ϸ�������
void flash_load_config_1(void) { flash_load_config(1); }
void flash_load_config_2(void) { flash_load_config(2); }
void flash_load_config_3(void) { flash_load_config(3); }
void flash_load_config_4(void) { flash_load_config(4); }
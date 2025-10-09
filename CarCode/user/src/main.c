#include "zf_common_headfile.h"
#include "BLDC.h"
#include "menu.h"
#include "encoder.h"
#include "key.h"
 #include "flash.h"  // ����
#include "motor.h"
#include "photo_chuli.h"
#include "screen.h"
#include "track.h"
#include "steer_pid.h"
#include "buzzer.h"
#include "speed.h"
bool stop_flag1;                    // ֹͣ��־
bool start_flag = false;            // ������־

extern uint8 leftline_num;          // ���ߵ���
extern uint8 rightline_num;         // ���ߵ���
extern int current_state;
extern int speed; 

extern int encodercounter1; //  ��̼���4
extern float filtering_angle; //������ĽǶ�
extern int16 extern_gy;      //�ⲿ����������

extern int image_threshold; 
extern int16 threshold_up;  //�����ֵ����
extern int16 threshold_down; //�����ֵ����
extern uint8 dis_image[MT9V03X_H][MT9V03X_W];
extern float filtering_angle; 
extern  int32 gyrocounter;                  //�����ǻ���

extern bool stop;
extern bool show_flag;
void all_init(void)
{
    clock_init(SYSTEM_CLOCK_120M);
    debug_init();
    imu660ra_init();          // IMU660RA ��ʼ��
    Encoder_Init();       // ��������ʼ��   TIM6�������ʼ��
    Menu_Screen_Init();         //��Ļ������ʾ��ʼ��        
    Key_init();                 //������ʼ��        TIm7�������ʼ��
    BUZZ_init();                // ��������ʼ��
    motor_init();               //�����ʼ��         
    BLDC_init();                // BLDC ��ʼ�� TIM2�����ﱻ����pwm���
    flash_load_config_default();

    while(1) // ����ͷ��ʼ��
    {
        if(mt9v03x_init())
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
        }
        else
        {
            break;
        }
        system_delay_ms(50);
    }
}


 
int main()
{
    all_init();
    while(1)
    {
        if(stop==true)
        {
            Key_Scan();             // ����ɨ��
            Menu_control();         // �˵�����
        }
        if(mt9v03x_finish_flag)
        { 
            photo_image_process_all();
            // protect(); // ����    
            // Velocity_Control();       // �ٶȿ���    
            if(current_state == 1)
            {
                if(stop&&show_flag)
                {
                    photo_displayimage();
                    show_line(); 
                }          
            }                                                                   

            mt9v03x_finish_flag = 0;
        } 
    }
}


// int main(void)
// { 
//     all_init();
//     while (1)
//     {
//         motor_run(1000,1000);//���Դ��룬�����10%���ٶ���ת
//         BLDC_run(10); //���Դ��룬�����10%��ռ�ձ���ת 
//     }
//     //ע���練ת��ļ��ԣ����ת�뷴������
// }
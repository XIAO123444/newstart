#include "zf_common_headfile.h"

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
    Encoder_Init();
    Menu_Screen_Init();         //��Ļ������ʾ��ʼ��        
    Key_init();                 //������ʼ��
    BUZZ_init();                // ��������ʼ��
    motor_init();               //�����ʼ��
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
        Key_Scan();             // ����ɨ��
        Menu_control();         // �˵�����
        if(mt9v03x_finish_flag)
        { 
            photo_image_process_all();
            protect(); // ����    
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
//     stop_flag1 = false;
//     while(1)
//     { 
//         if(start_flag == false)
//         {
//             Key_Scan();             // ����ɨ��
//             Menu_control();         // �˵�����
//             flash_save();           // flash�洢
//         }
//         BUZZ_cycle();           // ?    ��?��ѭ��
//         if(mt9v03x_finish_flag)
//         { 
//             image_threshold = my_adapt_threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H); // ͼ���ȡ
//             set_b_imagine(image_threshold);        // ��ֵ��
//             image_boundary_process2();              // ͼ��߽紦��
//             element_check();// Ԫ�ؼ��
//             Velocity_Control();       // �ٶȿ���    
//             if(current_state == 1)
//             {
//                 if(start_flag==false)
//                 {
//                     ips200_show_gray_image(0,120,(const uint8 *)dis_image,MT9V03X_W, MT9V03X_H,MT9V03X_W, MT9V03X_H,0);       //
//                     show_line(); 
//                 }          
//             }                                                                   
//             if(encodercounter1 > 70000)
//             {	
//                 banmaxian_check(); // �����߱���
//                 black_protect_check();  // ��ɫ����
//             }
//             if(stop_flag1)
//             {
//                 pit_disable(TIM6_PIT);  // ���ͣת  
//                 motor_run(0, 0);
//             }
//             mt9v03x_finish_flag = 0;
//         } 
//     }
// }
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
#include "Balance.h"
#include "zf_device_lora3a22.h"
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
extern car_mode carmode;  //����״̬
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
    // lora3a22_init();           // lora3a22 ��ʼ��
    // flash_load_config_default();
    lora3a22_init();
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


 
// int main()
// {
//     all_init();
//     while(1)
//     {
//         if(carmode!=car_run_mode1&&carmode!=car_run_mode2)
//         {
//             Key_Scan();             // ����ɨ��
//             Menu_control();         // �˵�����
//         }
//         if(mt9v03x_finish_flag)
//         { 
//             photo_image_process_all();
//             // protect(); // ����    
//             // Velocity_Control();       // �ٶȿ���    
//             if(current_state == 1)
//             {
//                 if(carmode!=car_run_mode1&&carmode!=car_run_mode2&&show_flag) 
//                 {
//                     photo_displayimage();
//                     show_line(); 
//                 }          
//             }                                                                   

//             mt9v03x_finish_flag = 0;
//         } 
//     }
// }



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

int main()
{
    all_init();
    while(1)
    {
        if (lora3a22_state_flag == 1)
        {
            if (lora3a22_finsh_flag == 1)
            {

				//lora3a22 ֡ͷ
				printf ("head = %d\r\n",lora3a22_uart_transfer.head);

				//lora3a22 ��У��
				printf ("sum_check = %d\r\n",lora3a22_uart_transfer.sum_check);

				//���ҡ������ֵ
				printf ("joystick[0] = %d\r\n",lora3a22_uart_transfer.joystick[0]);
				//���ҡ������ֵ
				printf ("joystick[1] = %d\r\n",lora3a22_uart_transfer.joystick[1]);
				//�ұ�ҡ������ֵ
				printf ("joystick[2] = %d\r\n",lora3a22_uart_transfer.joystick[2]);
				//�ұ�ҡ������ֵ
				printf ("joystick[3] = %d\r\n",lora3a22_uart_transfer.joystick[3]);

				//���ҡ�˰���
				printf ("key0 = %d\r\n",lora3a22_uart_transfer.key[0]);
				//�ұ�ҡ�˰���
				printf ("key1 = %d\r\n",lora3a22_uart_transfer.key[1]);
				// ��߲��򰴼�
				printf ("key2 = %d\r\n",lora3a22_uart_transfer.key[2]);
				// �ұ߲��򰴼�
				printf ("key3 = %d\r\n",lora3a22_uart_transfer.key[3]);

				// ��߲��뿪��_1
				printf ("switch_key[0] = %d\r\n",lora3a22_uart_transfer.switch_key[0]);
				// ��߲��뿪��_2
				printf ("switch_key[1] = %d\r\n",lora3a22_uart_transfer.switch_key[1]);
				// �ұ߲��뿪��_1
				printf ("switch_key[2] = %d\r\n",lora3a22_uart_transfer.switch_key[2]);
				// �ұ߲��뿪��_2
				printf ("switch_key[3] = %d\r\n",lora3a22_uart_transfer.switch_key[3]);
				
				
				lora3a22_finsh_flag = 0;
            }
        }
        else
        {
            printf("lora3a22 connection fail \r\n");
        }
    }
}
#include "zf_common_headfile.h"
#include "BLDC.h"
#include "menu.h"
#include "encoder.h"
#include "key.h"
 #include "flash.h"  // 闪存
#include "motor.h"
#include "photo_chuli.h"
#include "screen.h"
#include "track.h"
#include "steer_pid.h"
#include "buzzer.h"
#include "speed.h"
#include "Balance.h"
#include "zf_device_lora3a22.h"
extern uint8 leftline_num;          // 左线点数
extern uint8 rightline_num;         // 右线点数
extern int current_state;
extern int speed; 

extern int encodercounter1; //  里程计数4
extern float filtering_angle; //解算出的角度
extern int16 extern_gy;      //外部陀螺仪数据

extern int image_threshold; 
extern int16 threshold_up;  //大津法阈值上限
extern int16 threshold_down; //大津法阈值下限
extern uint8 dis_image[MT9V03X_H][MT9V03X_W];
extern float filtering_angle; 
extern  int32 gyrocounter;                  //陀螺仪积分
extern car_mode carmode;  //车的状态
extern bool show_flag;
void all_init(void)
{
    clock_init(SYSTEM_CLOCK_120M);
    debug_init();
    imu660ra_init();          // IMU660RA 初始化
    Encoder_Init();       // 编码器初始化   TIM6在这里初始化
    Menu_Screen_Init();         //屏幕基础显示初始化        
    Key_init();                 //按键初始化        TIm7在这里初始化
    BUZZ_init();                // 蜂鸣器初始化
    motor_init();               //电机初始化         
    BLDC_init();                // BLDC 初始化 TIM2在这里被用作pwm输出
    // lora3a22_init();           // lora3a22 初始化
    // flash_load_config_default();
    lora3a22_init();
    while(1) // 摄像头初始化
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
//             Key_Scan();             // 按键扫描
//             Menu_control();         // 菜单控制
//         }
//         if(mt9v03x_finish_flag)
//         { 
//             photo_image_process_all();
//             // protect(); // 保护    
//             // Velocity_Control();       // 速度控制    
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
//         motor_run(1000,1000);//测试代码，电机以10%的速度正转
//         BLDC_run(10); //测试代码，电机以10%的占空比正转 
//     }
//     //注：如反转请改极性，如疯转请反插驱动
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

				//lora3a22 帧头
				printf ("head = %d\r\n",lora3a22_uart_transfer.head);

				//lora3a22 和校验
				printf ("sum_check = %d\r\n",lora3a22_uart_transfer.sum_check);

				//左边摇杆左右值
				printf ("joystick[0] = %d\r\n",lora3a22_uart_transfer.joystick[0]);
				//左边摇杆上下值
				printf ("joystick[1] = %d\r\n",lora3a22_uart_transfer.joystick[1]);
				//右边摇杆左右值
				printf ("joystick[2] = %d\r\n",lora3a22_uart_transfer.joystick[2]);
				//右边摇杆上下值
				printf ("joystick[3] = %d\r\n",lora3a22_uart_transfer.joystick[3]);

				//左边摇杆按键
				printf ("key0 = %d\r\n",lora3a22_uart_transfer.key[0]);
				//右边摇杆按键
				printf ("key1 = %d\r\n",lora3a22_uart_transfer.key[1]);
				// 左边侧向按键
				printf ("key2 = %d\r\n",lora3a22_uart_transfer.key[2]);
				// 右边侧向按键
				printf ("key3 = %d\r\n",lora3a22_uart_transfer.key[3]);

				// 左边拨码开关_1
				printf ("switch_key[0] = %d\r\n",lora3a22_uart_transfer.switch_key[0]);
				// 左边拨码开关_2
				printf ("switch_key[1] = %d\r\n",lora3a22_uart_transfer.switch_key[1]);
				// 右边拨码开关_1
				printf ("switch_key[2] = %d\r\n",lora3a22_uart_transfer.switch_key[2]);
				// 右边拨码开关_2
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
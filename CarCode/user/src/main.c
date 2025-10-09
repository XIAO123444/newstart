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
bool stop_flag1;                    // 停止标志
bool start_flag = false;            // 启动标志

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

extern bool stop;
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
    flash_load_config_default();

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


 
int main()
{
    all_init();
    while(1)
    {
        if(stop==true)
        {
            Key_Scan();             // 按键扫描
            Menu_control();         // 菜单控制
        }
        if(mt9v03x_finish_flag)
        { 
            photo_image_process_all();
            // protect(); // 保护    
            // Velocity_Control();       // 速度控制    
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
//         motor_run(1000,1000);//测试代码，电机以10%的速度正转
//         BLDC_run(10); //测试代码，电机以10%的占空比正转 
//     }
//     //注：如反转请改极性，如疯转请反插驱动
// }
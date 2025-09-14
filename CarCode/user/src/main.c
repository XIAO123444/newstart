#include "zf_common_headfile.h"

#include "menu.h"
#include "encoder.h"
#include "key.h"
// #include "flash.h"  // 闪存
#include "motor.h"
#include "photo_chuli.h"
#include "screen.h"
#include "track.h"
#include "steer_pid.h"
#include "buzzer.h"
#include "speed.h"
bool save_flag = false;
bool stop_flag1;                    // 停止标志
bool start_flag = false;            // 启动标志

extern uint8 leftline_num;          // 左线点数
extern uint8 rightline_num;         // 右线点数
extern int current_state;
extern int speed; 
extern int forwardsight;
extern int forwardsight2; // 前瞻2  
extern int forwardsight3; // 前瞻3
extern int encodercounter1;
extern int image_threshold; 
extern int16 threshold_up;  //大津法阈值上限
extern int16 threshold_down; //大津法阈值下限
extern uint8 dis_image[MT9V03X_H][MT9V03X_W];

extern bool stop;
extern bool show_flag;
void all_init(void)
{
    clock_init(SYSTEM_CLOCK_120M);
    debug_init();
    imu660ra_init();            //陀螺仪初始化
    Encoder_Init();
    Menu_Screen_Init();         //屏幕基础显示初始化        
    // flash_init();               //闪存初始化
    Key_init();                 //按键初始化
    BUZZ_init();                // 蜂鸣器初始化
    motor_init();               //电机初始化

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


// int main(void)
// {
//     all_init();
//     stop_flag1 = false;
//     while(1)
//     { 
//         if(start_flag == false)
//         {
//             Key_Scan();             // 按键扫描
//             Menu_control();         // 菜单控制
//             flash_save();           // flash存储
//         }
//         BUZZ_cycle();           // ?    涿?器循环
//         if(mt9v03x_finish_flag)
//         { 
//             image_threshold = my_adapt_threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H); // 图像获取
//             set_b_imagine(image_threshold);        // 二值化
//             image_boundary_process2();              // 图像边界处理
//             element_check();// 元素检查
//             Velocity_Control();       // 速度控制    
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
//                 banmaxian_check(); // 斑马线保护
//                 black_protect_check();  // 黑色保护
//             }
//             if(stop_flag1)
//             {
//                 pit_disable(TIM6_PIT);  // 电机停转  
//                 motor_run(0, 0);
//             }
//             mt9v03x_finish_flag = 0;
//         } 
//     }
// }
void protect()
{
    if(encodercounter1 > 70000)
    {	
        banmaxian_check(); // 斑马线保护
        black_protect_check();  // 黑色保护
    }
}
int main()
{
    all_init();
    while(1)
    {
        Key_Scan();             // 按键扫描
        Menu_control();         // 菜单控制
        if(mt9v03x_finish_flag)
        { 
            image_threshold = my_adapt_threshold(mt9v03x_image[0], MT9V03X_W, MT9V03X_H); // 图像获取
            set_b_imagine(image_threshold);        // 二值化
            image_boundary_process2();              // 图像边界处理
            element_check();// 元素检查
            // Velocity_Control();       // 速度控制    
            if(current_state == 1)
            {
                if(stop&&show_flag)
                {
                    ips200_show_gray_image(0,140,(const uint8 *)dis_image,MT9V03X_W, MT9V03X_H,MT9V03X_W, MT9V03X_H,0);       //
                    show_line(); 
                }          
            }                                                                   

            mt9v03x_finish_flag = 0;
        } 
    }
}
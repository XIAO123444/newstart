#include "zf_common_headfile.h"

#include "menu.h"
#include "encoder.h"
#include "key.h"
#include "pid_v.h"
#include "flash.h"  // 闪存
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
extern struct pid_v PID_V;          // pid_V
extern struct steer_pid S_PID;
extern struct steer_pid S_PID1;
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

extern float beilv;
void all_init(void)
{
    clock_init(SYSTEM_CLOCK_120M);
    debug_init();
    Menu_Screen_Init();             
    flash_init();    
    Key_init();                     
    BUZZ_init();
    motor_init();
    
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

void flash_save(void)
{
    if(save_flag)
    {
        if(flash_check(100, 0)){flash_erase_page(100, 0);}
        flash_buffer_clear();
        
        // 100,0保存pid_v的数据
        flash_union_buffer[0].float_type = PID_V.p;
        flash_union_buffer[1].float_type = PID_V.i;    
        flash_union_buffer[2].float_type = PID_V.d;
        flash_union_buffer[3].float_type = PID_V.i_max;
        flash_union_buffer[4].float_type = PID_V.d_max;    
        flash_union_buffer[5].float_type = PID_V.output_max;
        
        flash_erase_page(100,0);
        flash_write_page_from_buffer(100,0);        // 将Flash扇区的页写入缓冲区数据

        // 100,1保存图像处理的数据
        
        if(flash_check(100, 1)){flash_erase_page(100, 1);}
        flash_buffer_clear();

        flash_union_buffer[0].float_type = S_PID.p;
        flash_union_buffer[1].float_type = S_PID.i;    
        flash_union_buffer[2].float_type = S_PID.d;
        flash_union_buffer[3].float_type = S_PID.outputmax;
        flash_union_buffer[4].float_type = S_PID.outputmin;
        
        flash_erase_page(100,1);
        flash_write_page_from_buffer(100,1);        // 将Flash扇区的页写入缓冲区数据

        if(flash_check(100, 2)){flash_erase_page(100, 2);}
        flash_buffer_clear();

        flash_union_buffer[0].int32_type = speed;
        flash_union_buffer[1].int32_type = forwardsight;
        flash_union_buffer[2].int32_type = forwardsight2;
        flash_union_buffer[3].int32_type = forwardsight3;
        flash_union_buffer[4].int32_type = threshold_up;
        flash_union_buffer[5].int32_type = threshold_down;
        
        flash_erase_page(100,2);
        flash_write_page_from_buffer(100,2);        // 将Flash扇区的页写入缓冲区数据

        if (flash_check(99, 0)){flash_erase_page(99, 0);}
        flash_buffer_clear();
        flash_union_buffer[0].float_type = S_PID1.p;
        flash_union_buffer[1].float_type = S_PID1.i;
        flash_union_buffer[2].float_type = S_PID1.d;
        flash_union_buffer[3].float_type = S_PID1.outputmax;
        flash_union_buffer[4].float_type = S_PID1.outputmin;
        flash_erase_page(99,0);
        flash_write_page_from_buffer(99,0);        
        save_flag = false;

        if (flash_check(99,1)){flash_erase_page(99,1);}
        flash_buffer_clear();
        flash_union_buffer[0].float_type = beilv;
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

int main()
{
    all_init();
    while(1)
    {
        Key_Scan();             // 按键扫描
        Menu_control();         // 菜单控制
    }
}
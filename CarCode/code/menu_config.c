#include "menu_config.h"
#include "key.h"
#include "photo_chuli.h"
#include "track.h"
#include "Beacon.h"
extern uint8 input;            // 输入状态
extern uint8 status;            // 状态变量
extern enum_menu_mode menu_Mode;         // 全局变量 菜单模式，默认为普通模式

extern MENU menu[];

// 快速显示函数
extern bool first_frame;

void show_element_new()
{
    ips200_clear();
    while((enum_Condition)input!=BACK)
    {   
        show_element();
    }
    ips200_clear();
    input=0;
    status=0;
    return;
}

void show_delta_line()
{
    ips200_clear();

    while((enum_Condition)input!=BACK)
    {   
        if(mt9v03x_finish_flag)
         { 
            photo_image_process_all();
            display_delta_line(20,60,20,110);
            
            photo_displayimage();
            show_line();
            mt9v03x_finish_flag = 0;
         } 
    }


    ips200_clear();
    input=0;
    status=0;
    return;
}


void show_jiao_point()
{
    ips200_clear();
    tubian_point_init();    //角点初始化
    debugger_jiaopoint_locate();    //角点显示

    while((enum_Condition)input!=BACK)
    {   
        if(mt9v03x_finish_flag)
         { 
            photo_image_process_all();
            check_point();          //角点检测
            debugger_jiaopoint_value();
            photo_displayimage();
            show_line();
            mt9v03x_finish_flag = 0;
         } 
    }

    ips200_clear();
    input=0;
    status=0;
}


/*--------------------------------------------------------------------------------------------------------------------
  @brief     Beacon功能函数
  @param     
  @return

--------------------------------------------------------------------------------------------------------------------*/
void Beacon_Function()
{
    ips200_clear();

    while(!((enum_Condition)input==BACK&&menu_Mode==normal))
    {   
        output();
        Menu_control();
        show_beacon_info();
    }

    ips200_clear();
    input=0;
    status=0;
}
#ifndef MENU_H__
#define MENU_H__


#include "zf_common_headfile.h"
typedef enum
{ straight,         //直道
    crossm,         //正入十字路口
    crossl,         //左斜入十字
    crossr,         //右斜入十字
    islandl,            //环岛左
    islandr,            //环岛右
    scurve,             //S弯
    curve,              //弯道
    speedup,            //加速带            
    ramp,               //坡道
    obstacle,           //障碍物
    blackprotect,       //黑线保护
    stall,              //堵转              
    zebra               //斑马线
    }  
enum_roadelementtypedef;
void output(void);
void car_init(void);
void Menu_control(void);
void Menu_Screen_Init(void);


#endif
#ifndef MENU_H__
#define MENU_H__


#include "zf_common_headfile.h"
//道路元素枚举
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
//菜单模式枚举
typedef enum
{   normal,
    edit_int,
    edit_float,
    edit_confirm,
    special_show_element1,          //流程图显示经过元素        
}enum_menu_mode;
//道路元素结构体（用于计数和开关功能）
typedef struct  
{
    int16 straight;     //直道
    int16 crossm;    //正入十字路口
    int16 crossl;   //左斜入十字
    int16 crossr;   //右斜入十字
    int16 islandl;       //环岛左
    int16 islandr;      //环岛右
    int16 scurve;      //S弯
    int16 curve;        //弯道
    int16 speedup;     //加速带
    int16 ramp;         //坡道
    int16 obstacle;     //障碍物
    int16 blackprotect; //黑线保护
    int16 stall;        //堵转
    int16 zebra;    //斑马线
}struct_roadelementypedef;
//菜单结构体{优先级，名字，坐标，浮点数据，整型数据，类型（枚举），默认执行函数}
typedef struct 
{
    unsigned char priority;             //页面优先级
    char str[20];                       //名字
    uint16 x;                           //显示横坐标
    uint16 y;                           //显示纵坐标
    float *value_f;                      //浮点数据
    int16 *value_i;                        //整型数据
    enum function
    {
        param_int,              //整型可编辑
        param_float,            //浮点可编辑
        confirm,                //确认
        catlog,                 //目录
        function,               //函数
        param_int_readonly,     //整型只读
        param_float_readonly,   //浮点只读
        on_off,                 //开关
        roadgothrough           //赛道元素通过情况
    } type; //类型:整型参数，浮点参数，目录， 
    void (*Operate_default)();          //默认执行函数

}MENU;

//菜单行为枚举
typedef enum {
    NOACTION,
    DOWN,
    UP,
    CONFIRM,
    BACK

}enum_Condition;  

void output(void);
void Menu_control(void);
void Menu_Screen_Init(void);


#endif
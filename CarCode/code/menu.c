 #include "menu.h"
#include "pid_v.h"
#include "encoder.h"
#include "key.h"
#include "steer_pid.h"



#define ips200_x_max 240
#define ips200_y_max 320
int current_state=1;
int p=0;//记录当前指针
int p_nearby=0;//记录所属的指针
uint8 input;
extern int status;
extern bool save_flag;      //保存标志位
extern bool start_flag;     //发车标志位
extern bool stop_flag1;     //停车标志位

bool show_flag=false;     //显示标志位,全局变量



float beilv;//倍率
//


//菜单调参
enum menu_mode//菜单模式
{   normal,
    edit_int,
    edit_float,
}menu_Mode=normal; 
int stepper_int[5]={1,5,10,20,50};                  //整型步进值
float stepper_float[5]={0.01,0.1,1.0,10.0,100.0};   //浮点型步进值
uint8 stepper_p_int=0;        //整型步进值指针
uint8 stepper_p_float=0;      //浮点型步进值指针

//防止空指针
int16 default_int=0;
float default_float=0.0;

//加减封装函数
//菜单函数
void show_image(void)
{
    show_flag=!show_flag; 
}
void add_intparam(int16* a)
{
    *a+=stepper_int[stepper_p_int];
}
void sub_intparam(int16* a)
{
    *a-=stepper_int[stepper_p_int];
}
void add_floatparam(float* a)
{
    *a+=stepper_float[stepper_p_float];
}
void sub_floatparam(float* a)
{
    *a-=stepper_float[stepper_p_float];
}
//加减封装函数

//pid结构体，可以
typedef struct 
{
    float p;
    float i;
    float d;
    float maxout;
}pidtest;
//结构体变量置0初始化
void pid_init(pidtest* pid)
{
    pid->p=0.0;
    pid->i=0.0;
    pid->d=0.0;
    pid->maxout=0.0;
}

//车驶过元素记录结构体
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
}roadelementypedef;
//注意注意注意
//斜入十字类型切换到正入十字不能算作两个元素。

enum roadelement_type
{   
    straight,
    crossm,
    crossl,
    crossr,
    islandl,
    islandr,
    scurve,
    curve,
    speedup,
    ramp,
    obstacle,
    blackprotect,
    stall,
    zebra
}roadelementType;//赛道元素类型

//菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量


int32 speed;
int16 forwardsight;
int16 forwardsight2;//直到判断前瞻
int16 forwardsight3;//弯道前瞻
//pid
pidtest pid_vtest=          {1.0,0.0,0.0,0.0};          //速度环测试
pidtest pid_steer_straight= {0.0,0.0,0.0,0.0};          //直道转向环测试
pidtest pid_steer_curve=    {0.0,0.0,0.0,0.0};          //弯道转向环测试

roadelementypedef roadelement_onoff={1,1,1,1,1,1,1,1,1,1,1,1,1,1};      //赛道元素功能开启关闭

roadelementypedef roadelement_record1={0,0,0,0,0,0,0,0,0,0,0,0,0,0};    //记录赛道元素
roadelementypedef roadelement_record2={0,0,0,0,0,0,0,0,0,0,0,0,0,0};    //记录赛道元素
roadelementypedef roadelement_record3={0,0,0,0,0,0,0,0,0,0,0,0,0,0};    //记录赛道元素
roadelementypedef roadelement_record4={0,0,0,0,0, 0,0,0,0,0,0,0,0,0};   //记录赛道元素


//大津法
int16 threshold_down=100;       //大津法阈值上限
int16 threshold_up=200;         //大津法阈值下限  
int16 OTSU_calperxpage=5;       //每x张图片计算一次大津法
//
//菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量

//菜单结构体
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
        param_int,
        param_float,
        catlog,
        function,
        param_int_readonly,
        param_float_readonly,
        on_off,
    } type; //类型:整型参数，浮点参数，目录， 
    void (*Operate_default)();          //默认执行函数

}MENU;
 

MENU menu[]=
{
    {1,"pidparam",0,20,&default_float,&default_int,catlog,NULL},
    //角速度环
    //角度环
    //速度环
        {2,"velocity_pid",0,20,&default_float,&default_int,catlog,NULL},
            {3,"p",      ips200_x_max-10 * 7, 20,  &pid_vtest.p, &default_int, param_float,      NULL},
            {3,"i",      ips200_x_max-10 * 7, 40,  &pid_vtest.i, &default_int, param_float,      NULL},
            {3,"d",      ips200_x_max-10 * 7, 60,  &pid_vtest.d, &default_int, param_float,      NULL},
            {3,"maxout", ips200_x_max-10 * 7, 80,  &pid_vtest.maxout, &default_int, param_float, NULL},
        {2,"steer_pid",0,40,&default_float,&default_int,catlog,NULL},
            {3,"p_s", ips200_x_max-10 * 7, 20,  &pid_steer_straight.p, &default_int, param_float, NULL},
            {3,"i_s", ips200_x_max-10 * 7, 40,  &pid_steer_straight.i, &default_int, param_float, NULL},
            {3,"d_s", ips200_x_max-10 * 7, 60,  &pid_steer_straight.d, &default_int, param_float, NULL},
            {3,"max_s", ips200_x_max-10 * 7, 80,  &pid_steer_straight.maxout, &default_int, param_float, NULL},
            {3,"p_c", ips200_x_max-10 * 7, 100,  &pid_steer_curve.p, &default_int, param_float, NULL},
            {3,"i_c", ips200_x_max-10 * 7, 120,  &pid_steer_curve.i, &default_int, param_float, NULL},
            {3,"d_c", ips200_x_max-10 * 7, 140,  &pid_steer_curve.d, &default_int, param_float, NULL},
            {3,"maxo", ips200_x_max-10 * 7, 160,  &pid_steer_curve.maxout, &default_int, param_float, NULL},
        {2,"vpid_set0",0,60,&default_float,&default_int,catlog,NULL},
        {2,"spid_set0",0,60,&default_float,&default_int,catlog,NULL},


    {1,"image",0,40,&default_float,&default_int,catlog,NULL},
        {2,"show_image",0,20,&default_float,&default_int,function,NULL},
        {2,"OTSU_threshold",0,40,&default_float,&default_int,catlog,NULL},
            {3,"OTSU_up",100,20,&default_float,&default_int    ,param_int,NULL},  
            {3,"OTSU_DOWN",100,40,&default_float,&default_int,param_int,NULL},

        {2,"image_point",0,60,&default_float,&default_int,catlog,NULL},
            {3,"crossroadall",0,20,&default_float,&default_int,catlog,NULL},       //十字大类
                {4,"r_up_p",100,20,&default_float,&default_int,param_int_readonly,NULL},
                {4,"r_down_p",100,40,&default_float,&default_int,param_int_readonly,NULL},
                {4,"l_up_p",100,60,&default_float,&default_int,param_int_readonly,NULL},
                {4,"l_down_p",100,80,&default_float,&default_int,param_int_readonly,NULL},
            {3,"round",0,40,&default_float,&default_int,catlog,NULL},           //圆环大类
        {2,"forwardsight",0,80,&default_float,&forwardsight,catlog,NULL},
            {3,"forwardsight2",100,20,&default_float,&forwardsight2,param_int,NULL},
            {3,"forwardsight3",100,40,&default_float,&forwardsight3,param_int,NULL},
    {1,"element_onoff",0,60,&default_float,&default_int,catlog,NULL},//赛道元素功能开启关闭
        {2,"crossl",100,20,&default_float,&roadelement_onoff.crossl,on_off,NULL},
        {2,"crossr",100,40,&default_float,&roadelement_onoff.crossr,on_off,NULL},
        {2,"crossm",100,60,&default_float,&roadelement_onoff.crossm,on_off,NULL},
        {2,"islandl",100,80,&default_float,&roadelement_onoff.islandl,on_off,NULL},
        {2,"islandr",100,100,&default_float,&roadelement_onoff.islandr,on_off,NULL},
        {2,"scurve",100,120,&default_float,&roadelement_onoff.scurve,on_off,NULL},
        {2,"speedup",100,140,&default_float,&roadelement_onoff.speedup,on_off,NULL},
        {2,"ramp",100,160,&default_float,&roadelement_onoff.ramp,on_off,NULL},
        {2,"obstacle",100,180,&default_float,&roadelement_onoff.obstacle,on_off,NULL},
    {1,"element_gothrough",0,80,&default_float,&default_int,catlog,NULL},//赛道元素记录

    {1,"flash",0,100,&default_float,&default_int,catlog,NULL},
    {1,"setting",0,120,&default_float,&default_int,catlog,NULL},
    {1,"end",0,0,&default_float,&default_int,catlog,NULL}//不可删去

};

enum condition{
    NOACTION,
    DOWN,
    UP,
    CONFIRM,
    BACK

}condition;  
/*
------------------------------------------------------------------------------------------------------------------
函数简介    初始化屏幕 
参数说明     无
返回参数     无
使用示例     直接调用
备注信息     无
-------------------------------------------------------------------------------------------------------------------
*/
void Menu_Screen_Init(void)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为白底黑字
    ips200_init(IPS200_TYPE_SPI);    //设置通信模式为SPI通信
}

/*
------------------------------------------------------------------------------------------------------------------
函数简介     屏幕显示
参数说明     无
返回参数     无
使用示例     直接调用
备注信息     无
-------------------------------------------------------------------------------------------------------------------
*/
void output(void) 
{
    int target_priority=current_state-1;
    if(menu_Mode==edit_int)
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);    //设置为棕色底黑字
        ips200_show_string(100,0,"len_i");
        ips200_show_int(160,0,stepper_int[stepper_p_int],3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为棕色底黑字

    }
    if(menu_Mode==edit_float)
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);    //设置为棕色底黑字
        ips200_show_string(100,0,"len_f");
        ips200_show_float(160,0,stepper_float[stepper_p_float],3,3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为棕色底黑字

    }

    if (target_priority==0)//顶级菜单
    {
    ips200_set_color(RGB565_DustyBlue, RGB565_BLACK);    //设置为蓝色黑底
    ips200_show_string(0,0,"menu");//输出标题字符
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为白色黑底
        for(int i=0;strcmp(menu[i].str, "end") != 0;i++)
        {
            if(menu[i].priority==1)
            {
                if(i==p)
                {
                    if(menu_Mode==normal)
                    {
                        ips200_show_string(0,menu[i].y,"->");//输出指向字符
                        ips200_show_string(20,menu[i].y,menu[i].str);
                    }
                    else if(menu_Mode==edit_int||menu_Mode==edit_float)
                    {
                        ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);    //设置为红色黑底
                        ips200_show_string(0,menu[i].y,"->");//输出指向字符
                        ips200_show_string(20,menu[i].y,menu[i].str);
                        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为棕色底黑字
                    }


                }
                else
                {
                    ips200_show_string(20,menu[i].y,menu[i].str);
                }
             }
        }
    }
    else if(target_priority!=0)//非顶级菜单
    {
    ips200_set_color(RGB565_DustyBlue, RGB565_BLACK);    //设置为绿色黑底
    ips200_show_string(0,0,menu[p_nearby].str);//输出上级字符
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为绿色黑底

        for(int i=p_nearby+1;menu[i].priority!=target_priority;i++)
        {
            if(menu[i].priority==current_state)
            {
                if(i==p)
                {
                    if(menu_Mode==normal)
                    {
                        ips200_show_string(0,menu[i].y,"->");//输出指向字符
                        ips200_show_string(20,menu[i].y,menu[i].str);
                    }
                    else if(menu_Mode==edit_int||menu_Mode==edit_float)
                    {
                        ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);    //设置为红色黑底
                        ips200_show_string(0,menu[i].y,"->");//输出指向字符
                        ips200_show_string(20,menu[i].y,menu[i].str);
                        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为黑底白字
                    }
                    // else if( menu_Mode==on_off)
                    // {
                    //     ips200_set_color(RGB565_BLUE, RGB565_BLACK);    //设置为红色黑底
                    //     ips200_show_string(0,menu[i].y,"->");//输出指向字符
                    //     ips200_show_string(20,menu[i].y,menu[i].str);
                    //     ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为黑底白字

                    // } 
                    
                    if(menu[i].type==param_float||menu[i].type==param_float_readonly)
                    {
                       ips200_show_float(menu[i].x,menu[i].y,*menu[i].value_f,3,3);
                    }
                    else if(menu[i].type==param_int||menu[i].type==param_int_readonly)
                    {
                       ips200_show_int(menu[i].x,menu[i].y,*menu[i].value_i,5);
                    }
                    else if(menu[i].type==on_off)
                    {
                        ips200_set_color(RGB565_ORANGE, RGB565_BLACK);    //设置为红色黑底
                        if (*menu[i].value_i)
                        {
                            ips200_show_string(menu[i].x,menu[i].y,"ON");

                        }
                        else
                        {
                            ips200_show_string(menu[i].x,menu[i].y,"OFF");
                        }
                        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为黑底白字

                    }

                }
                else
                {
                    ips200_show_string(20,menu[i].y,menu[i].str);
                    if(menu[i].type==param_float||menu[i].type==param_float_readonly)
                    {
                       ips200_show_float(menu[i].x,menu[i].y,*menu[i].value_f,3,3);
                    }
                    else if(menu[i].type==param_int||menu[i].type==param_int_readonly)
                    {
                       ips200_show_int(menu[i].x,menu[i].y,*menu[i].value_i,5);
                    }
                    else if(menu[i].type==on_off)
                    {
                        if (*menu[i].value_i)
                        {
                            ips200_show_string(menu[i].x,menu[i].y,"ON");

                        }
                        else
                        {
                            ips200_show_string(menu[i].x,menu[i].y,"OFF");
                        }
                        
                    }

                }
            } 
        }
    }

}

/*
------------------------------------------------------------------------------------------------------------------
函数简介     菜单控制
参数说明     无
返回参数     无
使用示例     直接调用
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
void Menu_control(void)
{
        output();
        status=0;
        condition = (enum condition)input; 
        if(input)
        {
            ips200_clear();
        }
        switch (condition)
        {
        case NOACTION:
                break;
            
        case DOWN:
            if(menu_Mode==edit_int)  //整型编辑模式下
            {
                *menu[p].value_i-=stepper_int[stepper_p_int];
                break;
            }
            if(menu_Mode==edit_float)
            { 
                *menu[p].value_f-=stepper_float[stepper_p_float];
                break;
            }
            if (strcmp(menu[p].str, "end") != 0&&menu[p+1].priority>=menu[p].priority)
            {
                int temp=menu[p].priority;
                uint8 old_p=p;
                p++;
                while(menu[p].priority!=temp && strcmp(menu[p+1].str, "end") != 0)
                {
                    p++;
                    if(menu[p].priority<temp)
                    {
                        p=old_p;    //回到原位置
                        ips200_show_string(0,180,"endorstart");

                        break;
                    }
                } 
                if(strcmp(menu[p].str,"end")==0)
                {
                    p=old_p;    //回到原位置
                    ips200_show_string(0,180,"endorstart");

                    break;
                }
            }
            else
            {
                ips200_show_string(0,180,"endorstart");
            }
            break;
        case UP:
            if(menu_Mode==edit_int)
            {
                *menu[p].value_i+=stepper_int[stepper_p_int];
                break;
            }
            if(menu_Mode==edit_float)
            { 
                *menu[p].value_f+=stepper_float[stepper_p_float];
                break;
            }
            if(p!=0&&menu[p-1].priority>=menu[p].priority)
            {
                int temp=menu[p].priority;
                p--;
                while (menu[p].priority!=temp){p--;}
            }
            else
            {
                ips200_show_string(0,180,"endorstart");
            }
             

            break;
        case CONFIRM:
            if(menu[p+1].priority==current_state+1&&strcmp(menu[p+1].str,"end")!=0&&menu[p].type==catlog)//子目录情况
            {
                current_state++;
                p_nearby=p;
                p++;
                break;
             }
            if(menu_Mode==edit_int)
            {
                stepper_p_int=(stepper_p_int+1)%5;
                break;

            }
            if(menu_Mode==edit_float)
            {
                stepper_p_float=(stepper_p_float+1)%5;
                break;
            }
            if(menu[p].type==param_float)             //参数情况                      
            {
                menu_Mode=edit_float;
                break;
                //进入浮点数编辑模式

            }
            if(menu[p].type==param_int)
            {
                menu_Mode=edit_int;
                break;
                //进入整型编辑模式
            }
            if(menu[p].type==on_off)
            {
                *menu[p].value_i=1-*menu[p].value_i;
            }
            if(menu[p].type==function)
            {
                menu[p].Operate_default();
            }
            if(menu[p].type==param_float_readonly||menu[p].type==param_int_readonly)
            {
                ips200_show_string(0,180,"error_readonly");
            }
         

            break;
        case BACK:
        if(menu_Mode!=normal) //编辑模式下按返回键退出编辑模式
        {
            menu_Mode=normal;
            break;
        }
        if(menu[p].priority!=1)
        {
            save_flag=true;
            current_state--;
            p=p_nearby;
            while (menu[p_nearby].priority!=current_state-1)
            {
                p_nearby--;
            }
        }
        else
        {
            ips200_show_string(0,180,"error");
        }

        default:
            break;
        }
        input=0;
        
}
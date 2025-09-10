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



bool showline;
int16 threshold_up;  //大津法阈值上限
int16 threshold_down; //大津法阈值下限

int32 speed;
int32 forwardsight;
int32 forwardsight2;//直到判断前瞻
int32 forwardsight3;//弯道前瞻

float beilv;//倍率
//


//菜单调参
enum menu_mode
{   normal,
    edit_int,
    edit_float
}menu_Mode=normal; //菜单模式
int stepper_int[5]={1,5,10,20,50};                  //整型步进值
float stepper_float[5]={0.01,0.1,1.0,10.0,100.0};   //浮点型步进值
uint8 stepper_p_int=0;        //整型步进值指针
uint8 stepper_p_float=0;      //浮点型步进值指针

//防止空指针
int default_int=0;
float default_float=0.0;

//加减封装函数
void add_intparam(int* a)
{
    *a+=stepper_int[stepper_p_int];
}
void sub_intparam(int* a)
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


typedef struct 
{
    float p;
    float i;
    float d;
    float maxout;

}pidtest;

pidtest pid_vtest=          {1.0,0.0,0.0,0.0};
pidtest pid_steer_straight= {0.0,0.0,0.0,0.0};
pidtest pid_steer_curve=    {0.0,0.0,0.0,0.0};
typedef struct 
{
    unsigned char priority;             //页面优先级
    char str[20];                       //名字
    uint16 x;                           //显示横坐标
    uint16 y;                           //显示纵坐标
    float *value_f;                      //浮点数据
    int *value_i;                        //整型数据
    enum function{param_int,param_float,catlog,function,param_int_readonly,param_float_readonly} type; //类型:整型参数，浮点参数，目录， 
    void (*Operate_default)();          //默认执行函数

}MENU;
 

MENU menu[]=
{
    {1,"pidparam",0,20,&default_float,&default_int,catlog,NULL},
        {2,"velocity_pid",0,20,&default_float,&default_int,catlog,NULL},
            {3,"p",      ips200_x_max-10 * 7, 20,  &pid_vtest.p, &default_int, param_float, NULL},
            {3,"i",      ips200_x_max-10 * 7, 40,  &pid_vtest.i, &default_int, param_float, NULL},
            {3,"d",      ips200_x_max-10 * 7, 60,  &pid_vtest.d, &default_int, param_float, NULL},
            {3,"maxout", ips200_x_max-10 * 7, 80,  &pid_vtest.maxout, &default_int, param_float, NULL},
        {2,"steer_pid",0,40,&default_float,&default_int,catlog,NULL},
            {3,"p_straight", ips200_x_max-10 * 7, 20,  &pid_steer_straight.p, &default_int, param_float, NULL},
            {3,"i_straight", ips200_x_max-10 * 7, 40,  &pid_steer_straight.i, &default_int, param_float, NULL},
            {3,"d_straight", ips200_x_max-10 * 7, 60,  &pid_steer_straight.d, &default_int, param_float, NULL},
            {3,"maxout_straight", ips200_x_max-10 * 7, 80,  &pid_steer_straight.maxout, &default_int, param_float, NULL},
            {3,"p_curve", ips200_x_max-10 * 7, 100,  &pid_steer_curve.p, &default_int, param_float, NULL},
            {3,"i_curve", ips200_x_max-10 * 7, 120,  &pid_steer_curve.i, &default_int, param_float, NULL},
            {3,"d_curve", ips200_x_max-10 * 7, 140,  &pid_steer_curve.d, &default_int, param_float, NULL},
            {3,"maxout_curve", ips200_x_max-10 * 7, 160,  &pid_steer_curve.maxout, &default_int, param_float, NULL},


    // {1,"START_THE_CAR",0,100,0,0,0,start_car,nfunc,nfunc},




    // {1,"end",0,0,0,0,0}//不可删去
    {1,"end",0,100,&default_float,&default_int,catlog,NULL}

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
        ips200_show_string(60,0,"edit_i");
        ips200_show_int(120,0,stepper_int[stepper_p_int],3);
    }
    if(menu_Mode==edit_float)
    {
        ips200_show_string(60,0,"edit_f");
        ips200_show_float(120,0,stepper_float[stepper_p_float],3,3);
    }
    if (target_priority==0)
    {
    ips200_show_string(0,0,"menu");//输出标题字符
        for(int i=0;strcmp(menu[i].str, "end") != 0;i++)
        {
            if(menu[i].priority==1)
            {
                if(i==p)
                {
                    ips200_show_string(0,menu[i].y,"->");//输出指向字符
                    ips200_show_string(20,menu[i].y,menu[i].str);
                }
                else
                {
                    ips200_show_string(20,menu[i].y,menu[i].str);
                }
             }
        }
    }
    else if(target_priority!=0)
    {
     ips200_show_string(0,0,menu[p_nearby].str);//输出上级字符
        for(int i=p_nearby+1;menu[i].priority!=target_priority;i++)
        {
            if(menu[i].priority==current_state)
            {
                if(i==p)
                {
                    ips200_show_string(0,menu[i].y,"->");//输出指向字符
                    ips200_show_string(20,menu[i].y,menu[i].str);
                    if(menu[i].type==param_float||menu[i].type==param_float_readonly)
                    {
                       ips200_show_float(menu[i].x,menu[i].y,*menu[i].value_f,3,3);
                    }
                    else if(menu[i].type==param_int||menu[i].type==param_int_readonly)
                    {
                       ips200_show_int(menu[i].x,menu[i].y,*menu[i].value_i,5);
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
                p++;
                while(menu[p].priority!=temp){p++;} 
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
 #include "menu.h"
#include "encoder.h"
#include "key.h"
#include "steer_pid.h"
#include "balance.h"
#include "PID.h"
#include "flash.h"
#include "photo_chuli.h"
#include "zf_device_lora3a22.h"

bool showline; 


#define ips200_x_max 240
#define ips200_y_max 320
int current_state=1;
int p=0;                        //记录当前指针
int p_nearby=0;                 //记录所属的指针
uint8 input;                    //菜单按键输入
extern int status;


extern uint8 flag;
bool show_flag=false;     //显示标志位,全局变量

int16 start_count=0;      //发车保护

//菜单调参
car_mode carmode=stop;                   //全局变量 车状态默认停止
stop_debug stopdebug=normal_debug;       //全局变量 停车debug默认正常
enum_menu_mode menu_Mode=normal;         //全局变量 菜单模式

int16 default_int=0;            //文件内变量 默认整型，防止空指针
float default_float=0.0;        //文件内变量 默认浮点型，防止空指针

uint8 confirm_flag=false;      //确认标志
int stepper_int[5]={1,5,10,20,50};                         //整型步进值
float stepper_float[6]={0.01,0.1,1.0,10.0,100.0,500.0};   //浮点型步进值
uint8 stepper_p_int=0;        //整型步进值指针
uint8 stepper_p_float=0;      //浮点型步进值指针

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
//加减封装函数（这个不要删了）↑↑↑↑↑↑
//菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量
enum_roadelementtypedef roadelementType[50]={zebra,straigh,curve
    ,straigh,ramp,crossr,straigh,speedup
    ,obstacle,islandl,straigh,zebra};//全局变量记录经过的元素
int16 element_num=12;       //全局变量，记录经过元素数量

int32 speed;
//前瞻显示+调节参数         用于track.h
int16 forwardsight;                 //默认前瞻
int16 forwardsight2;                //  直到判断前瞻！！！！注意这个和前瞻不同，用于三轮或者四轮车加速的！！！
int16 forwardsight3;                //弯道前瞻

//来自balance.c
extern float filtering_angle;       //解算出的角度
extern int16 pitch_angle_integr_read;          //俯仰角菜单显示的数据
extern int16 roll_angle_integr_read;           //横滚角菜单显示的数据
extern int16 yaw_angle_integr_read;            //偏航角菜单显示的数据
extern int16 raw_gyro_x ;                       //原始陀螺仪数据
extern int16 raw_gyro_y ;                       //原始陀螺仪数据
extern int16 raw_gyro_z ;                       //原始陀螺仪数据        

//来自zf_device_lora3a22.c
extern lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;

//pid显示+调节参数          来自pid.h
extern PID_t PID_gyro;          //角速度环
extern PID_t PID_angle;         //角度环
extern PID_t PID_speed;         //速度环  
extern PID_t PID_steer;         //转向环
extern PID_t PID_BLDC;          //负压风扇环 

struct_roadelementypedef roadelement_onoff={1,1,1,1,1,1,1,1,1,1,1,1,1,1};      //赛道元素功能开启关闭
struct_roadelementypedef roadelement_record={0,0,0,0,0,0,0,0,0,0,0,0,0,0};    //记录赛道元素
struct_imageshowcase image ={0,1,0};            //记录图像显示
bool startbool=false; //开始标志

//大津法显示＋调节参数          使用于photo_chuli.h
int16 threshold_down=100;       //大津法阈值上限
int16 threshold_up=200;         //大津法阈值下限  
int16 OTSU_calperxpage=5;       //每x张图片计算一次大津法

//来自      photo_chuli.c，分块大津法
extern int16 threshold1;  // 左上
extern int16 threshold2;  // 右上
extern int16 threshold3;  // 左下
extern int16 threshold4;  // 右下
//
//菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量菜单变量


//菜单函数菜单函数菜单函数菜单函数菜单函数菜单函数菜单函数
void show_element(void)
{
    for (int16 i = 0; i < element_num; i++)
    {
        switch (roadelementType[i])
        {
            
        case straigh:
            ips200_show_string(72*(i%3),30*(i/3),"straigh");
            break;
        case crossm:
            ips200_show_string(72*(i%3),30*(i/3)," crossM "); // 正入十字
            break;
        case crossl:
            ips200_show_string(72*(i%3),30*(i/3)," crossL "); // 左斜十字
            break;
        case crossr:
            ips200_show_string(72*(i%3),30*(i/3)," crossR "); // 右斜十字
            break;
        case islandl:
            ips200_show_string(72*(i%3),30*(i/3),"islandL"); // 环岛左
            break;
        case islandr:
            ips200_show_string(72*(i%3),30*(i/3),"islandR"); // 环岛右
            break;
        case scurve:
            ips200_show_string(72*(i%3),30*(i/3)," S-curve"); // S弯
            break;
        case curve:
            ips200_show_string(72*(i%3),30*(i/3)," curve "); // 弯道
            break;
        case speedup:
            ips200_show_string(72*(i%3),30*(i/3),"speedUp"); // 加速带
            break;
        case ramp:
            ips200_show_string(72*(i%3),30*(i/3),"  ramp "); // 坡道
            break;
        case obstacle:
            ips200_show_string(72*(i%3),30*(i/3),"obstacle"); // 障碍物
            break;
        case blackprotect:
            ips200_show_string(72*(i%3),30*(i/3),"blkProt"); // 黑线保护
            break;
        case stall:
            ips200_show_string(72*(i%3),30*(i/3)," stall "); // 堵转
            break;
        case zebra:
            ips200_show_string(72*(i%3),30*(i/3),"  zebra "); // 斑马线
            break;
        default:
            break;
        }
        if(i!=element_num-1)
        {
            ips200_set_color(RGB565_ORANGE, RGB565_BLACK);
            ips200_show_string(72*(i%3)+64,30*(i/3),">");
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);

        }

    }
}
//用于显示经过元素
void show_stopreason(void)
{
    ips200_show_string(0,0,"stop reason:");
    if(stopdebug==blackprotect_stop)
    {
        ips200_show_string(0,20,"blackprotect stop");
    }

    if(stopdebug==zebra_stop)
    {
        ips200_show_string(0,20,"zebra stop");
    }
    if(stopdebug==liftup_stop)
    {
        ips200_show_string(0,20,"liftup stop");
    }
    if (stopdebug==gyro_intrg_pitch_stop)
    {
        ips200_show_string(0,20,"gyro_intrg_pitch stop");
    }
    if (stopdebug==gyro_intrg_roll_stop)
    {
        ips200_show_string(0,20,"gyro_intrg_roll stop");
    }
    if (stopdebug==gyro_intrg_yaw_stop)         
    {
        ips200_show_string(0,20,"gyro_intrg_yaw stop");
    }
    if (stopdebug==timer_count_stop)          
    {
        ips200_show_string(0,20,"timer stop");
    }
 
}
//用于显示停止原因
void image_show()   {show_flag=true;}
void PID_clear()
{
	PID_gyro.error0 = 0;
    PID_gyro.errorint = 0;
	PID_angle.error0 = 0;
	PID_speed.errorint = 0;
	PID_steer.error0 = 0;
	PID_steer.errorint = 0; 
}
//误差清除函数防止爆炸(填的坑记得补哈)
void start_the_car() { carmode = car_run_mode1;start_count=0; angle_init();PID_clear();}//开始
void Calibrate_BLDC()   {carmode=Start_Calibrate;}
void Remote_start()     {carmode =remote;start_count=0; angle_init();PID_clear();}//远程开始

void pid_gyro_set0(){ PID_gyro.kp=0;PID_gyro.ki=0;PID_gyro.kd=0;PID_gyro.kd2=0;PID_gyro.maxout=5000;PID_gyro.minout=-5000;  ips200_show_string(0,180,"set 0 already");} 
void pid_angle_set0(){PID_angle.kp=0;PID_angle.ki=0;PID_angle.kd=0;PID_angle.kd2=0;PID_angle.maxout=5000;PID_angle.minout=-5000;ips200_show_string(0,180,"set 0 already");}     
void pid_V_set0(){PID_speed.kp=0;PID_speed.ki=0;PID_speed.kd=0;PID_speed.kd2=0;PID_speed.maxout=5000;PID_speed.minout=-5000;PID_speed.targ=400;ips200_show_string(0,180,"set 0 already");} 
void pid_steer_set0(){PID_steer.kp=0;PID_steer.ki=0;PID_steer.kd=0;PID_steer.kd2=0;PID_steer.maxout=5000;PID_steer.minout=-5000; ips200_show_string(0,180,"set 0 already");} 
void pid_BLDC_set0(){PID_BLDC.kp=0;PID_BLDC.ki=0;PID_BLDC.kd=0;PID_BLDC.kd2=0;PID_BLDC.maxout=0;PID_BLDC.minout=0; ips200_show_string(0,180,"set 0 already");}
void pid_all_set0(){pid_gyro_set0();pid_angle_set0();pid_V_set0();pid_steer_set0();pid_BLDC_set0();}
void pid_BLDC_mode_set(){pid_gyro_set0();PID_gyro.maxout=100;PID_gyro.minout=0;pid_angle_set0();PID_angle.maxout=40;
    PID_angle.minout=0;pid_V_set0();PID_speed.maxout=100;PID_speed.minout=0;
    pid_steer_set0();PID_steer.maxout=100;PID_steer.minout=0;pid_BLDC_set0();
    PID_BLDC.maxout=0;PID_BLDC.minout=0;}
//闪存存储 



//代码存储
void codeload1(){}
void codeload2(){}
void codeload3(){}
void codeload4(){}
//结构体变量置0初始化(有参数)


 
//菜单结构体
MENU menu[] = 
{
    {1,"start",                   0,               20, {.param_float=&default_float}, catlog,         NULL},
        {2,"car_go",              0,               20, {.param_float=&default_float}, function,       start_the_car},
        {2,"Calibrate",           0,               40, {.param_float=&default_float}, function,       Calibrate_BLDC},
        {2,"remote_start",        0,               60, {.param_float=&default_float}, function,       Remote_start},

    {1, "pidparam",               0,               40, {.param_float=&default_float}, catlog,         NULL},
        {2, "PID_gyro",           0,               20, {.param_float=&default_float}, catlog,         NULL},
            {3, "kp",          ips200_x_max-10 * 8,  20, {.param_float=&PID_gyro.kp}, param_float,   NULL},
            {3, "ki",          ips200_x_max-10 * 8,  40, {.param_float=&PID_gyro.ki}, param_float,   NULL},
            {3, "kd",          ips200_x_max-10 * 8,  60, {.param_float=&PID_gyro.kd}, param_float,   NULL},
            {3, "maxout",      ips200_x_max-10 * 8,  80, {.param_float=&PID_gyro.maxout}, param_float,  NULL},
            {3, "minout",      ips200_x_max-10 * 8, 100, {.param_float=&PID_gyro.minout}, param_float, NULL},
        {2, "PID_angle",      0,                   40, {.param_float=&default_float}, catlog,      NULL},
            {3, "kp",        ips200_x_max-10 * 8,  20, {.param_float=&PID_angle.kp}, param_float, NULL},
            {3, "ki",        ips200_x_max-10 * 8,  40, {.param_float=&PID_angle.ki}, param_float, NULL},
            {3, "kd",        ips200_x_max-10 * 8,  60, {.param_float=&PID_angle.kd}, param_float, NULL},
            {3, "maxout",    ips200_x_max-10 * 8,  80, {.param_float=&PID_angle.maxout}, param_float,NULL},
            {3, "minout",    ips200_x_max-10 * 8, 100, {.param_float=&PID_angle.minout}, param_float, NULL},
        {2, "PID_V",         0,                   60, {.param_float=&default_float}, catlog,      NULL},
            {3, "kp",        ips200_x_max-10 * 8,  20, {.param_float=&PID_speed.kp}, param_float, NULL},
            {3, "ki",        ips200_x_max-10 * 8,  40, {.param_float=&PID_speed.ki}, param_float, NULL},
            {3, "kd",        ips200_x_max-10 * 8,  60, {.param_float=&PID_speed.kd}, param_float, NULL},
            {3, "maxout",    ips200_x_max-10 * 8,  80, {.param_float=&PID_speed.maxout}, param_float,NULL},
            {3, "minout",    ips200_x_max-10 * 8, 100, {.param_float=&PID_speed.minout}, param_float, NULL},
            {3,"target",     ips200_x_max-10 * 8, 120, {.param_float=&PID_speed.targ}, param_float, NULL},
        {2, "PID_steer",      0,                  80, {.param_float=&default_float}, catlog,      NULL},
            {3, "kp",        ips200_x_max-10 * 8,  20, {.param_float=&PID_steer.kp}, param_float, NULL},
            {3, "ki",        ips200_x_max-10 * 8,  40, {.param_float=&PID_steer.ki}, param_float, NULL},
            {3, "kd",        ips200_x_max-10 * 8,  60, {.param_float=&PID_steer.kd}, param_float, NULL},
            {3, "kd2",       ips200_x_max-10 * 8,  80, {.param_float=&PID_steer.kd2}, param_float, NULL},
            {3, "maxout",    ips200_x_max-10 * 8, 100, {.param_float=&PID_steer.maxout}, param_float,NULL},
            {3, "minout",    ips200_x_max-10 * 8, 120, {.param_float=&PID_steer.minout}, param_float, NULL},
        {2, "PID_BLDC",      0,                  100, {.param_float=&default_float}, catlog,      NULL},
            {3,"kp",        ips200_x_max-10 * 8,  20, {.param_float=&PID_BLDC.kp}, param_float, NULL},
            {3,"ki",        ips200_x_max-10 * 8,  40, {.param_float=&PID_BLDC.ki}, param_float, NULL},
            {3,"kd",        ips200_x_max-10 * 8,  60, {.param_float=&PID_BLDC.kd}, param_float, NULL},
            {3,"kd2",       ips200_x_max-10 * 8,  80, {.param_float=&PID_BLDC.kd2}, param_float, NULL},
            {3,"maxout",    ips200_x_max-10 * 8, 100, {.param_float=&PID_BLDC.maxout}, param_float,NULL},
            {3,"minout",    ips200_x_max-10 * 8, 120, {.param_float=&PID_BLDC.minout}, param_float,NULL},
        {2, "allset0",       0,                 120, {.param_float=&default_float}, confirm,     pid_all_set0},
        {2, "PID_gyro_set0", 0,                 140, {.param_float=&default_float}, confirm,     pid_gyro_set0},
        {2, "PID_angle_set0",0,                 160, {.param_float=&default_float}, confirm,     pid_angle_set0},
        {2, "PID_V_set0",    0,                 180, {.param_float=&default_float}, confirm,     pid_V_set0},
        {2, "PID_steer_set0",0,                 200, {.param_float=&default_float}, confirm,     pid_steer_set0},
        {2,"PID_BLDC_modeset0",0,               220, {.param_float=&default_float}, confirm,     pid_BLDC_mode_set},
    
    {1, "image",              0,                  60, {.param_float=&default_float}, catlog,      NULL},
        {2, "ROLL_angle",    100,                 20, {.param_float=&filtering_angle}, param_float_readonly, NULL},
        {2, "display",       0,                  40, {.param_float=&default_float}, function,    image_show},
        {2, "show_image",    0,                  60, {.param_float=&default_float}, catlog,    NULL},
            {3, "show_grayimage",180,             20, {.param_float=&default_float}, chose1, NULL},
            {3, "show_ostuimage",180,             40, {.param_float=&default_float}, chose1, NULL},
            {3, "show_dev_image",180,             60, {.param_float=&default_float}, chose1, NULL},
        {2, "OTSU_threshold",0,                  80, {.param_float=&default_float}, catlog, NULL},
            {3, "OTSU_up",   100,                20, {.param_int16=&threshold_up}, param_int16, NULL},
            {3, "OTSU_DOWN",100,                 40, {.param_int16=&threshold_down}, param_int16, NULL},
            {3,"OTSU_perx", 100,                 60, {.param_int16=&OTSU_calperxpage}, param_int16, NULL},
            {3,"threshold1",100,                 80, {.param_int16=&threshold1}, param_int16, NULL},
            {3,"threshold2",100,                100, {.param_int16=&threshold2}, param_int16, NULL},
            {3,"threshold3",100,                120, {.param_int16=&threshold3}, param_int16, NULL},
            {3,"threshold4",100,                140, {.param_int16=&threshold4}, param_int16, NULL},
        {2, "image_point",   0,                 100, {.param_float=&default_float}, catlog, NULL},
            {3, "crossroadall",0,                20, {.param_float=&default_float}, catlog, NULL},
                {4, "r_up_p",100,                20, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "r_down_p",100,              40, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "l_up_p",100,                60, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "l_down_p",100,              80, {.param_int16=&default_int}, param_int16_readonly, NULL},
            {3, "round",     0,                  40, {.param_float=&default_float}, catlog, NULL},
        {2, "forwardsight",  0,                 120, {.param_float=&default_float}, catlog, NULL},
            {3, "forwardsight1",150,             20, {.param_int16=&forwardsight}, param_int16, NULL},
            {3, "forwardsight2",150,             40, {.param_int16=&forwardsight2}, param_int16, NULL},
            {3, "forwardsight3",150,             60, {.param_int16=&forwardsight3}, param_int16, NULL},
    
    {1, "debug",      0,                  80, {.param_float=&default_float}, catlog, NULL},
        {2,"gyro_info",      0,                 20, {.param_float=&default_float}, catlog, NULL},
            {3,"pit_intg",   100,               20, {.param_int16=&pitch_angle_integr_read}, param_int16_readonly, NULL},
            {3,"yaw_intg",   100,               40, {.param_int16=&yaw_angle_integr_read}, param_int16_readonly, NULL},
            {3,"roll_intg",  100,               60, {.param_int16=&roll_angle_integr_read}, param_int16_readonly, NULL},
            {3, "ROLL_angle",100,               80, {.param_float=&filtering_angle}, param_float_readonly, NULL},
            {3,"raw_gyro_x", 100,              100, {.param_int16=&raw_gyro_x}, param_int16_readonly, NULL},
            {3,"raw_gyro_y", 100,              120, {.param_int16=&raw_gyro_y}, param_int16_readonly, NULL},
            {3,"raw_gyro_z", 100,              140, {.param_int16=&raw_gyro_z}, param_int16_readonly, NULL},
            {3,"imu_gyro_x", 100,              160, {.param_int16=&imu660ra_gyro_x}, param_int16_readonly, NULL},
        {2,"remote_info",    0,                 40, {.param_float=&default_float}, catlog, NULL},
            {3,"l_stick_UD", 150,               20, {.param_int16=&lora3a22_uart_transfer.joystick[1]}, param_int16_readonly, NULL},
            {3,"l_stick_LR", 150,               40, {.param_int16=&lora3a22_uart_transfer.joystick[0]}, param_int16_readonly, NULL},
            {3,"r_stick_UD", 150,               60, {.param_int16=&lora3a22_uart_transfer.joystick[3]}, param_int16_readonly, NULL},
            {3,"r_stick_LR", 150,               80, {.param_int16=&lora3a22_uart_transfer.joystick[2]}, param_int16_readonly, NULL},
            {3,"l_stickey",  150,              100, {.param_uint8=&lora3a22_uart_transfer.key[0]}, param_uint8_readonly, NULL},
            {3,"r_stickey",  150,              120, {.param_uint8=&lora3a22_uart_transfer.key[1]}, param_uint8_readonly, NULL},
            {3,"l_key",      150,              140, {.param_uint8=&lora3a22_uart_transfer.key[2]}, param_uint8_readonly, NULL},
            {3,"r_key",      150,              160, {.param_uint8=&lora3a22_uart_transfer.key[3]}, param_uint8_readonly, NULL},
            {3,"Lswitch_key1",150,             180, {.param_uint8=&lora3a22_uart_transfer.switch_key[0]}, param_uint8_readonly, NULL},
            {3,"Lswitch_key2",150,             200, {.param_uint8=&lora3a22_uart_transfer.switch_key[1]}, param_uint8_readonly, NULL},
            {3,"Rswitch_key1",150,             220, {.param_uint8=&lora3a22_uart_transfer.switch_key[2]}, param_uint8_readonly, NULL},
            {3,"Rswitch_key2",150,             240, {.param_uint8=&lora3a22_uart_transfer.switch_key[3]}, param_uint8_readonly, NULL},

    {1, "element",  0,                  100, {.param_float=&default_float}, catlog, NULL},
        {2, "element_onoff", 0,                  20, {.param_float=&default_float}, catlog, NULL},
            {3, "crossl",    100,                20, {.param_int16=&roadelement_onoff.crossl}, on_off, NULL},
            {3, "crossr",    100,                40, {.param_int16=&roadelement_onoff.crossr}, on_off, NULL},
            {3, "crossm",    100,                60, {.param_int16=&roadelement_onoff.crossm}, on_off, NULL},
            {3, "islandl",   100,                80, {.param_int16=&roadelement_onoff.islandl}, on_off, NULL},
            {3, "islandR",   100,               100, {.param_int16=&roadelement_onoff.islandr}, on_off, NULL},
            {3, "scurve",    100,               120, {.param_int16=&roadelement_onoff.scurve}, on_off, NULL},
            {3, "speedup",   100,               140, {.param_int16=&roadelement_onoff.speedup}, on_off, NULL},
            {3, "ramp",      100,               160, {.param_int16=&roadelement_onoff.ramp}, on_off, NULL},
            {3, "obstacle",  100,               180, {.param_int16=&roadelement_onoff.obstacle}, on_off, NULL},
        {2, "element_count",0,                  40, {.param_float=&default_float}, catlog, NULL},
            {3, "straigh",  100,                20, {.param_int16=&roadelement_record.straigh}, param_int16_readonly, NULL},
            {3, "crossm",    100,                40, {.param_int16=&roadelement_record.crossm}, param_int16_readonly, NULL},
            {3, "crossl",    100,                60, {.param_int16=&roadelement_record.crossl}, param_int16_readonly, NULL},
            {3, "crossr",    100,                80, {.param_int16=&roadelement_record.crossr}, param_int16_readonly, NULL},
            {3, "islandl",   100,               100, {.param_int16=&roadelement_record.islandl}, param_int16_readonly, NULL},
            {3, "islandr",   100,               120, {.param_int16=&roadelement_record.islandr}, param_int16_readonly, NULL},
            {3, "scurve",    100,               140, {.param_int16=&roadelement_record.scurve}, param_int16_readonly, NULL},
            {3, "curve",     100,               160, {.param_int16=&roadelement_record.curve}, param_int16_readonly, NULL},
            {3, "speedup",   100,               180, {.param_int16=&roadelement_record.speedup}, param_int16_readonly, NULL},
            {3, "ramp",      100,               200, {.param_int16=&roadelement_record.ramp}, param_int16_readonly, NULL},
            {3, "obstacle",  100,               220, {.param_int16=&roadelement_record.obstacle}, param_int16_readonly, NULL},
            {3, "blackprotect",100,             240, {.param_int16=&roadelement_record.blackprotect}, param_int16_readonly, NULL},
            {3, "stall",     100,               260, {.param_int16=&roadelement_record.stall}, param_int16_readonly, NULL},
            {3, "zebra",     100,               280, {.param_int16=&roadelement_record.zebra}, param_int16_readonly, NULL},
        {2, "element_gothrough",0,              60, {.param_float=&default_float}, roadgothrough, NULL},
        {2, "record_clear", 0,                  80, {.param_float=&default_float}, function, NULL},
    
    {1, "flash",              0,                 120, {.param_float=&default_float}, catlog, NULL},
        {2, "code_load",    100,                 20, {.param_float=&default_float}, catlog, NULL},
            {3, "load1",    100,                 20, {.param_float=&default_float}, confirm, NULL},
            {3, "load2",    100,                 40, {.param_float=&default_float}, confirm, NULL},
            {3, "load3",    100,                 60, {.param_float=&default_float}, confirm, NULL},
            {3, "load4",    100,                 80, {.param_float=&default_float}, confirm, NULL},
        {2, "flash_load",   100,                 40, {.param_float=&default_float}, catlog, NULL},
            {3, "load1",    100,                 20, {.param_float=&default_float}, confirm, flash_load_config_1},
            {3, "load2",    100,                 40, {.param_float=&default_float}, confirm, flash_load_config_2},
            {3, "load3",    100,                 60, {.param_float=&default_float}, confirm, flash_load_config_3},
            {3, "load4",    100,                 80, {.param_float=&default_float}, confirm, flash_load_config_4},
            {3, "loaddefault",100,              100, {.param_float=&default_float}, confirm, flash_load_config_default},
        {2, "flash_save",   100,                 60, {.param_float=&default_float}, catlog, NULL},
            {3, "save1",    100,                 20, {.param_float=&default_float}, confirm, flash_save_config_1},
            {3, "save2",    100,                 40, {.param_float=&default_float}, confirm, flash_save_config_2},
            {3, "save3",    100,                 60, {.param_float=&default_float}, confirm, flash_save_config_3},
            {3, "save4",    100,                 80, {.param_float=&default_float}, confirm, flash_save_config_4},
        {2, "resetflash",   100,                 80, {.param_float=&default_float}, confirm, flash_reset},
    
    {1, "setting",            0,                 140, {.param_float=&default_float}, catlog, NULL},
    {1, "end",                0,                   0, {.param_float=&default_float}, catlog, NULL}
};



enum_Condition condition = NOACTION;//菜单行为初始化
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
函数简介     快速显示
参数说明     X1为字符串显示起始位
                X2为数值显示起始位
                Y为显示行
                type为显示类型
                *a为整型参数地址
                *p为浮点型参数地址
                str为参数名字
返回参数     无
使用示例     display_fast(0, 100, 0, param_int16, &default_int,  "default");
备注信息     在0，0位置显示default参数名，在0行100列显示default_int整型参数值
-------------------------------------------------------------------------------------------------------------------
*/

union_param fast_show[7]={
{.param_float=&filtering_angle},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float}
};

void display_fast(int16 X1,int16 X2,int16 Y,enum_function type,union_param Union_param,char str[20])
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(X1,Y,str);
    switch (type)
    {
    case param_uint8:
        ips200_show_int(X2,Y,*Union_param.param_uint8,5);
        break;
    case param_int8:
        ips200_show_int(X2,Y,*Union_param.param_int8,5);    
        break;
    case param_uint16:
        ips200_show_int(X2,Y,*Union_param.param_uint16,5);
        break;
    case param_int16:
        ips200_show_int(X2,Y,*Union_param.param_int16,5);
        break;
    case param_uint32:
        ips200_show_int(X2,Y,*Union_param.param_uint32,10); 
        break;
    case param_int32:       
        ips200_show_int(X2,Y,*Union_param.param_int32,10); 
        break;
    case param_float:
        ips200_show_float(X2,Y,*Union_param.param_float,5,3); 
        break;
    case param_double:
        ips200_show_float(X2,Y,*Union_param.param_double,10,6); 
        break;    
    default:
        break;
    }

}
void outputscreen_fast()
{
    if(show_flag==false&&current_state==1&&menu_Mode==normal)//如果图像显示没开且在顶级菜单
    {    
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(0,160,"fast_show");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        //要添加快速显示请在这里增加

        display_fast(0, 60, 180, param_float,fast_show[0] , "ROLL_angle");
    }


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
    int16 target_priority=current_state-1;
    outputscreen_fast(); //快速显示
    if(menu_Mode==edit_int)     //整型编辑模式下
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);    //设置为棕色底黑字
        ips200_show_string(100,0,"len_i");
        ips200_show_int(160,0,stepper_int[stepper_p_int],3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为棕色底黑字

    }
    if(menu_Mode==edit_float)       //浮点编辑模式下
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);    //设置为棕色底黑字
        ips200_show_string(100,0,"len_f");
        ips200_show_float(160,0,stepper_float[stepper_p_float],3,3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //设置为棕色底黑字

    }
    if(menu_Mode==edit_confirm)     //确认模式下
    {
        ips200_set_color(RGB565_ORANGE,RGB565_BLACK);
        ips200_show_string(20,0,"WARNING!WARNING!WARNING!");
        ips200_show_string(20,160,"PRESS BOTTON3 TO CONFIRM");
        ips200_show_string(20,300,"WARNING!WARNING!WARNING!");
        return;                     //提前退出
    }
    if(menu_Mode==special_show_element1)//显示经过元素模式下
    {
        show_element();
        return;
    }
    if(menu_Mode==stop_debug_display)//停止调试显示模式下
    {
        show_stopreason();
        return;                     //提前退出
    }
    if(target_priority==0)         //顶级菜单
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
                    if(menu_Mode==normal)       //普通显示，仅仅显示箭头
                    {
                        ips200_show_string(0,menu[i].y,"->");//输出指向字符
                        ips200_show_string(20,menu[i].y,menu[i].str);
                    }
                    else if(menu_Mode==edit_int||menu_Mode==edit_float) //编辑模式下，改变颜色    
                    {
                        ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);//设置为红色黑底
                        ips200_show_string(0,menu[i].y,"->");//输出指向字符
                        ips200_show_string(20,menu[i].y,menu[i].str);
                        ips200_set_color(RGB565_WHITE, RGB565_BLACK);//设置为黑底白字
                    }
                    if(menu[i].type==param_float||menu[i].type==param_float_readonly)
                    {
                       ips200_show_float(menu[i].x,menu[i].y,*menu[i].param_union.param_float,4,3);
                    }
                    else if(menu[i].type>=param_int8&&menu[i].type<=param_uint32_readonly)
                    {
                        int32 temp;
                        switch (menu[i].type)
                        {
                        case param_int8:
                            temp = (int32)(*menu[i].param_union.param_int8);
                            break;
                        case param_uint8:
                            temp = (int32)(*menu[i].param_union.param_uint8);
                            break;
                        case param_int16:
                            temp = (int32)(*menu[i].param_union.param_int16);   
                            break;
                        case param_uint16:
                            temp = (int32)(*menu[i].param_union.param_uint16);
                            break;
                        case param_int32:
                            temp = *menu[i].param_union.param_int32;
                            break;
                        case param_uint32:
                            temp = (int32)(*menu[i].param_union.param_uint32);
                            break;
                        case param_uint8_readonly:
                            temp = (int32)(*menu[i].param_union.param_uint8);
                            break;
                        case param_int8_readonly:
                            temp = (int32)(*menu[i].param_union.param_int8);
                            break;
                        case param_uint16_readonly:
                            temp = (int32)(*menu[i].param_union.param_uint16);
                            break;
                        case param_int16_readonly:
                            temp = (int32)(*menu[i].param_union.param_int16);   
                            break;
                        case param_int32_readonly:
                            temp = *menu[i].param_union.param_int32;    
                            break;
                        case param_uint32_readonly: 
                            temp = (int32)(*menu[i].param_union.param_uint32);
                            break;

                                
                        default:
                            break;
                        }
                       ips200_show_int(menu[i].x,menu[i].y,temp,5);
                    }
                    else if(menu[i].type==on_off||menu[i].type==chose1)
                    {
                        ips200_set_color(RGB565_ORANGE, RGB565_BLACK);    //设置为红色黑底
                        if (*menu[i].param_union.param_int16)
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
                       ips200_show_float(menu[i].x,menu[i].y,*menu[i].param_union.param_float,4,3);
                    }
                    else if(menu[i].type>=param_int8&&menu[i].type<=param_uint32_readonly)
                    {
                        int32 temp;
                        switch (menu[i].type)
                        {
                        case param_int8:
                            temp = (int32)(*menu[i].param_union.param_int8);
                            break;
                        case param_uint8:
                            temp = (int32)(*menu[i].param_union.param_uint8);
                            break;
                        case param_int16:
                            temp = (int32)(*menu[i].param_union.param_int16);   
                            break;
                        case param_uint16:
                            temp = (int32)(*menu[i].param_union.param_uint16);
                            break;
                        case param_int32:
                            temp = *menu[i].param_union.param_int32;
                            break;
                        case param_uint32:
                            temp = (int32)(*menu[i].param_union.param_uint32);
                            break;
                        case param_uint8_readonly:
                            temp = (int32)(*menu[i].param_union.param_uint8);
                            break;
                        case param_int8_readonly:
                            temp = (int32)(*menu[i].param_union.param_int8);
                            break;
                        case param_uint16_readonly:
                            temp = (int32)(*menu[i].param_union.param_uint16);
                            break;
                        case param_int16_readonly:
                            temp = (int32)(*menu[i].param_union.param_int16);
                            break;
                        case param_int32_readonly:
                            temp = *menu[i].param_union.param_int32;    
                            break;
                        default:
                            break;
                        }
                       ips200_show_int(menu[i].x,menu[i].y,temp,5);
                    }
                    else if(menu[i].type==on_off||menu[i].type==chose1)
                    {
                        if (*menu[i].param_union.param_int16)
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
        condition = (enum_Condition)input; 
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
                switch (menu[p].type)
                {
                case param_int8:
                    *menu[p].param_union.param_int8-=stepper_int[stepper_p_int];
                    break;
                case param_uint8:
                    *menu[p].param_union.param_uint8-=stepper_int[stepper_p_int];
                    break;
                case param_int16:
                    *menu[p].param_union.param_int16-=stepper_int[stepper_p_int];
                    break;
                case param_uint16:
                    *menu[p].param_union.param_uint16-=stepper_int[stepper_p_int];
                    break;
                case param_int32:
                    *menu[p].param_union.param_int32-=stepper_int[stepper_p_int];
                    break;
                case param_uint32:
                    *menu[p].param_union.param_uint32-=stepper_int[stepper_p_int];
                    break;
                default:
                    break;
                }
                // *menu[p].param_union.param_int16-=stepper_int[stepper_p_int];
                return;
            }
            if(menu_Mode==edit_float)
            { 
                switch (menu[p].type)
                {
                case param_float:
                    *menu[p].param_union.param_float-=stepper_float[stepper_p_float];
                    break;
                case param_double:
                    *menu[p].param_union.param_double-=stepper_float[stepper_p_float];
                    break;
                
                default:
                    break;
                }
                return;
            }
            if (strcmp(menu[p].str, "end") != 0&&menu[p+1].priority>=menu[p].priority)        //换菜单等级(读者不要动)

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
        //增减功能实现  增减功能实现  增减功能实现  增减功能实现  增减功能实现  增减功能实现  增减功能实现  
            if(menu_Mode==edit_int)         //整型编辑模式下,包含所有整型数据类型
            {
                switch (menu[p].type)
                {
                case param_int8:
                    *menu[p].param_union.param_int8+=stepper_int[stepper_p_int];
                    break;
                case param_uint8:
                    *menu[p].param_union.param_uint8+=stepper_int[stepper_p_int];
                    break;
                case param_int16:
                    *menu[p].param_union.param_int16+=stepper_int[stepper_p_int];
                    break;
                case param_uint16:
                    *menu[p].param_union.param_uint16+=stepper_int[stepper_p_int];
                    break;
                case param_int32:
                    *menu[p].param_union.param_int32+=stepper_int[stepper_p_int];
                    break;
                case param_uint32:
                    *menu[p].param_union.param_uint32+=stepper_int[stepper_p_int];
                    break;
                default:
                    break;
                }
                return;
            }
            if(menu_Mode==edit_float)       //浮点编辑模式下,包含所有浮点数据类型
            { 
                switch (menu[p].type)
                {
                case param_float:
                    *menu[p].param_union.param_float+=stepper_float[stepper_p_float];
                    break;
                case param_double:
                    *menu[p].param_union.param_double+=stepper_float[stepper_p_float];
                    break;
                
                default:
                    break;
                }
                return;
            }
        //增减功能实现  增减功能实现  增减功能实现  增减功能实现  增减功能实现  增减功能实现  增减功能实现  
            if(p!=0&&menu[p-1].priority>=menu[p].priority)      //换菜单等级(读者不要动)
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
        //换菜单等级(读者不要动)
            if(menu[p+1].priority==current_state+1&&strcmp(menu[p+1].str,"end")!=0&&menu[p].type==catlog)       //子目录情况
            {
                current_state++;
                p_nearby=p;
                p++;
                break;
            }
        //菜单模式情况
            if(menu_Mode==edit_int)                         //整型编辑模式下
            {
                stepper_p_int=(stepper_p_int+1)%5;
                break;
            }
            if(menu_Mode==edit_float)                       //浮点编辑模式下
            {
                stepper_p_float=(stepper_p_float+1)%6;
                break;
            }
            if(menu_Mode==edit_confirm)                     //确认模式下
            {
                ips200_clear();                             //清屏
                menu[p].Operate_default();
                menu_Mode=normal;
                break;
            }
        //菜单种类情况
            if(menu[p].type==param_float||menu[p].type==param_double)                   //菜单参数情况                      
            {
                menu_Mode=edit_float;
                break;
                //进入浮点数编辑模式
            }
            if(menu[p].type==param_int16||menu[p].type==param_int32
                ||menu[p].type==param_int8||menu[p].type==param_uint16
                ||menu[p].type==param_uint32||menu[p].type==param_uint8)                 //菜单参数情况
            {
                menu_Mode=edit_int;
                break;
                //进入整型编辑模式
            }
            if (menu[p].type==confirm)                  //确认情况
            {
                menu_Mode=edit_confirm;
                break;
            }
            if(menu[p].type==on_off)                    //开关情况
            {
                *menu[p].param_union.param_int16=1-*menu[p].param_union.param_int16;
                break;
            }
            if(menu[p].type==function)                  //函数情况
            {
                menu[p].Operate_default();
                break;
            }
            if(menu[p].type==roadgothrough)             //赛道元素通过情况
            {
                menu_Mode=special_show_element1;        //进入显示经过元素模式
                break;
            }
            if(menu[p].type==param_float_readonly||menu[p].type==param_int16_readonly
                ||menu[p].type==param_int32_readonly||menu[p].type==param_uint16_readonly
                ||menu[p].type==param_uint32_readonly||menu[p].type==param_double_readonly
                ||menu[p].type==param_int8_readonly||menu[p].type==param_uint8_readonly)        //只读参数情况
            {
                ips200_show_string(0,180,"error_readonly");
            }
            if(menu[p].type==chose1)        //同一父级下同等级同种类只选一个
            {
                for(int i=p_nearby+1;menu[i].priority!=current_state-1;i++)
                {
                    if(menu[i].priority==current_state&&menu[i].type==chose1&&i!=p)
                    {
                        *menu[i].param_union.param_int16=0;
                    }
                    if(i==p)
                    {
                        *menu[p].param_union.param_int16=1;
                    }
                }
                break;
            }
            break;
        case BACK:
        if(menu_Mode==edit_float||menu_Mode==edit_confirm||menu_Mode==edit_int
        ||menu_Mode==special_show_element1||menu_Mode==stop_debug_display) //编辑模式下按返回键退出编辑模式
        {
            menu_Mode=normal;
            break;
        }
        if(menu[p].priority==1)
        {
            flash_save_config_default();
            ips200_set_color(RGB565_PURPLE,RGB565_BLACK);
            ips200_show_string(0,300,"save default already");
            ips200_set_color(RGB565_WHITE,RGB565_BLACK);

            break;
        }
        if(menu[p].priority!=1)                 //换菜单等级(读者不要动)
        {
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

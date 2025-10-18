#include "menu.h"
#include "encoder.h"
#include "key.h"
#include "steer_pid.h"
#include "balance.h"
#include "PID.h"
#include "flash.h"
#include "photo_chuli.h"
#include "zf_device_lora3a22.h"
#include "BLDC.h"

bool showline; // 显示线条标志

#define ips200_x_max 240 // IPS屏幕最大X坐标
#define ips200_y_max 320 // IPS屏幕最大Y坐标
int current_state=1;    // 当前菜单层级
int p=0;                // 当前菜单项指针
int p_nearby=0;         // 邻近菜单项指针
uint8 input;            // 菜单输入值
extern int status;      // 外部状态变量

extern uint8 flag;      // 外部标志位
bool show_flag=false;   // 显示标志位，全局变量

int32 start_count=0;    // 启动计数器

// 菜单相关变量
car_mode carmode=stop;                   // 小车模式，默认为停止
stop_debug stopdebug=normal_debug;       // 停止调试模式，默认为普通调试
enum_menu_mode menu_Mode=normal;         // 菜单模式，默认为普通模式

int16 default_int=0;            // 默认整数值
float default_float=0.0;        // 默认浮点值

uint8 confirm_flag=false;      // 确认标志
int stepper_int[5]={1,5,10,20,50};       // 整型步进值
float stepper_float[6]={0.01,0.1,1.0,10.0,100.0,500.0}; // 浮点步进值
uint8 stepper_p_int=0;        // 整型步进指针
uint8 stepper_p_float=0;      // 浮点步进指针

// 整型参数增加
void add_intparam(int16* a)
{
    *a+=stepper_int[stepper_p_int];
}
// 整型参数减少
void sub_intparam(int16* a)
{
    *a-=stepper_int[stepper_p_int];
}
// 浮点参数增加
void add_floatparam(float* a)
{
    *a+=stepper_float[stepper_p_float];
}
// 浮点参数减少
void sub_floatparam(float* a)
{
    *a-=stepper_float[stepper_p_float];
}

// 道路元素类型枚举
enum_roadelementtypedef roadelementType[50]={zebra,straigh,curve
    ,straigh,ramp,crossr,straigh,speedup
    ,obstacle,islandl,straigh,zebra}; // 记录道路元素类型
int16 element_num=12;       // 记录道路元素数量

int32 speed; // 速度变量

// 前瞻距离设置
int16 forwardsight;         // 默认前瞻
int16 forwardsight2;        // 直道判断前瞻
int16 forwardsight3;        // 弯道前瞻

// 平衡相关变量
extern float filtering_angle;       // 滤波后的角度
extern int16 pitch_angle_integr_read; // 俯仰角积分值
extern int16 roll_angle_integr_read;  // 横滚角积分值
extern int16 yaw_angle_integr_read;   // 偏航角积分值
extern int16 raw_gyro_x;             // 原始陀螺仪X值
extern int16 raw_gyro_y;             // 原始陀螺仪Y值
extern int16 raw_gyro_z;             // 原始陀螺仪Z值        

// LoRa通信相关
extern lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;

// PID控制器
extern PID_t PID_gyro;      // 角速度环
extern PID_t PID_angle;     // 角度环
extern PID_t PID_speed;     // 速度环  
extern PID_t PID_steer;     // 转向环
extern PID_t PID_BLDC;      // 电压闭环 

// BLDC电机参数
extern BLDC_Param bldc_param;

// 道路元素开关和记录
struct_roadelementypedef roadelement_onoff={1,1,1,1,1,1,1,1,1,1,1,1,1,1}; // 道路元素功能开关
struct_roadelementypedef roadelement_record={0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // 记录道路元素
struct_imageshowcase image ={0,1,0}; // 图像显示设置
bool startbool=false; // 启动标志

// 图像处理参数
int16 threshold_down=100;   // 二值化下限
int16 threshold_up=200;     // 二值化上限  
int16 OTSU_calperxpage=5;   // 每x行图像计算一次

// 图像处理阈值
extern int16 threshold1;  // 阈值1
extern int16 threshold2;  // 阈值2
extern int16 threshold3;  // 阈值3
extern int16 threshold4;  // 阈值4

// 编码器数据
extern int32 encoder_R;    // 右编码器
extern int32 encoder_L;    // 左编码器
extern int32 encoder_R_d;  // 右编码器差值
extern int32 encoder_L_d;  // 左编码器差值
extern int32 encoder_R_last; // 右编码器上次值
extern int32 encoder_L_last; // 左编码器上次值

// 显示道路元素
void show_element(void)
{
    for (int16 i = 0; i < element_num; i++)
    {
        switch (roadelementType[i])
        {
        case straigh:
            ips200_show_string(72*(i%3),30*(i/3),"straigh"); // 直道
            break;
        case crossm:
            ips200_show_string(72*(i%3),30*(i/3)," crossM "); // 十字路口
            break;
        case crossl:
            ips200_show_string(72*(i%3),30*(i/3)," crossL "); // 左斜十字
            break;
        case crossr:
            ips200_show_string(72*(i%3),30*(i/3)," crossR "); // 右斜十字
            break;
        case islandl:
            ips200_show_string(72*(i%3),30*(i/3),"islandL"); // 左环岛
            break;
        case islandr:
            ips200_show_string(72*(i%3),30*(i/3),"islandR"); // 右环岛
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
            ips200_show_string(72*(i%3),30*(i/3)," stall "); // 停车
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

// 显示停止原因
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
    if(stopdebug==remotestop)          
    {
        ips200_show_string(0,20,"remote stop");
    }   
}

// 图像显示函数
void image_show()   {show_flag=true;}

// 清除PID参数
void PID_clear()
{
    PID_gyro.error0 = 0;
    PID_gyro.errorint = 0;
    PID_angle.error0 = 0;
    PID_speed.errorint = 0;
    PID_steer.error0 = 0;
    PID_steer.errorint = 0; 
}

// 启动小车
void start_the_car() { carmode = car_run_mode1;start_count=0; angle_init();PID_clear();}

// BLDC校准
void Calibrate_BLDC()   {carmode=Start_Calibrate;}

// 远程启动
void Remote_start()     {carmode =remote;start_count=0; angle_init();PID_clear();}

// 通用PID参数重置函数
void reset_pid_params(PID_t* pid, float maxout, float minout, float targ)
{
    pid->kp = 0;
    pid->ki = 0;
    pid->kd = 0;
    pid->kd2 = 0;
    pid->maxout = maxout;
    pid->minout = minout;
    pid->targ = targ;
}

// 各种PID参数重置函数
void pid_gyro_set0() { reset_pid_params(&PID_gyro, 5000, -5000, 0); ips200_show_string(0,180,"set 0 already"); }
void pid_angle_set0() { reset_pid_params(&PID_angle, 5000, -5000, 0); ips200_show_string(0,180,"set 0 already"); }
void pid_V_set0() { reset_pid_params(&PID_speed, 5000, -5000, 400); ips200_show_string(0,180,"set 0 already"); }
void pid_steer_set0() { reset_pid_params(&PID_steer, 5000, -5000, 0); ips200_show_string(0,180,"set 0 already"); }
void pid_BLDC_set0() { reset_pid_params(&PID_BLDC, 0, 0, 0); ips200_show_string(0,180,"set 0 already"); }
void pid_all_set0(){pid_gyro_set0();pid_angle_set0();pid_V_set0();pid_steer_set0();pid_BLDC_set0();}
void pid_BLDC_mode_set(){pid_gyro_set0();PID_gyro.maxout=100;PID_gyro.minout=0;pid_angle_set0();PID_angle.maxout=40;
    PID_angle.minout=0;pid_V_set0();PID_speed.maxout=100;PID_speed.minout=0;
    pid_steer_set0();PID_steer.maxout=100;PID_steer.minout=0;pid_BLDC_set0();
    PID_BLDC.maxout=0;PID_BLDC.minout=0;}
void pid_Bldc_param_set0(){bldc_param.basic_duty=400;bldc_param.encoder_p=1;bldc_param.max_output=600;bldc_param.min_output=-400;ips200_show_string(0,180,"set 0 already");}

// 代码加载函数
void codeload1(){}
void codeload2(){}
void codeload3(){}
void codeload4(){}

// 菜单结构体
MENU menu[] = 
{
    // 主菜单项
    {1,"start", 0, 20, {.param_float=&default_float}, catlog, NULL},
        {2,"car_go", 0, 20, {.param_float=&default_float}, function, start_the_car},
        {2,"Calibrate", 0, 40, {.param_float=&default_float}, function, Calibrate_BLDC},
        {2,"remote_start", 0, 60, {.param_float=&default_float}, function, Remote_start},

    // PID参数菜单
    {1, "pidparam", 0, 40, {.param_float=&default_float}, catlog, NULL},
        // 陀螺仪PID
        {2, "PID_gyro", 0, 20, {.param_float=&default_float}, catlog, NULL},
            {3, "kp", ips200_x_max-10 * 8, 20, {.param_float=&PID_gyro.kp}, param_float, NULL},
            {3, "ki", ips200_x_max-10 * 8, 40, {.param_float=&PID_gyro.ki}, param_float, NULL},
            {3, "kd", ips200_x_max-10 * 8, 60, {.param_float=&PID_gyro.kd}, param_float, NULL},
            {3, "maxout", ips200_x_max-10 * 8, 80, {.param_float=&PID_gyro.maxout}, param_float, NULL},
            {3, "minout", ips200_x_max-10 * 8, 100, {.param_float=&PID_gyro.minout}, param_float, NULL},
        // 角度PID
        {2, "PID_angle", 0, 40, {.param_float=&default_float}, catlog, NULL},
            {3, "kp", ips200_x_max-10 * 8, 20, {.param_float=&PID_angle.kp}, param_float, NULL},
            {3, "ki", ips200_x_max-10 * 8, 40, {.param_float=&PID_angle.ki}, param_float, NULL},
            {3, "kd", ips200_x_max-10 * 8, 60, {.param_float=&PID_angle.kd}, param_float, NULL},
            {3, "maxout", ips200_x_max-10 * 8, 80, {.param_float=&PID_angle.maxout}, param_float,NULL},
            {3, "minout", ips200_x_max-10 * 8, 100, {.param_float=&PID_angle.minout}, param_float, NULL},
        // 速度PID
        {2, "PID_Speed", 0, 60, {.param_float=&default_float}, catlog, NULL},
            {3, "kp", ips200_x_max-10 * 8, 20, {.param_float=&PID_speed.kp}, param_float, NULL},
            {3, "ki", ips200_x_max-10 * 8, 40, {.param_float=&PID_speed.ki}, param_float, NULL},
            {3, "kd", ips200_x_max-10 * 8, 60, {.param_float=&PID_speed.kd}, param_float, NULL},
            {3, "maxout", ips200_x_max-10 * 8, 80, {.param_float=&PID_speed.maxout}, param_float,NULL},
            {3, "minout", ips200_x_max-10 * 8, 100, {.param_float=&PID_speed.minout}, param_float, NULL},
            {3,"target", ips200_x_max-10 * 8, 120, {.param_float=&PID_speed.targ}, param_float, NULL},
        // 转向PID
        {2, "PID_steer", 0, 80, {.param_float=&default_float}, catlog, NULL},
            {3, "kp", ips200_x_max-10 * 8, 20, {.param_float=&PID_steer.kp}, param_float, NULL},
            {3, "ki", ips200_x_max-10 * 8, 40, {.param_float=&PID_steer.ki}, param_float, NULL},
            {3, "kd", ips200_x_max-10 * 8, 60, {.param_float=&PID_steer.kd}, param_float, NULL},
            {3, "kd2", ips200_x_max-10 * 8, 80, {.param_float=&PID_steer.kd2}, param_float, NULL},
            {3, "maxout", ips200_x_max-10 * 8, 100, {.param_float=&PID_steer.maxout}, param_float,NULL},
            {3, "minout", ips200_x_max-10 * 8, 120, {.param_float=&PID_steer.minout}, param_float, NULL},
        // BLDC参数
        {2, "BLDC_param", 0, 100, {.param_float=&default_float}, catlog, NULL},
            {3, "basic_duty", ips200_x_max-10 * 8, 20, {.param_int16=&bldc_param.basic_duty},param_int16, NULL},
            {3, "encoder_p", ips200_x_max-10 * 8, 40, {.param_int16=&bldc_param.encoder_p}, param_int16, NULL},
            {3, "max_output",ips200_x_max-10 * 8, 60, {.param_int16=&bldc_param.max_output}, param_int16, NULL},
            {3, "min_output",ips200_x_max-10 * 8, 80, {.param_int16=&bldc_param.min_output}, param_int16, NULL},
        // PID重置选项
        {2, "allset0", 0, 120, {.param_float=&default_float}, confirm, pid_all_set0},
        {2, "PID_gyro_set0", 0, 140, {.param_float=&default_float}, confirm, pid_gyro_set0},
        {2, "PID_angle_set0",0, 160, {.param_float=&default_float}, confirm, pid_angle_set0},
        {2, "PID_V_set0", 0, 180, {.param_float=&default_float}, confirm, pid_V_set0},
        {2, "PID_steer_set0",0, 200, {.param_float=&default_float}, confirm, pid_steer_set0},
        {2,"PID_BLDC_modeset0",0, 220, {.param_float=&default_float}, confirm, pid_BLDC_mode_set},
        {2,"PID_Bldc_paramset0",0, 240, {.param_float=&default_float}, confirm, pid_Bldc_param_set0},

    // 图像菜单
    {1, "image", 0, 60, {.param_float=&default_float}, catlog, NULL},
        {2, "ROLL_angle", 100, 20, {.param_float=&filtering_angle}, param_float_readonly, NULL},
        {2, "display", 0, 40, {.param_float=&default_float}, function, image_show},
        {2, "show_image", 0, 60, {.param_float=&default_float}, catlog, NULL},
            {3, "show_grayimage",180, 20, {.param_uint8=&image.gray_image}, chose1, NULL},
            {3, "show_ostuimage",180, 40, {.param_uint8=&image.OSTU_fast_image}, chose1, NULL},
            {3, "show_dev_image",180, 60, {.param_uint8=&image.OTSU_dev_image}, chose1, NULL},
        {2, "OTSU_threshold",0, 80, {.param_float=&default_float}, catlog, NULL},
            {3, "OTSU_up", 100, 20, {.param_int16=&threshold_up}, param_int16, NULL},
            {3, "OTSU_DOWN",100, 40, {.param_int16=&threshold_down}, param_int16, NULL},
            {3,"OTSU_perx", 100, 60, {.param_int16=&OTSU_calperxpage}, param_int16, NULL},
            {3,"threshold1",100, 80, {.param_int16=&threshold1}, param_int16, NULL},
            {3,"threshold2",100, 100, {.param_int16=&threshold2}, param_int16, NULL},
            {3,"threshold3",100, 120, {.param_int16=&threshold3}, param_int16, NULL},
            {3,"threshold4",100, 140, {.param_int16=&threshold4}, param_int16, NULL},
        {2, "image_point", 0, 100, {.param_float=&default_float}, catlog, NULL},
            {3, "crossroadall",0, 20, {.param_float=&default_float}, catlog, NULL},
                {4, "r_up_p",100, 20, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "r_down_p",100, 40, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "l_up_p",100, 60, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "l_down_p",100, 80, {.param_int16=&default_int}, param_int16_readonly, NULL},
            {3, "round", 0, 40, {.param_float=&default_float}, catlog, NULL},
        {2, "forwardsight", 0, 120, {.param_float=&default_float}, catlog, NULL},
            {3, "forwardsight1",150, 20, {.param_int16=&forwardsight}, param_int16, NULL},
            {3, "forwardsight2",150, 40, {.param_int16=&forwardsight2}, param_int16, NULL},
            {3, "forwardsight3",150, 60, {.param_int16=&forwardsight3}, param_int16, NULL},
    
    // 调试菜单
    {1, "debug", 0, 80, {.param_float=&default_float}, catlog, NULL},
        {2,"gyro_info", 0, 20, {.param_float=&default_float}, catlog, NULL},
            {3,"pit_intg", 100, 20, {.param_int16=&pitch_angle_integr_read}, param_int16_readonly, NULL},
            {3,"yaw_intg", 100, 40, {.param_int16=&yaw_angle_integr_read}, param_int16_readonly, NULL},
            {3,"roll_intg", 100, 60, {.param_int16=&roll_angle_integr_read}, param_int16_readonly, NULL},
            {3, "ROLL_angle",100, 80, {.param_float=&filtering_angle}, param_float_readonly, NULL},
            {3,"raw_gyro_x", 100, 100, {.param_int16=&raw_gyro_x}, param_int16_readonly, NULL},
            {3,"raw_gyro_y", 100, 120, {.param_int16=&raw_gyro_y}, param_int16_readonly, NULL},
            {3,"raw_gyro_z", 100, 140, {.param_int16=&raw_gyro_z}, param_int16_readonly, NULL},
            {3,"imu_gyro_x", 100, 160, {.param_int16=&imu660ra_gyro_x}, param_int16_readonly, NULL},
            {3,"left_encode",150, 180, {.param_int32=&encoder_L}, param_int32_readonly, NULL},
            {3,"right_encode",150, 200, {.param_int32=&encoder_R}, param_int32_readonly, NULL},
        {2,"remote_info", 0, 40, {.param_float=&default_float}, catlog, NULL},
            {3,"l_stick_UD", 150, 20, {.param_int16=&lora3a22_uart_transfer.joystick[1]}, param_int16_readonly, NULL},
            {3,"l_stick_LR", 150, 40, {.param_int16=&lora3a22_uart_transfer.joystick[0]}, param_int16_readonly, NULL},
            {3,"r_stick_UD", 150, 60, {.param_int16=&lora3a22_uart_transfer.joystick[3]}, param_int16_readonly, NULL},
            {3,"r_stick_LR", 150, 80, {.param_int16=&lora3a22_uart_transfer.joystick[2]}, param_int16_readonly, NULL},
            {3,"l_stickey", 150, 100, {.param_uint8=&lora3a22_uart_transfer.key[0]}, param_uint8_readonly, NULL},
            {3,"r_stickey", 150, 120, {.param_uint8=&lora3a22_uart_transfer.key[1]}, param_uint8_readonly, NULL},
            {3,"l_key", 150, 140, {.param_uint8=&lora3a22_uart_transfer.key[2]}, param_uint8_readonly, NULL},
            {3,"r_key", 150, 160, {.param_uint8=&lora3a22_uart_transfer.key[3]}, param_uint8_readonly, NULL},
            {3,"Lswitch_key1",150, 180, {.param_uint8=&lora3a22_uart_transfer.switch_key[0]}, param_uint8_readonly, NULL},
            {3,"Lswitch_key2",150, 200, {.param_uint8=&lora3a22_uart_transfer.switch_key[1]}, param_uint8_readonly, NULL},
            {3,"Rswitch_key1",150, 220, {.param_uint8=&lora3a22_uart_transfer.switch_key[2]}, param_uint8_readonly, NULL},
            {3,"Rswitch_key2",150, 240, {.param_uint8=&lora3a22_uart_transfer.switch_key[3]}, param_uint8_readonly, NULL},
        {2,"Encoder_info", 0, 60, {.param_float=&default_float}, catlog, NULL},
            {3,"left_encode",150, 20, {.param_int32=&encoder_L}, param_int32_readonly, NULL},
            {3,"right_encode",150, 40, {.param_int32=&encoder_R}, param_int32_readonly, NULL},
            {3,"left_encode_d",150, 60, {.param_int32=&encoder_L_d}, param_int32_readonly, NULL},
            {3,"right_encode_d",150, 80, {.param_int32=&encoder_R_d}, param_int32_readonly, NULL},
            {3,"left_encode_last",150, 100, {.param_int32=&encoder_L_last}, param_int32_readonly, NULL},
            {3,"right_encode_last",150, 120, {.param_int32=&encoder_R_last}, param_int32_readonly, NULL},
    
    // 道路元素菜单
    {1, "element", 0, 100, {.param_float=&default_float}, catlog, NULL},
        {2, "element_onoff", 0, 20, {.param_float=&default_float}, catlog, NULL},
            {3, "crossl", 100, 20, {.param_int16=&roadelement_onoff.crossl}, on_off, NULL},
            {3, "crossr", 100, 40, {.param_int16=&roadelement_onoff.crossr}, on_off, NULL},
            {3, "crossm", 100, 60, {.param_int16=&roadelement_onoff.crossm}, on_off, NULL},
            {3, "islandl", 100, 80, {.param_int16=&roadelement_onoff.islandl}, on_off, NULL},
            {3, "islandR", 100, 100, {.param_int16=&roadelement_onoff.islandr}, on_off, NULL},
            {3, "scurve", 100, 120, {.param_int16=&roadelement_onoff.scurve}, on_off, NULL},
            {3, "speedup", 100, 140, {.param_int16=&roadelement_onoff.speedup}, on_off, NULL},
            {3, "ramp", 100, 160, {.param_int16=&roadelement_onoff.ramp}, on_off, NULL},
            {3, "obstacle", 100, 180, {.param_int16=&roadelement_onoff.obstacle}, on_off, NULL},
        {2, "element_count",0, 40, {.param_float=&default_float}, catlog, NULL},
            {3, "straigh", 100, 20, {.param_int16=&roadelement_record.straigh}, param_int16_readonly, NULL},
            {3, "crossm", 100, 40, {.param_int16=&roadelement_record.crossm}, param_int16_readonly, NULL},
            {3, "crossl", 100, 60, {.param_int16=&roadelement_record.crossl}, param_int16_readonly, NULL},
            {3, "crossr", 100, 80, {.param_int16=&roadelement_record.crossr}, param_int16_readonly, NULL},
            {3, "islandl", 100, 100, {.param_int16=&roadelement_record.islandl}, param_int16_readonly, NULL},
            {3, "islandr", 100, 120, {.param_int16=&roadelement_record.islandr}, param_int16_readonly, NULL},
            {3, "scurve", 100, 140, {.param_int16=&roadelement_record.scurve}, param_int16_readonly, NULL},
            {3, "curve", 100, 160, {.param_int16=&roadelement_record.curve}, param_int16_readonly, NULL},
            {3, "speedup", 100, 180, {.param_int16=&roadelement_record.speedup}, param_int16_readonly, NULL},
            {3, "ramp", 100, 200, {.param_int16=&roadelement_record.ramp}, param_int16_readonly, NULL},
            {3, "obstacle", 100, 220, {.param_int16=&roadelement_record.obstacle}, param_int16_readonly, NULL},
            {3, "blackprotect",100, 240, {.param_int16=&roadelement_record.blackprotect}, param_int16_readonly, NULL},
            {3, "stall", 100, 260, {.param_int16=&roadelement_record.stall}, param_int16_readonly, NULL},
            {3, "zebra", 100, 280, {.param_int16=&roadelement_record.zebra}, param_int16_readonly, NULL},
        {2, "element_gothrough",0, 60, {.param_float=&default_float}, roadgothrough, NULL},
        {2, "record_clear", 0, 80, {.param_float=&default_float}, function, NULL},
    
    // Flash存储菜单
    {1, "flash", 0, 120, {.param_float=&default_float}, catlog, NULL},
        {2, "code_load", 100, 20, {.param_float=&default_float}, catlog, NULL},
            {3, "load1", 100, 20, {.param_float=&default_float}, confirm, NULL},
            {3, "load2", 100, 40, {.param_float=&default_float}, confirm, NULL},
            {3, "load3", 100, 60, {.param_float=&default_float}, confirm, NULL},
            {3, "load4", 100, 80, {.param_float=&default_float}, confirm, NULL},
        {2, "flash_load", 100, 40, {.param_float=&default_float}, catlog, NULL},
            {3, "load1", 100, 20, {.param_float=&default_float}, confirm, flash_load_config_1},
            {3, "load2", 100, 40, {.param_float=&default_float}, confirm, flash_load_config_2},
            {3, "load3", 100, 60, {.param_float=&default_float}, confirm, flash_load_config_3},
            {3, "load4", 100, 80, {.param_float=&default_float}, confirm, flash_load_config_4},
            {3, "loaddefault",100, 100, {.param_float=&default_float}, confirm, flash_load_config_default},
        {2, "flash_save", 100, 60, {.param_float=&default_float}, catlog, NULL},
            {3, "save1", 100, 20, {.param_float=&default_float}, confirm, flash_save_config_1},
            {3, "save2", 100, 40, {.param_float=&default_float}, confirm, flash_save_config_2},
            {3, "save3", 100, 60, {.param_float=&default_float}, confirm, flash_save_config_3},
            {3, "save4", 100, 80, {.param_float=&default_float}, confirm, flash_save_config_4},
        {2, "resetflash", 100, 80, {.param_float=&default_float}, confirm, flash_reset},
    
    // 设置菜单
    {1, "setting", 0, 140, {.param_float=&default_float}, catlog, NULL},
    {1, "end", 0, 0, {.param_float=&default_float}, catlog, NULL}
};

enum_Condition condition = NOACTION; // 菜单行为初始化为无动作

// 初始化菜单屏幕
void Menu_Screen_Init(void)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);    // 设置为白底黑字
    ips200_init(IPS200_TYPE_SPI);    // 初始化通信模式为SPI通信
}

union_param fast_show[7]=   //快速显示数据存储
{
{.param_float=&filtering_angle},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float}
};

// 快速显示函数
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

// 快速输出屏幕信息
void outputscreen_fast()
{
    if(show_flag==false&&current_state==1&&menu_Mode==normal)// 如果图像显示没有开启且在顶层菜单
    {    
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(0,160,"fast_show");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        // 要添加更多显示内容可以在这里添加

        display_fast(0, 60, 180, param_float,fast_show[0] , "ROLL_angle");
        display_fast(0, 160, 200, param_float,fast_show[1] , "STEER_out");
    }
}

// 辅助函数：将参数转换为int32类型用于显示
int32 convert_param_to_int32(MENU* menu_item)
{
    switch (menu_item->type)
    {
    case param_int8:
    case param_int8_readonly:
        return (int32)(*menu_item->param_union.param_int8);
    case param_uint8:
    case param_uint8_readonly:
        return (int32)(*menu_item->param_union.param_uint8);
    case param_int16:
    case param_int16_readonly:
        return (int32)(*menu_item->param_union.param_int16);
    case param_uint16:
    case param_uint16_readonly:
        return (int32)(*menu_item->param_union.param_uint16);
    case param_int32:
    case param_int32_readonly:
        return *menu_item->param_union.param_int32;
    case param_uint32:
    case param_uint32_readonly:
        return (int32)(*menu_item->param_union.param_uint32);
    default:
        return 0;
    }
}

// 辅助函数：根据参数类型调整参数值
void adjust_param_value(MENU* menu_item, int16 step_int, float step_float, bool increase)
{
    int16 multiplier = increase ? 1 : -1;

    switch (menu_item->type)
    {
    case param_int8:
        *menu_item->param_union.param_int8 += multiplier * step_int;
        break;
    case param_uint8:
        *menu_item->param_union.param_uint8 += multiplier * step_int;
        break;
    case param_int16:
        *menu_item->param_union.param_int16 += multiplier * step_int;
        break;
    case param_uint16:
        *menu_item->param_union.param_uint16 += multiplier * step_int;
        break;
    case param_int32:
        *menu_item->param_union.param_int32 += multiplier * step_int;
        break;
    case param_uint32:
        *menu_item->param_union.param_uint32 += multiplier * step_int;
        break;
    case param_float:
        *menu_item->param_union.param_float += multiplier * step_float;
        break;
    case param_double:
        *menu_item->param_union.param_double += multiplier * step_float;
        break;
    default:
        break;
    }
}

// 辅助函数：显示参数值
void display_param_value(MENU* menu_item)
{
    if(menu_item->type == param_float || menu_item->type == param_float_readonly)
    {
        ips200_show_float(menu_item->x, menu_item->y, *menu_item->param_union.param_float, 4, 3);
    }
    else if(menu_item->type >= param_int8 && menu_item->type <= param_uint32_readonly)
    {
        int32 temp = convert_param_to_int32(menu_item);
        ips200_show_int(menu_item->x, menu_item->y, temp, 5);
    }
    else if(menu_item->type == on_off || menu_item->type == chose1)
    {
        if (*menu_item->param_union.param_uint8)
        {
            ips200_show_string(menu_item->x, menu_item->y, "ON ");
        }
        else
        {
            ips200_show_string(menu_item->x, menu_item->y, "OFF");
        }
    }
}

// 屏幕输出函数
void output(void) 
{
    int16 target_priority=current_state-1;
    outputscreen_fast(); // 快速显示
    
    // 根据菜单模式显示不同内容
    if(menu_Mode==edit_int)     // 整型编辑模式
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);    // 设置为棕色背景
        ips200_show_string(100,0,"len_i");
        ips200_show_int(160,0,stepper_int[stepper_p_int],3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    // 恢复为白色背景
    }
    if(menu_Mode==edit_float)       // 浮点编辑模式
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);    // 设置为棕色背景
        ips200_show_string(100,0,"len_f");
        ips200_show_float(160,0,stepper_float[stepper_p_float],3,3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    // 恢复为白色背景
    }
    if(menu_Mode==edit_confirm)     // 确认模式
    {
        ips200_set_color(RGB565_ORANGE,RGB565_BLACK);
        ips200_show_string(20,0,"WARNING!WARNING!WARNING!");
        ips200_show_string(20,160,"PRESS BOTTON3 TO CONFIRM");
        ips200_show_string(20,300,"WARNING!WARNING!WARNING!");
        return;                     // 直接返回
    }
    if(menu_Mode==special_show_element1)// 显示道路元素模式
    {
        ips200_clear(); // 清屏
        show_element();
        return;
    }
    if(menu_Mode==stop_debug_display)// 停止调试显示模式
    {
        show_stopreason();
        return;                     // 直接返回
    }
    
    // 根据当前菜单层级显示不同内容
    if(target_priority==0)         // 顶层菜单
    {
        ips200_set_color(RGB565_DustyBlue, RGB565_BLACK);    // 设置为蓝色背景
        ips200_show_string(0,0,"menu");// 显示菜单标题
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    // 恢复为白色背景
        
        // 遍历菜单项
        for(int i=0;strcmp(menu[i].str, "end") != 0;i++)
        {
            if(menu[i].priority==1)
            {
                if(i==p)
                {
                    if(menu_Mode==normal)
                    {
                        ips200_show_string(0,menu[i].y,"->");// 显示指针
                        ips200_show_string(20,menu[i].y,menu[i].str);
                    }
                    else if(menu_Mode==edit_int||menu_Mode==edit_float)
                    {
                        ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);    // 设置为紫色画笔
                        ips200_show_string(0,menu[i].y,"->");// 显示指针
                        ips200_show_string(20,menu[i].y,menu[i].str);
                        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    // 恢复为白色画笔
                    }
                }
                else
                {
                    ips200_show_string(20,menu[i].y,menu[i].str);
                }
             }
        }
    }
    else if(target_priority!=0)// 非顶层菜单
    {
        ips200_set_color(RGB565_DustyBlue, RGB565_BLACK);    // 设置为蓝色背景
        ips200_show_string(0,0,menu[p_nearby].str);// 显示上级菜单标题
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    // 恢复为白色背景

        // 遍历菜单项
        for(int i=p_nearby+1;menu[i].priority!=target_priority;i++)
        {
            if(menu[i].priority==current_state)
            {
                if(i==p)
                {
                    if(menu_Mode==normal)       // 普通显示模式，显示箭头
                    {
                        ips200_show_string(0,menu[i].y,"->");// 显示指针
                        ips200_show_string(20,menu[i].y,menu[i].str);
                    }
                    else if(menu_Mode==edit_int||menu_Mode==edit_float) // 编辑模式下，改变颜色
                    {
                        ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);// 设置为紫色背景
                        ips200_show_string(0,menu[i].y,"->");// 显示指针
                        ips200_show_string(20,menu[i].y,menu[i].str);
                        ips200_set_color(RGB565_WHITE, RGB565_BLACK);// 恢复为白底黑字
                    }
                    // 显示参数值
                    display_param_value(&menu[i]);
                }
                else
                {
                    ips200_show_string(20,menu[i].y,menu[i].str);
                    // 显示参数值
                    display_param_value(&menu[i]);
                }
            }
        }
    }
}

// 菜单控制函数
void Menu_control(void)
{
    output(); // 输出屏幕内容
    status=0;
    condition = (enum_Condition)input; 
    // if(input)
    // {
    //     ips200_clear(); // 清屏
    // }
    
    // 根据输入条件执行不同操作
    switch (condition)
    {
    case NOACTION:
        break;
        
    case DOWN:
        if(menu_Mode==edit_int)  // 整型编辑模式
        {
            adjust_param_value(&menu[p], stepper_int[stepper_p_int], stepper_float[stepper_p_float], false);
            return;
        }
        if(menu_Mode==edit_float) // 浮点编辑模式
        {
            adjust_param_value(&menu[p], stepper_int[stepper_p_int], stepper_float[stepper_p_float], false);
            return;
        }
        // 向下移动菜单指针
        if (strcmp(menu[p].str, "end") != 0&&menu[p+1].priority>=menu[p].priority)
        {
            ips200_show_string(0,menu[p].y,"  ");// 清除原指针位置
            int temp=menu[p].priority;
            uint8 old_p=p;
            p++;
            while(menu[p].priority!=temp && strcmp(menu[p+1].str, "end") != 0)
            {
                p++;
                if(menu[p].priority<temp)
                {
                    p=old_p;    // 回到原位
                    ips200_show_string(0,180,"endorstart");
                    break;
                }
            } 
            if(strcmp(menu[p].str,"end")==0)
            {
                p=old_p;    // 回到原位
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
        if(menu_Mode==edit_int)         // 整型编辑模式，增加参数值
        {
            adjust_param_value(&menu[p], stepper_int[stepper_p_int], stepper_float[stepper_p_float], true);
            return;
        }
        if(menu_Mode==edit_float)       // 浮点编辑模式，增加参数值
        {
            adjust_param_value(&menu[p], stepper_int[stepper_p_int], stepper_float[stepper_p_float], true);
            return;
        }
        
        // 向上移动菜单指针
        if(p!=0&&menu[p-1].priority>=menu[p].priority)
        {
            ips200_show_string(0,menu[p].y,"  ");// 清除原指针位置
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
        // 进入子菜单
        if(menu[p+1].priority==current_state+1&&strcmp(menu[p+1].str,"end")!=0&&menu[p].type==catlog)
        {
            ips200_clear();                             // 清屏
            current_state++;
            p_nearby=p;
            p++;
            break;
        }
        
        // 菜单模式切换
        if(menu_Mode==edit_int)                         // 整型编辑模式
        {
            stepper_p_int=(stepper_p_int+1)%5;
            break;
        }
        if(menu_Mode==edit_float)                       // 浮点编辑模式
        {
            stepper_p_float=(stepper_p_float+1)%6;
            break;
        }
        if(menu_Mode==edit_confirm)                     // 确认模式
        {
            ips200_clear();                             // 清屏
            menu[p].Operate_default();
            menu_Mode=normal;
            break;
        }
        
        // 菜单项类型处理
        if(menu[p].type==param_float||menu[p].type==param_double)                   // 浮点参数                      
        {
            menu_Mode=edit_float;
            break;
        }
        if(menu[p].type==param_int16||menu[p].type==param_int32
            ||menu[p].type==param_int8||menu[p].type==param_uint16
            ||menu[p].type==param_uint32||menu[p].type==param_uint8)                 // 整型参数
        {
            menu_Mode=edit_int;
            break;
        }
        if (menu[p].type==confirm)                  // 确认项
        {
            menu_Mode=edit_confirm;
            break;
        }
        if(menu[p].type==on_off)                    // 开关项
        {
            *menu[p].param_union.param_int16=1-*menu[p].param_union.param_int16;
            break;
        }
        if(menu[p].type==function)                  // 功能项
        {
            menu[p].Operate_default();
            break;
        }
        if(menu[p].type==roadgothrough)             // 道路元素通过项
        {
            menu_Mode=special_show_element1;        // 进入显示道路元素模式
            break;
        }
        if(menu[p].type==param_float_readonly||menu[p].type==param_int16_readonly
            ||menu[p].type==param_int32_readonly||menu[p].type==param_uint16_readonly
            ||menu[p].type==param_uint32_readonly||menu[p].type==param_double_readonly
            ||menu[p].type==param_int8_readonly||menu[p].type==param_uint8_readonly)        // 只读参数
        {
            ips200_show_string(0,180,"error_readonly");
        }
        if(menu[p].type==chose1)        // 单选项目
        {
            for(int i=p_nearby+1;menu[i].priority!=current_state-1;i++)
            {
                if(menu[i].priority==current_state&&menu[i].type==chose1&&i!=p)
                {
                    *menu[i].param_union.param_uint8=0;
                }
                if(i==p)
                {
                    *menu[p].param_union.param_uint8=1;
                }
            }
            break;
        }
        break;
        
    case BACK:
        if(menu_Mode==edit_float||menu_Mode==edit_confirm||menu_Mode==edit_int
        ||menu_Mode==special_show_element1||menu_Mode==stop_debug_display) // 编辑模式下按返回键退出编辑模式
        {
            ips200_clear();                             // 清屏
            menu_Mode=normal;
            break;
        }
        if(menu[p].priority==1)
        {
            show_flag=false;
            flash_save_config_default();
            ips200_set_color(RGB565_PURPLE,RGB565_BLACK);
            ips200_show_string(0,300,"save default already");
            ips200_set_color(RGB565_WHITE,RGB565_BLACK);
            break;
        }
        if(menu[p].priority!=1)                 // 返回上一级菜单
        {
            ips200_clear();                     // 清屏
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
    input=0; // 清空输入
}
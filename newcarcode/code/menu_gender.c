#include "menu_gender.h"
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

//=============================================================================
// 全局变量定义
//=============================================================================

// 显示控制
bool showline = false;                          // 是否显示循迹线
bool show_flag = false;                         // 图像显示标志位
extern uint8 flag;

// 菜单导航变量
int current_menu_index = 0;                     // 当前菜单项索引（原p）
int parent_menu_index = 0;                      // 父级菜单项索引（原p_nearby）
int current_state = 1;                          // 当前菜单层级
uint8 input;                                    // 菜单输入操作
extern int status;

// 车辆运行控制
int32 start_count = 0;                          // 启动计时器
car_mode carmode = stop;                        // 车辆模式，默认停止
stop_debug stopdebug = normal_debug;            // 停车原因，默认正常
enum_menu_mode menu_Mode = normal;              // 菜单模式，默认普通模式
bool startbool = false;                         // 启动标志

// 参数编辑相关
int16 default_int = 0;                          // 默认整型值（用于占位）
float default_float = 0.0;                      // 默认浮点值（用于占位）
uint8 confirm_flag = false;                     // 确认标志

// 步进值设置
int stepper_int[STEPPER_INT_SIZE] = {1, 5, 10, 20, 50};                      // 整型步进值
float stepper_float[STEPPER_FLOAT_SIZE] = {0.01, 0.1, 1.0, 10.0, 100.0, 500.0}; // 浮点步进值
uint8 stepper_index_int = 0;                    // 整型步进值索引（原stepper_p_int）
uint8 stepper_index_float = 0;                  // 浮点步进值索引（原stepper_p_float）

//=============================================================================
// 赛道元素相关
//=============================================================================
enum_roadelementtypedef roadelementType[50] = {
    zebra, straigh, curve, straigh, ramp, crossr, straigh, speedup,
    obstacle, islandl, straigh, zebra
};
int16 element_num = 12;                         // 赛道元素数量

struct_roadelementypedef roadelement_onoff = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};  // 元素功能开关
struct_roadelementypedef roadelement_record = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 元素计数记录
struct_imageshowcase image = {0, 1, 0};         // 图像显示配置

//=============================================================================
// 外部变量声明
//=============================================================================

// 前瞻参数（来自track.h）
int16 forwardsight;                             // 默认前瞻
int16 forwardsight2;                            // 直道判断前瞻
int16 forwardsight3;                            // 弯道前瞻

// 平衡控制参数（来自balance.c）
extern float filtering_angle;                   // 滤波后的角度
extern int16 pitch_angle_integr_read;          // 俯仰角菜单显示积分量
extern int16 roll_angle_integr_read;           // 横滚角菜单显示积分量
extern int16 yaw_angle_integr_read;            // 偏航角菜单显示积分量
extern int16 raw_gyro_x;                        // 原始陀螺仪数据
extern int16 raw_gyro_y;
extern int16 raw_gyro_z;

// LoRa遥控（来自zf_device_lora3a22.c）
extern lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;

// PID控制器（来自PID.h）
extern PID_t PID_gyro;                          // 角速度环
extern PID_t PID_angle;                         // 角度环
extern PID_t PID_speed;                         // 速度环
extern PID_t PID_steer;                         // 转向环
extern PID_t PID_BLDC;                          // 负压风扇环

// BLDC参数（来自BLDC.h）
extern BLDC_Param bldc_param;

// 图像处理参数（来自photo_chuli.h）
int16 threshold_down = 100;                     // 大津阈值下限
int16 threshold_up = 200;                       // 大津阈值上限
int16 OTSU_calperxpage = 5;                     // 每x页图片计算一次大津法

extern int16 threshold1;                        // 阈值1
extern int16 threshold2;                        // 阈值2
extern int16 threshold3;                        // 阈值3
extern int16 threshold4;                        // 阈值4

// 编码器数据（来自encoder.c）
extern int32 encoder_R;                         // 右编码器
extern int32 encoder_L;                         // 左编码器
extern int32 encoder_R_d;                       // 右编码器差分值
extern int32 encoder_L_d;                       // 左编码器差分值
extern int32 encoder_R_last;                    // 右编码器上次值
extern int32 encoder_L_last;                    // 左编码器上次值

int32 speed;                                    // 速度变量

//=============================================================================
// 辅助函数 - 参数调整
//=============================================================================

// 增加整型参数
void add_intparam(int16* a)
{
    *a += stepper_int[stepper_index_int];
}

// 减少整型参数
void sub_intparam(int16* a)
{
    *a -= stepper_int[stepper_index_int];
}

// 增加浮点参数
void add_floatparam(float* a)
{
    *a += stepper_float[stepper_index_float];
}

// 减少浮点参数
void sub_floatparam(float* a)
{
    *a -= stepper_float[stepper_index_float];
}

//=============================================================================
// PID参数设置函数
//=============================================================================

// 清空PID积分项和误差
void PID_clear(void)
{
    PID_gyro.error0 = 0;
    PID_gyro.errorint = 0;
    PID_angle.error0 = 0;
    PID_speed.errorint = 0;
    PID_steer.error0 = 0;
    PID_steer.errorint = 0;
}

// 陀螺仪PID归零
void pid_gyro_set0(void)
{
    PID_gyro.kp = 0;
    PID_gyro.ki = 0;
    PID_gyro.kd = 0;
    PID_gyro.kd2 = 0;
    PID_gyro.maxout = 5000;
    PID_gyro.minout = -5000;
    ips200_show_string(0, 180, "set 0 already");
}

// 角度PID归零
void pid_angle_set0(void)
{
    PID_angle.kp = 0;
    PID_angle.ki = 0;
    PID_angle.kd = 0;
    PID_angle.kd2 = 0;
    PID_angle.maxout = 5000;
    PID_angle.minout = -5000;
    ips200_show_string(0, 180, "set 0 already");
}

// 速度PID归零
void pid_V_set0(void)
{
    PID_speed.kp = 0;
    PID_speed.ki = 0;
    PID_speed.kd = 0;
    PID_speed.kd2 = 0;
    PID_speed.maxout = 5000;
    PID_speed.minout = -5000;
    PID_speed.targ = 400;
    ips200_show_string(0, 180, "set 0 already");
}

// 转向PID归零
void pid_steer_set0(void)
{
    PID_steer.kp = 0;
    PID_steer.ki = 0;
    PID_steer.kd = 0;
    PID_steer.kd2 = 0;
    PID_steer.maxout = 5000;
    PID_steer.minout = -5000;
    ips200_show_string(0, 180, "set 0 already");
}

// BLDC PID归零
void pid_BLDC_set0(void)
{
    PID_BLDC.kp = 0;
    PID_BLDC.ki = 0;
    PID_BLDC.kd = 0;
    PID_BLDC.kd2 = 0;
    PID_BLDC.maxout = 0;
    PID_BLDC.minout = 0;
    ips200_show_string(0, 180, "set 0 already");
}

// 所有PID归零
void pid_all_set0(void)
{
    pid_gyro_set0();
    pid_angle_set0();
    pid_V_set0();
    pid_steer_set0();
    pid_BLDC_set0();
}

// BLDC模式设置（限制输出范围）
void pid_BLDC_mode_set(void)
{
    pid_gyro_set0();
    PID_gyro.maxout = 100;
    PID_gyro.minout = 0;

    pid_angle_set0();
    PID_angle.maxout = 40;
    PID_angle.minout = 0;

    pid_V_set0();
    PID_speed.maxout = 100;
    PID_speed.minout = 0;

    pid_steer_set0();
    PID_steer.maxout = 100;
    PID_steer.minout = 0;

    pid_BLDC_set0();
    PID_BLDC.maxout = 0;
    PID_BLDC.minout = 0;
}

// BLDC参数归零
void pid_Bldc_param_set0(void)
{
    bldc_param.basic_duty = 400;
    bldc_param.encoder_p = 1;
    bldc_param.max_output = 600;
    bldc_param.min_output = -400;
    ips200_show_string(0, 180, "set 0 already");
}

//=============================================================================
// 车辆控制函数
//=============================================================================

// 启动车辆
void start_the_car(void)
{
    carmode = car_run_mode1;
    start_count = 0;
    angle_init();
    PID_clear();
}

// BLDC电调校准
void Calibrate_BLDC(void)
{
    carmode = Start_Calibrate;
}

// 遥控模式启动
void Remote_start(void)
{
    carmode = remote;
    start_count = 0;
    angle_init();
    PID_clear();
}

// 图像显示开关
void image_show(void)
{
    show_flag = true;
}

//=============================================================================
// Flash参数加载函数（占位）
//=============================================================================
void codeload1(void) {}
void codeload2(void) {}
void codeload3(void) {}
void codeload4(void) {}

//=============================================================================
// 显示函数 - 赛道元素
//=============================================================================

/**
 * @brief 显示赛道元素序列
 */
void show_element(void)
{
    for (int16 i = 0; i < element_num; i++)
    {
        uint16 x = MENU_ELEMENT_COL_WIDTH * (i % 3);
        uint16 y = MENU_ELEMENT_ROW_HEIGHT * (i / 3);

        switch (roadelementType[i])
        {
        case straigh:
            ips200_show_string(x, y, "straigh");
            break;
        case crossm:
            ips200_show_string(x, y, " crossM ");
            break;
        case crossl:
            ips200_show_string(x, y, " crossL ");
            break;
        case crossr:
            ips200_show_string(x, y, " crossR ");
            break;
        case islandl:
            ips200_show_string(x, y, "islandL");
            break;
        case islandr:
            ips200_show_string(x, y, "islandR");
            break;
        case scurve:
            ips200_show_string(x, y, " S-curve");
            break;
        case curve:
            ips200_show_string(x, y, " curve ");
            break;
        case speedup:
            ips200_show_string(x, y, "speedUp");
            break;
        case ramp:
            ips200_show_string(x, y, "  ramp ");
            break;
        case obstacle:
            ips200_show_string(x, y, "obstacle");
            break;
        case blackprotect:
            ips200_show_string(x, y, "blkProt");
            break;
        case stall:
            ips200_show_string(x, y, " stall ");
            break;
        case zebra:
            ips200_show_string(x, y, "  zebra ");
            break;
        default:
            break;
        }

        // 显示箭头（除最后一个元素外）
        if (i != element_num - 1)
        {
            ips200_set_color(RGB565_ORANGE, RGB565_BLACK);
            ips200_show_string(x + 64, y, ">");
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        }
    }
}

//=============================================================================
// 显示函数 - 停车原因
//=============================================================================

/**
 * @brief 显示停车原因（优化为switch-case）
 */
void show_stopreason(void)
{
    ips200_show_string(0, 0, "stop reason:");

    switch (stopdebug)
    {
    case blackprotect_stop:
        ips200_show_string(0, 20, "blackprotect stop");
        break;

    case zebra_stop:
        ips200_show_string(0, 20, "zebra stop");
        break;

    case liftup_stop:
        ips200_show_string(0, 20, "liftup stop");
        break;

    case gyro_intrg_pitch_stop:
        ips200_show_string(0, 20, "gyro_intrg_pitch stop");
        break;

    case gyro_intrg_roll_stop:
        ips200_show_string(0, 20, "gyro_intrg_roll stop");
        break;

    case gyro_intrg_yaw_stop:
        ips200_show_string(0, 20, "gyro_intrg_yaw stop");
        break;

    case timer_count_stop:
        ips200_show_string(0, 20, "timer stop");
        break;

    case remotestop:
        ips200_show_string(0, 20, "remote stop");
        break;

    case normal_debug:
        ips200_show_string(0, 20, "normal running");
        break;

    default:
        ips200_show_string(0, 20, "unknown stop");
        break;
    }
}

//=============================================================================
// 辅助函数 - 参数类型转换和显示
//=============================================================================

/**
 * @brief 将各种整型参数转换为int32统一显示
 * @param menu_item 菜单项指针
 * @return int32类型的参数值
 */
static int32 convert_int_param_to_int32(MENU* menu_item)
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

/**
 * @brief 判断参数类型是否为整型（包括只读）
 */
static bool is_int_param(enum_function type)
{
    return (type >= param_int8 && type <= param_uint32_readonly);
}

/**
 * @brief 判断参数类型是否为浮点型（包括只读）
 */
static bool is_float_param(enum_function type)
{
    return (type == param_float || type == param_float_readonly ||
            type == param_double || type == param_double_readonly);
}

/**
 * @brief 显示单个菜单项的参数值
 * @param menu_item 菜单项指针
 * @param highlight 是否高亮显示
 */
static void display_menu_param(MENU* menu_item, bool highlight)
{
    // 浮点型参数显示
    if (is_float_param(menu_item->type))
    {
        ips200_show_float(menu_item->x, menu_item->y,
                         *menu_item->param_union.param_float,
                         PARAM_FLOAT_WIDTH, PARAM_FLOAT_PRECISION);
    }
    // 整型参数显示
    else if (is_int_param(menu_item->type))
    {
        int32 value = convert_int_param_to_int32(menu_item);
        ips200_show_int(menu_item->x, menu_item->y, value, PARAM_INT_WIDTH);
    }
    // 开关型参数显示
    else if (menu_item->type == on_off || menu_item->type == chose1)
    {
        if (!highlight)
        {
            // 非高亮模式
            ips200_show_string(menu_item->x, menu_item->y,
                             *menu_item->param_union.param_uint8 ? "ON" : "OFF");
        }
        else
        {
            // 高亮模式（使用橙色）
            ips200_set_color(RGB565_ORANGE, RGB565_BLACK);
            ips200_show_string(menu_item->x, menu_item->y,
                             *menu_item->param_union.param_uint8 ? "ON" : "OFF");
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        }
    }
}

/**
 * @brief 显示菜单项文本和参数
 * @param menu_item 菜单项指针
 * @param is_selected 是否被选中
 */
static void display_menu_item(MENU* menu_item, bool is_selected)
{
    if (is_selected)
    {
        // 显示选中标记
        if (menu_Mode == normal)
        {
            ips200_show_string(MENU_ARROW_X, menu_item->y, MENU_ARROW_STR);
            ips200_show_string(MENU_TEXT_X, menu_item->y, menu_item->str);
        }
        else if (menu_Mode == edit_int || menu_Mode == edit_float)
        {
            // 编辑模式使用洋红色高亮
            ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);
            ips200_show_string(MENU_ARROW_X, menu_item->y, MENU_ARROW_STR);
            ips200_show_string(MENU_TEXT_X, menu_item->y, menu_item->str);
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        }

        // 显示参数值
        display_menu_param(menu_item, true);
    }
    else
    {
        // 未选中项
        ips200_show_string(MENU_TEXT_X, menu_item->y, menu_item->str);
        display_menu_param(menu_item, false);
    }
}

//=============================================================================
// 快速显示功能
//=============================================================================

// 快速显示参数配置（在主菜单显示常用参数）
union_param fast_show[7] = {
    {.param_float = &filtering_angle},
    {.param_float = &PID_steer.out},
    {.param_float = &default_float},
    {.param_float = &default_float},
    {.param_float = &default_float},
    {.param_float = &default_float},
    {.param_float = &default_float}
};

/**
 * @brief 快速参数显示
 * @param X1 文本X坐标
 * @param X2 数值X坐标
 * @param Y Y坐标
 * @param type 参数类型
 * @param Union_param 参数联合体
 * @param str 参数名称
 */
void display_fast(int16 X1, int16 X2, int16 Y, enum_function type, union_param Union_param, char str[20])
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(X1, Y, str);

    switch (type)
    {
    case param_uint8:
        ips200_show_int(X2, Y, *Union_param.param_uint8, PARAM_INT_WIDTH);
        break;
    case param_int8:
        ips200_show_int(X2, Y, *Union_param.param_int8, PARAM_INT_WIDTH);
        break;
    case param_uint16:
        ips200_show_int(X2, Y, *Union_param.param_uint16, PARAM_INT_WIDTH);
        break;
    case param_int16:
        ips200_show_int(X2, Y, *Union_param.param_int16, PARAM_INT_WIDTH);
        break;
    case param_uint32:
        ips200_show_int(X2, Y, *Union_param.param_uint32, 10);
        break;
    case param_int32:
        ips200_show_int(X2, Y, *Union_param.param_int32, 10);
        break;
    case param_float:
        ips200_show_float(X2, Y, *Union_param.param_float, PARAM_FLOAT_WIDTH, PARAM_FLOAT_PRECISION);
        break;
    case param_double:
        ips200_show_float(X2, Y, *Union_param.param_double, PARAM_DOUBLE_WIDTH, PARAM_DOUBLE_PRECISION);
        break;
    default:
        break;
    }
}

/**
 * @brief 在主菜单显示快速监控信息
 */
void outputscreen_fast(void)
{
    // 仅在图像未显示、处于主菜单、普通模式时显示
    if (show_flag == false && current_state == 1 && menu_Mode == normal)
    {
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(0, 160, "fast_show");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);

        // 显示常用参数
        display_fast(0, 60, 180, param_float, fast_show[0], "ROLL_angle");
        display_fast(0, 160, 200, param_float, fast_show[1], "STEER_out");
    }
}

//=============================================================================
// 主显示函数
//=============================================================================

/**
 * @brief 屏幕输出主函数
 */
void output(void)
{
    int16 target_priority = current_state - 1;

    // 快速显示区域
    outputscreen_fast();

    // 显示步进值指示器
    if (menu_Mode == edit_int)
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);
        ips200_show_string(100, 0, "len_i");
        ips200_show_int(160, 0, stepper_int[stepper_index_int], 3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    }

    if (menu_Mode == edit_float)
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);
        ips200_show_string(100, 0, "len_f");
        ips200_show_float(160, 0, stepper_float[stepper_index_float], 3, 3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    }

    // 特殊显示模式
    if (menu_Mode == edit_confirm)
    {
        ips200_set_color(RGB565_ORANGE, RGB565_BLACK);
        ips200_show_string(20, 0, "WARNING!WARNING!WARNING!");
        ips200_show_string(20, 160, "PRESS BOTTON3 TO CONFIRM");
        ips200_show_string(20, 300, "WARNING!WARNING!WARNING!");
        return;
    }

    if (menu_Mode == special_show_element1)
    {
        show_element();
        return;
    }

    if (menu_Mode == stop_debug_display)
    {
        show_stopreason();
        return;
    }

    // 主菜单显示
    if (target_priority == 0)
    {
        ips200_set_color(RGB565_DustyBlue, RGB565_BLACK);
        ips200_show_string(0, 0, "menu");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);

        for (int i = 0; strcmp(menu[i].str, "end") != 0; i++)
        {
            if (menu[i].priority == 1)
            {
                display_menu_item(&menu[i], i == current_menu_index);
            }
        }
    }
    // 子菜单显示
    else
    {
        ips200_set_color(RGB565_DustyBlue, RGB565_BLACK);
        ips200_show_string(0, 0, menu[parent_menu_index].str);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);

        for (int i = parent_menu_index + 1; menu[i].priority != target_priority; i++)
        {
            if (menu[i].priority == current_state)
            {
                display_menu_item(&menu[i], i == current_menu_index);
            }
        }
    }
}

//=============================================================================
// 参数修改辅助函数
//=============================================================================

/**
 * @brief 修改整型参数（通用函数）
 * @param menu_item 菜单项指针
 * @param increment 是否增加（true=增加，false=减少）
 */
static void modify_int_param(MENU* menu_item, bool increment)
{
    int step = increment ? stepper_int[stepper_index_int] : -stepper_int[stepper_index_int];

    switch (menu_item->type)
    {
    case param_int8:
        *menu_item->param_union.param_int8 += step;
        break;
    case param_uint8:
        *menu_item->param_union.param_uint8 += step;
        break;
    case param_int16:
        *menu_item->param_union.param_int16 += step;
        break;
    case param_uint16:
        *menu_item->param_union.param_uint16 += step;
        break;
    case param_int32:
        *menu_item->param_union.param_int32 += step;
        break;
    case param_uint32:
        *menu_item->param_union.param_uint32 += step;
        break;
    default:
        break;
    }
}

/**
 * @brief 修改浮点参数（通用函数）
 * @param menu_item 菜单项指针
 * @param increment 是否增加（true=增加，false=减少）
 */
static void modify_float_param(MENU* menu_item, bool increment)
{
    float step = increment ? stepper_float[stepper_index_float] : -stepper_float[stepper_index_float];

    switch (menu_item->type)
    {
    case param_float:
        *menu_item->param_union.param_float += step;
        break;
    case param_double:
        *menu_item->param_union.param_double += step;
        break;
    default:
        break;
    }
}

//=============================================================================
// 菜单导航辅助函数
//=============================================================================

/**
 * @brief 查找下一个同级菜单项
 * @return true=找到，false=已到末尾
 */
static bool move_to_next_item(void)
{
    if (strcmp(menu[current_menu_index].str, "end") != 0 &&
        menu[current_menu_index + 1].priority >= menu[current_menu_index].priority)
    {
        int temp = menu[current_menu_index].priority;
        uint8 old_p = current_menu_index;
        current_menu_index++;

        while (menu[current_menu_index].priority != temp &&
               strcmp(menu[current_menu_index + 1].str, "end") != 0)
        {
            current_menu_index++;
            if (menu[current_menu_index].priority < temp)
            {
                current_menu_index = old_p;
                ips200_show_string(0, 180, "endorstart");
                return false;
            }
        }

        if (strcmp(menu[current_menu_index].str, "end") == 0)
        {
            current_menu_index = old_p;
            ips200_show_string(0, 180, "endorstart");
            return false;
        }
        return true;
    }
    else
    {
        ips200_show_string(0, 180, "endorstart");
        return false;
    }
}

/**
 * @brief 查找上一个同级菜单项
 * @return true=找到，false=已到开头
 */
static bool move_to_prev_item(void)
{
    if (current_menu_index != 0 &&
        menu[current_menu_index - 1].priority >= menu[current_menu_index].priority)
    {
        int temp = menu[current_menu_index].priority;
        current_menu_index--;
        while (menu[current_menu_index].priority != temp)
        {
            current_menu_index--;
        }
        return true;
    }
    else
    {
        ips200_show_string(0, 180, "endorstart");
        return false;
    }
}

//=============================================================================
// 菜单控制主函数
//=============================================================================

/**
 * @brief 菜单控制函数
 */
void Menu_control(void)
{
    output();
    status = 0;
    enum_Condition condition = (enum_Condition)input;

    if (input)
    {
        ips200_clear();
    }

    switch (condition)
    {
    case NOACTION:
        break;

    //=========================================================================
    // DOWN - 向下导航或减少参数
    //=========================================================================
    case DOWN:
        if (menu_Mode == edit_int)
        {
            modify_int_param(&menu[current_menu_index], false);
            return;
        }
        if (menu_Mode == edit_float)
        {
            modify_float_param(&menu[current_menu_index], false);
            return;
        }
        move_to_next_item();
        break;

    //=========================================================================
    // UP - 向上导航或增加参数
    //=========================================================================
    case UP:
        if (menu_Mode == edit_int)
        {
            modify_int_param(&menu[current_menu_index], true);
            return;
        }
        if (menu_Mode == edit_float)
        {
            modify_float_param(&menu[current_menu_index], true);
            return;
        }
        move_to_prev_item();
        break;

    //=========================================================================
    // CONFIRM - 确认操作
    //=========================================================================
    case CONFIRM:
        // 进入子目录
        if (menu[current_menu_index + 1].priority == current_state + 1 &&
            strcmp(menu[current_menu_index + 1].str, "end") != 0 &&
            menu[current_menu_index].type == catlog)
        {
            current_state++;
            parent_menu_index = current_menu_index;
            current_menu_index++;
            break;
        }

        // 编辑整型参数模式 - 切换步进值
        if (menu_Mode == edit_int)
        {
            stepper_index_int = (stepper_index_int + 1) % STEPPER_INT_SIZE;
            break;
        }

        // 编辑浮点参数模式 - 切换步进值
        if (menu_Mode == edit_float)
        {
            stepper_index_float = (stepper_index_float + 1) % STEPPER_FLOAT_SIZE;
            break;
        }

        // 确认模式 - 执行操作
        if (menu_Mode == edit_confirm)
        {
            ips200_clear();
            menu[current_menu_index].Operate_default();
            menu_Mode = normal;
            break;
        }

        // 进入浮点参数编辑模式
        if (menu[current_menu_index].type == param_float ||
            menu[current_menu_index].type == param_double)
        {
            menu_Mode = edit_float;
            break;
        }

        // 进入整型参数编辑模式
        if (menu[current_menu_index].type == param_int16 ||
            menu[current_menu_index].type == param_int32 ||
            menu[current_menu_index].type == param_int8 ||
            menu[current_menu_index].type == param_uint16 ||
            menu[current_menu_index].type == param_uint32 ||
            menu[current_menu_index].type == param_uint8)
        {
            menu_Mode = edit_int;
            break;
        }

        // 确认操作类型
        if (menu[current_menu_index].type == confirm)
        {
            menu_Mode = edit_confirm;
            break;
        }

        // 开关类型 - 切换状态
        if (menu[current_menu_index].type == on_off)
        {
            *menu[current_menu_index].param_union.param_int16 =
                1 - *menu[current_menu_index].param_union.param_int16;
            break;
        }

        // 函数类型 - 执行函数
        if (menu[current_menu_index].type == function)
        {
            menu[current_menu_index].Operate_default();
            break;
        }

        // 赛道元素通过情况显示
        if (menu[current_menu_index].type == roadgothrough)
        {
            menu_Mode = special_show_element1;
            break;
        }

        // 只读参数提示
        if (menu[current_menu_index].type == param_float_readonly ||
            menu[current_menu_index].type == param_int16_readonly ||
            menu[current_menu_index].type == param_int32_readonly ||
            menu[current_menu_index].type == param_uint16_readonly ||
            menu[current_menu_index].type == param_uint32_readonly ||
            menu[current_menu_index].type == param_double_readonly ||
            menu[current_menu_index].type == param_int8_readonly ||
            menu[current_menu_index].type == param_uint8_readonly)
        {
            ips200_show_string(0, 180, "error_readonly");
        }

        // 单选类型 - 只能选一个
        if (menu[current_menu_index].type == chose1)
        {
            for (int i = parent_menu_index + 1; menu[i].priority != current_state - 1; i++)
            {
                if (menu[i].priority == current_state && menu[i].type == chose1 && i != current_menu_index)
                {
                    *menu[i].param_union.param_uint8 = 0;
                }
                if (i == current_menu_index)
                {
                    *menu[current_menu_index].param_union.param_uint8 = 1;
                }
            }
            break;
        }
        break;

    //=========================================================================
    // BACK - 返回上级或退出编辑
    //=========================================================================
    case BACK:
        // 从编辑模式退出
        if (menu_Mode == edit_float || menu_Mode == edit_confirm ||
            menu_Mode == edit_int || menu_Mode == special_show_element1 ||
            menu_Mode == stop_debug_display)
        {
            menu_Mode = normal;
            break;
        }

        // 主菜单退出 - 保存默认配置
        if (menu[current_menu_index].priority == 1)
        {
            show_flag = false;
            flash_save_config_default();
            ips200_set_color(RGB565_PURPLE, RGB565_BLACK);
            ips200_show_string(0, 300, "save default already");
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);
            break;
        }

        // 返回上级菜单
        if (menu[current_menu_index].priority != 1)
        {
            current_state--;
            current_menu_index = parent_menu_index;
            while (menu[parent_menu_index].priority != current_state - 1)
            {
                parent_menu_index--;
            }
        }
        else
        {
            ips200_show_string(0, 180, "error");
        }
        break;

    default:
        break;
    }

    input = 0;
}

//=============================================================================
// 初始化函数
//=============================================================================

/**
 * @brief 菜单屏幕初始化
 */
void Menu_Screen_Init(void)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);  // 设置为白底黑字
    ips200_init(IPS200_TYPE_SPI);                  // 初始化SPI通讯模式
}

//=============================================================================
// 菜单结构体定义
//=============================================================================

enum_Condition condition = NOACTION;  // 菜单行为初始化

MENU menu[] =
{
    // 1级菜单：启动控制
    {1, "start",                  0,                   20, {.param_float=&default_float}, catlog,         NULL},
        {2, "car_go",             0,                   20, {.param_float=&default_float}, function,       start_the_car},
        {2, "Calibrate",          0,                   40, {.param_float=&default_float}, function,       Calibrate_BLDC},
        {2, "remote_start",       0,                   60, {.param_float=&default_float}, function,       Remote_start},

    // 1级菜单：PID参数
    {1, "pidparam",               0,                   40, {.param_float=&default_float}, catlog,         NULL},
        // 2级菜单：陀螺仪PID
        {2, "PID_gyro",           0,                   20, {.param_float=&default_float}, catlog,         NULL},
            {3, "kp",          MENU_PARAM_X_OFFSET,     20, {.param_float=&PID_gyro.kp}, param_float,   NULL},
            {3, "ki",          MENU_PARAM_X_OFFSET,     40, {.param_float=&PID_gyro.ki}, param_float,   NULL},
            {3, "kd",          MENU_PARAM_X_OFFSET,     60, {.param_float=&PID_gyro.kd}, param_float,   NULL},
            {3, "maxout",      MENU_PARAM_X_OFFSET,     80, {.param_float=&PID_gyro.maxout}, param_float,  NULL},
            {3, "minout",      MENU_PARAM_X_OFFSET,    100, {.param_float=&PID_gyro.minout}, param_float, NULL},
        // 2级菜单：角度PID
        {2, "PID_angle",          0,                   40, {.param_float=&default_float}, catlog,      NULL},
            {3, "kp",        MENU_PARAM_X_OFFSET,       20, {.param_float=&PID_angle.kp}, param_float, NULL},
            {3, "ki",        MENU_PARAM_X_OFFSET,       40, {.param_float=&PID_angle.ki}, param_float, NULL},
            {3, "kd",        MENU_PARAM_X_OFFSET,       60, {.param_float=&PID_angle.kd}, param_float, NULL},
            {3, "maxout",    MENU_PARAM_X_OFFSET,       80, {.param_float=&PID_angle.maxout}, param_float,NULL},
            {3, "minout",    MENU_PARAM_X_OFFSET,      100, {.param_float=&PID_angle.minout}, param_float, NULL},
        // 2级菜单：速度PID
        {2, "PID_Speed",          0,                   60, {.param_float=&default_float}, catlog,      NULL},
            {3, "kp",        MENU_PARAM_X_OFFSET,       20, {.param_float=&PID_speed.kp}, param_float, NULL},
            {3, "ki",        MENU_PARAM_X_OFFSET,       40, {.param_float=&PID_speed.ki}, param_float, NULL},
            {3, "kd",        MENU_PARAM_X_OFFSET,       60, {.param_float=&PID_speed.kd}, param_float, NULL},
            {3, "maxout",    MENU_PARAM_X_OFFSET,       80, {.param_float=&PID_speed.maxout}, param_float,NULL},
            {3, "minout",    MENU_PARAM_X_OFFSET,      100, {.param_float=&PID_speed.minout}, param_float, NULL},
            {3, "target",    MENU_PARAM_X_OFFSET,      120, {.param_float=&PID_speed.targ}, param_float, NULL},
        // 2级菜单：转向PID
        {2, "PID_steer",          0,                   80, {.param_float=&default_float}, catlog,      NULL},
            {3, "kp",        MENU_PARAM_X_OFFSET,       20, {.param_float=&PID_steer.kp}, param_float, NULL},
            {3, "ki",        MENU_PARAM_X_OFFSET,       40, {.param_float=&PID_steer.ki}, param_float, NULL},
            {3, "kd",        MENU_PARAM_X_OFFSET,       60, {.param_float=&PID_steer.kd}, param_float, NULL},
            {3, "kd2",       MENU_PARAM_X_OFFSET,       80, {.param_float=&PID_steer.kd2}, param_float, NULL},
            {3, "maxout",    MENU_PARAM_X_OFFSET,      100, {.param_float=&PID_steer.maxout}, param_float,NULL},
            {3, "minout",    MENU_PARAM_X_OFFSET,      120, {.param_float=&PID_steer.minout}, param_float, NULL},
        // 2级菜单：BLDC参数
        {2, "BLDC_param",         0,                  100, {.param_float=&default_float}, catlog,      NULL},
            {3, "basic_duty", MENU_PARAM_X_OFFSET,      20, {.param_int16=&bldc_param.basic_duty},param_int16, NULL},
            {3, "encoder_p", MENU_PARAM_X_OFFSET,       40, {.param_int16=&bldc_param.encoder_p}, param_int16, NULL},
            {3, "max_output",MENU_PARAM_X_OFFSET,       60, {.param_int16=&bldc_param.max_output}, param_int16, NULL},
            {3, "min_output",MENU_PARAM_X_OFFSET,       80, {.param_int16=&bldc_param.min_output}, param_int16, NULL},

        // 2级菜单：参数归零操作
        {2, "allset0",            0,                  120, {.param_float=&default_float}, confirm,     pid_all_set0},
        {2, "PID_gyro_set0",      0,                  140, {.param_float=&default_float}, confirm,     pid_gyro_set0},
        {2, "PID_angle_set0",     0,                  160, {.param_float=&default_float}, confirm,     pid_angle_set0},
        {2, "PID_V_set0",         0,                  180, {.param_float=&default_float}, confirm,     pid_V_set0},
        {2, "PID_steer_set0",     0,                  200, {.param_float=&default_float}, confirm,     pid_steer_set0},
        {2, "PID_BLDC_modeset0",  0,                  220, {.param_float=&default_float}, confirm,     pid_BLDC_mode_set},
        {2, "PID_Bldc_paramset0", 0,                  240, {.param_float=&default_float}, confirm,     pid_Bldc_param_set0},

    // 1级菜单：图像处理
    {1, "image",                  0,                   60, {.param_float=&default_float}, catlog,      NULL},
        {2, "ROLL_angle",       100,                   20, {.param_float=&filtering_angle}, param_float_readonly, NULL},
        {2, "display",            0,                   40, {.param_float=&default_float}, function,    image_show},
        {2, "show_image",         0,                   60, {.param_float=&default_float}, catlog,    NULL},
            {3, "show_grayimage",180,                  20, {.param_uint8=&image.gray_image}, chose1, NULL},
            {3, "show_ostuimage",180,                  40, {.param_uint8=&image.OSTU_fast_image}, chose1, NULL},
            {3, "show_dev_image",180,                  60, {.param_uint8=&image.OTSU_dev_image}, chose1, NULL},
        {2, "OTSU_threshold",     0,                   80, {.param_float=&default_float}, catlog, NULL},
            {3, "OTSU_up",      100,                   20, {.param_int16=&threshold_up}, param_int16, NULL},
            {3, "OTSU_DOWN",    100,                   40, {.param_int16=&threshold_down}, param_int16, NULL},
            {3, "OTSU_perx",    100,                   60, {.param_int16=&OTSU_calperxpage}, param_int16, NULL},
            {3, "threshold1",   100,                   80, {.param_int16=&threshold1}, param_int16, NULL},
            {3, "threshold2",   100,                  100, {.param_int16=&threshold2}, param_int16, NULL},
            {3, "threshold3",   100,                  120, {.param_int16=&threshold3}, param_int16, NULL},
            {3, "threshold4",   100,                  140, {.param_int16=&threshold4}, param_int16, NULL},
        {2, "image_point",        0,                  100, {.param_float=&default_float}, catlog, NULL},
            {3, "crossroadall",   0,                   20, {.param_float=&default_float}, catlog, NULL},
                {4, "r_up_p",   100,                   20, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "r_down_p", 100,                   40, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "l_up_p",   100,                   60, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "l_down_p", 100,                   80, {.param_int16=&default_int}, param_int16_readonly, NULL},
            {3, "round",          0,                   40, {.param_float=&default_float}, catlog, NULL},
        {2, "forwardsight",       0,                  120, {.param_float=&default_float}, catlog, NULL},
            {3, "forwardsight1", 150,                  20, {.param_int16=&forwardsight}, param_int16, NULL},
            {3, "forwardsight2", 150,                  40, {.param_int16=&forwardsight2}, param_int16, NULL},
            {3, "forwardsight3", 150,                  60, {.param_int16=&forwardsight3}, param_int16, NULL},

    // 1级菜单：调试信息
    {1, "debug",                  0,                   80, {.param_float=&default_float}, catlog, NULL},
        {2, "gyro_info",          0,                   20, {.param_float=&default_float}, catlog, NULL},
            {3, "pit_intg",     100,                   20, {.param_int16=&pitch_angle_integr_read}, param_int16_readonly, NULL},
            {3, "yaw_intg",     100,                   40, {.param_int16=&yaw_angle_integr_read}, param_int16_readonly, NULL},
            {3, "roll_intg",    100,                   60, {.param_int16=&roll_angle_integr_read}, param_int16_readonly, NULL},
            {3, "ROLL_angle",   100,                   80, {.param_float=&filtering_angle}, param_float_readonly, NULL},
            {3, "raw_gyro_x",   100,                  100, {.param_int16=&raw_gyro_x}, param_int16_readonly, NULL},
            {3, "raw_gyro_y",   100,                  120, {.param_int16=&raw_gyro_y}, param_int16_readonly, NULL},
            {3, "raw_gyro_z",   100,                  140, {.param_int16=&raw_gyro_z}, param_int16_readonly, NULL},
            {3, "imu_gyro_x",   100,                  160, {.param_int16=&imu660ra_gyro_x}, param_int16_readonly, NULL},
            {3, "left_encode",  150,                  180, {.param_int32=&encoder_L}, param_int32_readonly, NULL},
            {3, "right_encode", 150,                  200, {.param_int32=&encoder_R}, param_int32_readonly, NULL},
        {2, "remote_info",        0,                   40, {.param_float=&default_float}, catlog, NULL},
            {3, "l_stick_UD",   150,                   20, {.param_int16=&lora3a22_uart_transfer.joystick[1]}, param_int16_readonly, NULL},
            {3, "l_stick_LR",   150,                   40, {.param_int16=&lora3a22_uart_transfer.joystick[0]}, param_int16_readonly, NULL},
            {3, "r_stick_UD",   150,                   60, {.param_int16=&lora3a22_uart_transfer.joystick[3]}, param_int16_readonly, NULL},
            {3, "r_stick_LR",   150,                   80, {.param_int16=&lora3a22_uart_transfer.joystick[2]}, param_int16_readonly, NULL},
            {3, "l_stickey",    150,                  100, {.param_uint8=&lora3a22_uart_transfer.key[0]}, param_uint8_readonly, NULL},
            {3, "r_stickey",    150,                  120, {.param_uint8=&lora3a22_uart_transfer.key[1]}, param_uint8_readonly, NULL},
            {3, "l_key",        150,                  140, {.param_uint8=&lora3a22_uart_transfer.key[2]}, param_uint8_readonly, NULL},
            {3, "r_key",        150,                  160, {.param_uint8=&lora3a22_uart_transfer.key[3]}, param_uint8_readonly, NULL},
            {3, "Lswitch_key1", 150,                  180, {.param_uint8=&lora3a22_uart_transfer.switch_key[0]}, param_uint8_readonly, NULL},
            {3, "Lswitch_key2", 150,                  200, {.param_uint8=&lora3a22_uart_transfer.switch_key[1]}, param_uint8_readonly, NULL},
            {3, "Rswitch_key1", 150,                  220, {.param_uint8=&lora3a22_uart_transfer.switch_key[2]}, param_uint8_readonly, NULL},
            {3, "Rswitch_key2", 150,                  240, {.param_uint8=&lora3a22_uart_transfer.switch_key[3]}, param_uint8_readonly, NULL},
        {2, "Encoder_info",       0,                   60, {.param_float=&default_float}, catlog, NULL},
            {3, "left_encode",  150,                   20, {.param_int32=&encoder_L}, param_int32_readonly, NULL},
            {3, "right_encode", 150,                   40, {.param_int32=&encoder_R}, param_int32_readonly, NULL},
            {3, "left_encode_d",150,                   60, {.param_int32=&encoder_L_d}, param_int32_readonly, NULL},
            {3, "right_encode_d",150,                  80, {.param_int32=&encoder_R_d}, param_int32_readonly, NULL},
            {3, "left_encode_last",150,               100, {.param_int32=&encoder_L_last}, param_int32_readonly, NULL},
            {3, "right_encode_last",150,              120, {.param_int32=&encoder_R_last}, param_int32_readonly, NULL},

    // 1级菜单：赛道元素
    {1, "element",                0,                  100, {.param_float=&default_float}, catlog, NULL},
        {2, "element_onoff",      0,                   20, {.param_float=&default_float}, catlog, NULL},
            {3, "crossl",       100,                   20, {.param_int16=&roadelement_onoff.crossl}, on_off, NULL},
            {3, "crossr",       100,                   40, {.param_int16=&roadelement_onoff.crossr}, on_off, NULL},
            {3, "crossm",       100,                   60, {.param_int16=&roadelement_onoff.crossm}, on_off, NULL},
            {3, "islandl",      100,                   80, {.param_int16=&roadelement_onoff.islandl}, on_off, NULL},
            {3, "islandR",      100,                  100, {.param_int16=&roadelement_onoff.islandr}, on_off, NULL},
            {3, "scurve",       100,                  120, {.param_int16=&roadelement_onoff.scurve}, on_off, NULL},
            {3, "speedup",      100,                  140, {.param_int16=&roadelement_onoff.speedup}, on_off, NULL},
            {3, "ramp",         100,                  160, {.param_int16=&roadelement_onoff.ramp}, on_off, NULL},
            {3, "obstacle",     100,                  180, {.param_int16=&roadelement_onoff.obstacle}, on_off, NULL},
        {2, "element_count",      0,                   40, {.param_float=&default_float}, catlog, NULL},
            {3, "straigh",      100,                   20, {.param_int16=&roadelement_record.straigh}, param_int16_readonly, NULL},
            {3, "crossm",       100,                   40, {.param_int16=&roadelement_record.crossm}, param_int16_readonly, NULL},
            {3, "crossl",       100,                   60, {.param_int16=&roadelement_record.crossl}, param_int16_readonly, NULL},
            {3, "crossr",       100,                   80, {.param_int16=&roadelement_record.crossr}, param_int16_readonly, NULL},
            {3, "islandl",      100,                  100, {.param_int16=&roadelement_record.islandl}, param_int16_readonly, NULL},
            {3, "islandr",      100,                  120, {.param_int16=&roadelement_record.islandr}, param_int16_readonly, NULL},
            {3, "scurve",       100,                  140, {.param_int16=&roadelement_record.scurve}, param_int16_readonly, NULL},
            {3, "curve",        100,                  160, {.param_int16=&roadelement_record.curve}, param_int16_readonly, NULL},
            {3, "speedup",      100,                  180, {.param_int16=&roadelement_record.speedup}, param_int16_readonly, NULL},
            {3, "ramp",         100,                  200, {.param_int16=&roadelement_record.ramp}, param_int16_readonly, NULL},
            {3, "obstacle",     100,                  220, {.param_int16=&roadelement_record.obstacle}, param_int16_readonly, NULL},
            {3, "blackprotect", 100,                  240, {.param_int16=&roadelement_record.blackprotect}, param_int16_readonly, NULL},
            {3, "stall",        100,                  260, {.param_int16=&roadelement_record.stall}, param_int16_readonly, NULL},
            {3, "zebra",        100,                  280, {.param_int16=&roadelement_record.zebra}, param_int16_readonly, NULL},
        {2, "element_gothrough",  0,                   60, {.param_float=&default_float}, roadgothrough, NULL},
        {2, "record_clear",       0,                   80, {.param_float=&default_float}, function, NULL},

    // 1级菜单：Flash存储
    {1, "flash",                  0,                  120, {.param_float=&default_float}, catlog, NULL},
        {2, "code_load",        100,                   20, {.param_float=&default_float}, catlog, NULL},
            {3, "load1",        100,                   20, {.param_float=&default_float}, confirm, NULL},
            {3, "load2",        100,                   40, {.param_float=&default_float}, confirm, NULL},
            {3, "load3",        100,                   60, {.param_float=&default_float}, confirm, NULL},
            {3, "load4",        100,                   80, {.param_float=&default_float}, confirm, NULL},
        {2, "flash_load",       100,                   40, {.param_float=&default_float}, catlog, NULL},
            {3, "load1",        100,                   20, {.param_float=&default_float}, confirm, flash_load_config_1},
            {3, "load2",        100,                   40, {.param_float=&default_float}, confirm, flash_load_config_2},
            {3, "load3",        100,                   60, {.param_float=&default_float}, confirm, flash_load_config_3},
            {3, "load4",        100,                   80, {.param_float=&default_float}, confirm, flash_load_config_4},
            {3, "loaddefault",  100,                  100, {.param_float=&default_float}, confirm, flash_load_config_default},
        {2, "flash_save",       100,                   60, {.param_float=&default_float}, catlog, NULL},
            {3, "save1",        100,                   20, {.param_float=&default_float}, confirm, flash_save_config_1},
            {3, "save2",        100,                   40, {.param_float=&default_float}, confirm, flash_save_config_2},
            {3, "save3",        100,                   60, {.param_float=&default_float}, confirm, flash_save_config_3},
            {3, "save4",        100,                   80, {.param_float=&default_float}, confirm, flash_save_config_4},
        {2, "resetflash",       100,                   80, {.param_float=&default_float}, confirm, flash_reset},

    // 1级菜单：设置
    {1, "setting",                0,                  140, {.param_float=&default_float}, catlog, NULL},

    // 结束标记
    {1, "end",                    0,                    0, {.param_float=&default_float}, catlog, NULL}
};

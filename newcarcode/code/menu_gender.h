#ifndef MENU_GENDER_H__
#define MENU_GENDER_H__

#include "zf_common_headfile.h"

//=============================================================================
// 显示位置相关宏定义
//=============================================================================
#define MENU_PARAM_X_OFFSET     (240 - 10 * 8)  // 参数值显示X坐标偏移
#define MENU_ELEMENT_COL_WIDTH  72                        // 元素列宽度
#define MENU_ELEMENT_ROW_HEIGHT 30                        // 元素行高度
#define MENU_ARROW_STR          "->"                      // 菜单指针符号
#define MENU_ARROW_X            0                         // 箭头X坐标
#define MENU_TEXT_X             20                        // 文本X坐标

// 菜单步进数组大小
#define STEPPER_INT_SIZE        5                         // 整型步进值数量
#define STEPPER_FLOAT_SIZE      6                         // 浮点步进值数量

// 参数显示格式
#define PARAM_INT_WIDTH         5                         // 整型显示宽度
#define PARAM_FLOAT_WIDTH       4                         // 浮点数显示宽度
#define PARAM_FLOAT_PRECISION   3                         // 浮点数小数位数
#define PARAM_DOUBLE_WIDTH      10                        // 双精度显示宽度
#define PARAM_DOUBLE_PRECISION  6                         // 双精度小数位数

//=============================================================================
// 道路元素枚举
//=============================================================================
typedef enum
{
    straigh,         // 直道
    crossm,          // 正入十字路口
    crossl,          // 左斜入十字
    crossr,          // 右斜入十字
    islandl,         // 环岛左
    islandr,         // 环岛右
    scurve,          // S弯
    curve,           // 弯道
    speedup,         // 加速带
    ramp,            // 坡道
    obstacle,        // 障碍物
    blackprotect,    // 黑线保护
    stall,           // 堵转
    zebra            // 斑马线
} enum_roadelementtypedef;

//=============================================================================
// 菜单模式枚举
//=============================================================================
typedef enum
{
    normal,                  // 普通菜单显示
    edit_int,                // 编辑整型数据
    edit_float,              // 编辑浮点数据
    edit_confirm,            // 确认操作模式
    special_show_element1,   // 显示赛道元素流程图
    stop_debug_display       // 停车调试信息显示
} enum_menu_mode;

//=============================================================================
// 参数类型枚举
//=============================================================================
typedef enum
{
    // 可编辑整型参数
    param_int8,              // 有符号8位可编辑
    param_uint8,             // 无符号8位可编辑
    param_int16,             // 有符号16位可编辑
    param_uint16,            // 无符号16位可编辑
    param_int32,             // 有符号32位可编辑
    param_uint32,            // 无符号32位可编辑

    // 只读整型参数
    param_uint8_readonly,    // 无符号8位只读
    param_int8_readonly,     // 有符号8位只读
    param_uint16_readonly,   // 无符号16位只读
    param_int16_readonly,    // 有符号16位只读
    param_int32_readonly,    // 有符号32位只读
    param_uint32_readonly,   // 无符号32位只读

    // 浮点参数
    param_float,             // 单精度浮点可编辑
    param_double,            // 双精度浮点可编辑
    param_float_readonly,    // 单精度浮点只读
    param_double_readonly,   // 双精度浮点只读

    // 特殊类型
    confirm,                 // 确认操作
    catlog,                  // 目录/子菜单
    function,                // 函数调用
    on_off,                  // 开关
    chose1,                  // 单选（同级只选一个）
    roadgothrough            // 赛道元素通过情况
} enum_function;

//=============================================================================
// 道路元素结构体（用于计数和开关功能）
//=============================================================================
typedef struct
{
    int16 straigh;           // 直道
    int16 crossm;            // 正入十字路口
    int16 crossl;            // 左斜入十字
    int16 crossr;            // 右斜入十字
    int16 islandl;           // 环岛左
    int16 islandr;           // 环岛右
    int16 scurve;            // S弯
    int16 curve;             // 弯道
    int16 speedup;           // 加速带
    int16 ramp;              // 坡道
    int16 obstacle;          // 障碍物
    int16 blackprotect;      // 黑线保护
    int16 stall;             // 堵转
    int16 zebra;             // 斑马线
} struct_roadelementypedef;

//=============================================================================
// 菜单参数联合体（用于不同类型指针的统一管理）
//=============================================================================
typedef union
{
    int8*   param_int8;
    uint8*  param_uint8;
    int16*  param_int16;
    uint16* param_uint16;
    int32*  param_int32;
    uint32* param_uint32;
    float*  param_float;
    double* param_double;
} union_param;

//=============================================================================
// 菜单项结构体
//=============================================================================
typedef struct
{
    unsigned char priority;          // 菜单层级（1=主菜单，2=二级菜单，以此类推）
    char str[20];                    // 菜单项显示名称
    uint16 x;                        // 显示横坐标
    uint16 y;                        // 显示纵坐标
    union_param param_union;         // 参数指针联合体
    enum_function type;              // 参数类型
    void (*Operate_default)();       // 操作函数指针
} MENU;

//=============================================================================
// 图像显示配置结构体
//=============================================================================
typedef struct
{
    uint8 gray_image;                // 是否显示灰度图像
    uint8 OSTU_fast_image;           // 是否显示快速大津法图像
    uint8 OTSU_dev_image;            // 是否显示分块大津法图像
} struct_imageshowcase;

//=============================================================================
// 菜单操作枚举
//=============================================================================
typedef enum
{
    NOACTION,                        // 无操作
    DOWN,                            // 向下
    UP,                              // 向上
    CONFIRM,                         // 确认
    BACK                             // 返回
} enum_Condition;

//=============================================================================
// 车辆运行模式枚举
//=============================================================================
typedef enum
{
    stop,                            // 停止
    remote,                          // 遥控模式
    Start_Calibrate,                 // 开始BLDC校准
    Now_Calibrate,                   // 校准进行中
    car_run_mode1,                   // 自主驾驶模式1
    car_run_mode2                    // 自主驾驶模式2
} car_mode;

//=============================================================================
// 停车原因枚举（调试用）
//=============================================================================
typedef enum
{
    normal_debug,                    // 正常运行，无触发停车
    timer_count_stop,                // 定时停车（超时）
    blackprotect_stop,               // 黑线保护停车
    zebra_stop,                      // 斑马线停车
    liftup_stop,                     // 车辆抬升停车
    remotestop,                      // 远程控制停车
    gyro_intrg_pitch_stop,           // 陀螺仪俯仰角积分过大
    gyro_intrg_roll_stop,            // 陀螺仪横滚角积分过大
    gyro_intrg_yaw_stop              // 陀螺仪偏航角积分过大
} stop_debug;

//=============================================================================
// 函数声明
//=============================================================================
void output(void);
void Menu_control(void);
void Menu_Screen_Init(void);

#endif // MENU_GENDER_H__

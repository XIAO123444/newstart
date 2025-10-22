#ifndef FLASH_H__
#define FLASH_H__
#include "zf_common_headfile.h"

typedef enum // Flash参数类型
{
    FLASH_TYPE_UINT16 = 0,
    FLASH_TYPE_INT16,
    FLASH_TYPE_UINT32,
    FLASH_TYPE_INT32,
    FLASH_TYPE_FLOAT,
    FLASH_TYPE_DOUBLE,
    FLASH_TYPE_UINT8,
    FLASH_TYPE_INT8,
}Enum_flash_param;
typedef union 
{
    int16* param_int16;
    uint8* param_uint8;
    int8* param_int8;
    uint16* param_uint16;
    int32* param_int32;
    uint32* param_uint32;
    double* param_double;
    float* param_float;
}UNION_Flash_Param;

// Flash参数版本管理
#define FLASH_PARAM_COUNT       36      // 当前保存的参数总数（修改时更新此值）

// Flash参数索引定义（按模块分组）
// ====== 元数据 (99，0页) ======

#define FLASH_IDX_PARAM_COUNT   0      // 存储已保存的参数数量


// 函数声明
void flash_reset(void);
void flash_save_config(int16_t i);
void flash_save_config_default(void);
void flash_save_config_1(void);
void flash_save_config_2(void);
void flash_save_config_3(void);
void flash_save_config_4(void);
void flash_load_config(int16_t i);
void flash_load_config_default(void);
void flash_load_config_1(void);
void flash_load_config_2(void);
void flash_load_config_3(void);
void flash_load_config_4(void);
void save_flash_param_count();
void load_flash_param_count();


#endif
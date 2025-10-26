#include "flash.h"
#include "steer_pid.h"
#include "menu.h"
#include "pid.h"
#include "BLDC.h"

extern int speed;
extern int16 threshold_up;
extern int16 threshold_down;
extern int16 forwardsight;
extern int16 forwardsight2;
extern int16 forwardsight3;

extern PID_t PID_gyro;
extern PID_t PID_angle;
extern PID_t PID_speed;
extern PID_t PID_steer;
extern PID_t PID_BLDC;

extern BLDC_Param bldc_param;

Enum_flash_param flash_param_type[]=
{
    FLASH_TYPE_FLOAT,      // 0  PID_gyro.kp
    FLASH_TYPE_FLOAT,      // 1  PID_gyro.ki
    FLASH_TYPE_FLOAT,      // 2  PID_gyro.kd
    FLASH_TYPE_FLOAT,      // 3  PID_gyro.maxout
    FLASH_TYPE_FLOAT,      // 4  PID_gyro.minout

    FLASH_TYPE_FLOAT,      // 5  PID_angle.kp
    FLASH_TYPE_FLOAT,      // 6  PID_angle.ki
    FLASH_TYPE_FLOAT,      // 7  PID_angle.kd
    FLASH_TYPE_FLOAT,      // 8  PID_angle.maxout
    FLASH_TYPE_FLOAT,      // 9  PID_angle.minout

    FLASH_TYPE_FLOAT,      // 10 PID_speed.kp
    FLASH_TYPE_FLOAT,      // 11 PID_speed.ki
    FLASH_TYPE_FLOAT,      // 12 PID_speed.kd
    FLASH_TYPE_FLOAT,      // 13 PID_speed.maxout
    FLASH_TYPE_FLOAT,      // 14 PID_speed.minout
    FLASH_TYPE_FLOAT,      // 15 PID_speed.targ

    FLASH_TYPE_FLOAT,      // 16 PID_steer.kp
    FLASH_TYPE_FLOAT,      // 17 PID_steer.ki
    FLASH_TYPE_FLOAT,      // 18 PID_steer.kd
    FLASH_TYPE_FLOAT,      // 19 PID_steer.kd2
    FLASH_TYPE_FLOAT,      // 20 PID_steer.maxout
    FLASH_TYPE_FLOAT,      // 21 PID_steer.minout

    FLASH_TYPE_FLOAT,      // 22 PID_BLDC.kp
    FLASH_TYPE_FLOAT,      // 23 PID_BLDC.ki
    FLASH_TYPE_FLOAT,      // 24 PID_BLDC.kd
    FLASH_TYPE_FLOAT,      // 25 PID_BLDC.maxout
    FLASH_TYPE_FLOAT,      // 26 PID_BLDC.minout
    FLASH_TYPE_INT16,     // 27 bldc_param.basic_duty

    FLASH_TYPE_INT16,     // 28 bldc_param.encoder_p
    FLASH_TYPE_INT16,     // 29 bldc_param.max_output
    FLASH_TYPE_INT16,     // 30 bldc_param.min_output

    FLASH_TYPE_INT16,     // 31 threshold_up
    FLASH_TYPE_INT16,     // 32 threshold_down
    FLASH_TYPE_INT16,     // 33 forwardsight
    FLASH_TYPE_INT16,     // 34 forwardsight2
    FLASH_TYPE_INT16,     // 35 forwardsight3
};

UNION_Flash_Param flash_union_pointer[]=
{
    { .param_float = &PID_gyro.kp },         // 0
    { .param_float = &PID_gyro.ki },         // 1
    { .param_float = &PID_gyro.kd },         // 2
    { .param_float = &PID_gyro.maxout },     // 3
    { .param_float = &PID_gyro.minout },     // 4

    { .param_float = &PID_angle.kp },        // 5
    { .param_float = &PID_angle.ki },        // 6
    { .param_float = &PID_angle.kd },        // 7
    { .param_float = &PID_angle.maxout },    // 8
    { .param_float = &PID_angle.minout },    // 9

    { .param_float = &PID_speed.kp },        // 10
    { .param_float = &PID_speed.ki },        // 11
    { .param_float = &PID_speed.kd },        // 12
    { .param_float = &PID_speed.maxout },    // 13
    { .param_float = &PID_speed.minout },    // 14
    { .param_float = &PID_speed.targ },      // 15

    { .param_float = &PID_steer.kp },        // 16
    { .param_float = &PID_steer.ki },        // 17
    { .param_float = &PID_steer.kd },        // 18
    { .param_float = &PID_steer.kd2 },       // 19
    { .param_float = &PID_steer.maxout },    // 20
    { .param_float = &PID_steer.minout },    // 21

    { .param_float = &PID_BLDC.kp },         // 22
    { .param_float = &PID_BLDC.ki },         // 23
    { .param_float = &PID_BLDC.kd },         // 24
    { .param_float = &PID_BLDC.maxout },     // 25
    { .param_float = &PID_BLDC.minout },     // 26
    { .param_int16 = &bldc_param.basic_duty},//

    { .param_int16 = &bldc_param.encoder_p },// 28
    { .param_int16 = &bldc_param.max_output },// 29
    { .param_int16 = &bldc_param.min_output },// 30

    { .param_int16 = &threshold_up },        // 31
    { .param_int16 = &threshold_down },      // 32
    { .param_int16 = &forwardsight },        // 33
    { .param_int16 = &forwardsight2 },       // 34
    { .param_int16 = &forwardsight3 },       // 35
};
uint16 flash_param_count=0; //存储参数个数
uint16 Beacon_param_count=20;//最多打20个信标点
//-------------------------------------------------------------------------------------------------------------------
// 函数名称     ：flash_reset
// 功能说明     ：擦除所有配置页
// 参数说明     ：void
// 返回参数     ：void
// 使用示例     ：flash_reset();
// 备注信息     ：擦除所有配置槽位的Flash数据
//-------------------------------------------------------------------------------------------------------------------
void flash_reset(void)
{
    flash_erase_page(99, 0);  // 参数个数
    flash_erase_page(100, 0);  // 默认配置
    flash_erase_page(100, 1);  // 配置1
    flash_erase_page(100, 2);  // 配置2
    flash_erase_page(100, 3);  // 配置3
    flash_erase_page(101, 0);  // 配置4
}
//-------------------------------------------------------------------------------------------------------------------
// 函数名称     save_flash_param_count
// 功能说明     ：保存参数个数到Flash
// 参数说明     ：无
// 返回参数     ：void
// 使用示例     ：save_flash_param_count()
// 备注信息     ：
//   - 配置槽位映射：0→(100,0), 1→(100,1), 2→(100,2), 3→(100,3), 4→(101,0)
//   - 参数按模块分组，索引定义在flash.h中
//   - 自动保存版本号和参数计数，用于后续加载时的兼容性检查
//-------------------------------------------------------------------------------------------------------------------

void save_flash_param_count()
{
    if(flash_check(99,0))
    {
        flash_erase_page(99,0);
    }
    flash_buffer_clear();
    flash_union_buffer[FLASH_IDX_PARAM_COUNT].uint16_type = flash_param_count;
    // 写入Flash
    flash_write_page_from_buffer(99, 0);
}

void load_flash_param_count()
{
    flash_buffer_clear();
    flash_read_page_to_buffer(99,0);
    flash_param_count = flash_union_buffer[FLASH_IDX_PARAM_COUNT].uint16_type;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数名称     ：flash_save_config
// 功能说明     ：保存参数到Flash
// 参数说明     ：i - 配置槽位 (0=默认, 1-4=用户配置)
// 返回参数     ：void
// 使用示例     ：flash_save_config(0);
// 备注信息     ：
//   - 配置槽位映射：0→(100,0), 1→(100,1), 2→(100,2), 3→(100,3), 4→(101,0)
//   - 参数按模块分组，索引定义在flash.h中
//   - 自动保存版本号和参数计数，用于后续加载时的兼容性检查
//-------------------------------------------------------------------------------------------------------------------

void flash_save_config(int16 i)
{
    // 检查并擦除Flash页
    if(flash_check(100 + i/4, i%4))
    {
        flash_erase_page(100 + i/4, i%4);
    }

    flash_buffer_clear();

    for(int i =0;i<flash_param_count;i++)
    {
        switch(flash_param_type[i])
        {
            case FLASH_TYPE_FLOAT:
                flash_union_buffer[i].float_type = *(flash_union_pointer[i].param_float);
                break;
            case FLASH_TYPE_INT16:
                flash_union_buffer[i].int16_type = *(flash_union_pointer[i].param_int16);
                break;
            case FLASH_TYPE_UINT16:
                flash_union_buffer[i].uint16_type = *(flash_union_pointer[i].param_uint16);
                break;
            case FLASH_TYPE_INT32:
                flash_union_buffer[i].int32_type = *(flash_union_pointer[i].param_int32);
                break;
            case FLASH_TYPE_UINT32:
                flash_union_buffer[i].uint32_type = *(flash_union_pointer[i].param_uint32);
                break;
            default:
                break;
        }
    }

    for (int16 i = flash_param_count; i < FLASH_PARAM_COUNT; i++)
    {
        switch(flash_param_type[i])
        {
            case FLASH_TYPE_FLOAT:
                flash_union_buffer[i].float_type = 0.0f;
                *(flash_union_pointer[i].param_float) = 0.0f;
                break;
            case FLASH_TYPE_INT16:
                flash_union_buffer[i].int16_type = 0;
                *(flash_union_pointer[i].param_int16) = 0;
                break;
            case FLASH_TYPE_UINT16:
                flash_union_buffer[i].uint16_type = 0;
                *(flash_union_pointer[i].param_uint16) = 0;
                break;
            case FLASH_TYPE_INT32:
                flash_union_buffer[i].int32_type = 0;
                *(flash_union_pointer[i].param_int32) = 0;
                break;
            case FLASH_TYPE_UINT32:
                flash_union_buffer[i].uint32_type = 0;
                *(flash_union_pointer[i].param_uint32) = 0;
                break;
            default:
                break;
        }
    }
    
    // 写入Flash
    flash_write_page_from_buffer(100 + i/4, i%4);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     ：flash_load_config
// 功能说明     ：从Flash加载参数
// 参数说明     ：i - 配置槽位 (0=默认, 1-4=用户配置)
// 返回参数     ：void
// 使用示例     ：flash_load_config(0);
// 备注信息     ：
//   - 自动检测版本号和参数计数
//   - 如果Flash中保存的参数数量少于当前定义的参数数量，未保存的参数会被初始化为0
//   - 这样可以在添加新参数时不会导致程序卡死
//-------------------------------------------------------------------------------------------------------------------
void flash_load_config(int16 i)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(100 + i/4, i%4);
    for(int16 i=0;i<flash_param_count;i++)
    {
        switch(flash_param_type[i])
        {
            case FLASH_TYPE_FLOAT:
                *(flash_union_pointer[i].param_float) = flash_union_buffer[i].float_type;
                break;
            case FLASH_TYPE_INT16:
                *(flash_union_pointer[i].param_int16) = flash_union_buffer[i].int16_type;
                break;
            case FLASH_TYPE_UINT16:
                *(flash_union_pointer[i].param_uint16) = flash_union_buffer[i].uint16_type;
                break;
            case FLASH_TYPE_INT32:
                *(flash_union_pointer[i].param_int32) = flash_union_buffer[i].int32_type;
                break;
            case FLASH_TYPE_UINT32:
                *(flash_union_pointer[i].param_uint32) = flash_union_buffer[i].uint32_type;
                break;
            default:
                break;
        }
    }
    if(flash_param_count < FLASH_PARAM_COUNT)
    {
        for(int16 i=flash_param_count;i<FLASH_PARAM_COUNT;i++)
        {
            switch(flash_param_type[i])
            {
                case FLASH_TYPE_FLOAT:
                    *(flash_union_pointer[i].param_float) = 0.0f;
                    break;
                case FLASH_TYPE_INT16:
                    *(flash_union_pointer[i].param_int16) = 0;
                    break;
                case FLASH_TYPE_UINT16:
                    *(flash_union_pointer[i].param_uint16) = 0;
                    break;
                case FLASH_TYPE_INT32:
                    *(flash_union_pointer[i].param_int32) = 0;
                    break;
                case FLASH_TYPE_UINT32:
                    *(flash_union_pointer[i].param_uint32) = 0;
                    break;
                default:
                    break;
            }
        }
        flash_save_config(i);
    }
   
}

// 便捷函数：保存到不同配置槽位
void flash_save_init(void)
{
    if(flash_param_count < FLASH_PARAM_COUNT)
    {   
        flash_save_config(0);
        flash_save_config(1);
        flash_save_config(2);
        flash_save_config(3);
        flash_save_config(4);
        flash_param_count = FLASH_PARAM_COUNT;
        save_flash_param_count();
        return; 
    }
}


void flash_save_config_default(void) {flash_save_config(0);}
void flash_save_config_1(void) { flash_save_config(1); }
void flash_save_config_2(void) { flash_save_config(2); }
void flash_save_config_3(void) { flash_save_config(3); }
void flash_save_config_4(void) { flash_save_config(4); }

// 便捷函数：从不同配置槽位加载
// void flash_load_config_default(void) { flash_save_config_default();flash_load_config(0); }
void flash_load_config_default(void) {flash_load_config(0); }
void flash_load_config_1(void) { flash_load_config(1); }
void flash_load_config_2(void) { flash_load_config(2); }
void flash_load_config_3(void) { flash_load_config(3); }
void flash_load_config_4(void) { flash_load_config(4); }

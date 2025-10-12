#ifndef FLASH_H__
#define FLASH_H__
#include "zf_common_headfile.h"

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

#endif
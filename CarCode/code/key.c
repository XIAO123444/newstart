#include "key.h"

#define KEY1                    (E3 )
#define KEY2                    (E2 )
#define KEY3                    (E4 )
#define KEY4                    (E5 )
int8 offset=0;

uint32 key1_count;
uint32 key2_count;
uint32 key3_count;
uint32 key4_count;
uint32 key5_count;
uint32 key6_count;
uint8  key1_flag;
uint8  key2_flag;
uint8  key3_flag;
uint8  key4_flag;
uint8  key5_flag;
uint8  key6_flag;
uint32 count_time=100;
int status=0;
int alltime_count=0;
void Key_init(void)
{
	gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);                               // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);                               // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);                               // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);                               // 初始化 KEY4 输入 默认高电平 上拉输入
	pit_ms_init(TIM2_PIT, 1 );                                                   //硬件定时，周期100ms
    interrupt_set_priority(TIM2_IRQn, 1);
}
void Key_Scan(void)
{
    if(!gpio_get_level(KEY1)&&gpio_get_level(KEY2))
	{
		key1_flag=1;
		key1_count=0;
	}
	if(!gpio_get_level(KEY2)&&gpio_get_level(KEY1))
	{
		key2_flag=1;
		key2_count=0;
	}
	if(!gpio_get_level(KEY3)&&gpio_get_level(KEY4))
	{
		key3_flag=1;
		key3_count=0;
	}
	if(!gpio_get_level(KEY4)&&gpio_get_level(KEY3))
	{
		key4_flag=1;
		key4_count=0;
	}
    if(!gpio_get_level(KEY1)&&!gpio_get_level(KEY2))
	{
		key5_flag=1;
		key5_count=0;
	}
    if(!gpio_get_level(KEY4)&&!gpio_get_level(KEY3))
	{
		key6_flag=1;
		key6_count=0;
	}
}

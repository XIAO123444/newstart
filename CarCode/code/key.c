#include "key.h"

#define KEY1                    (E4 )
#define KEY2                    (E5 )
#define KEY3                    (E3 )
#define KEY4                    (E2 )
int8 offset=0;

uint32 key1_count;
uint32 key2_count;
uint32 key3_count;
uint32 key4_count;

uint8  key1_flag;
uint8  key2_flag;
uint8  key3_flag;
uint8  key4_flag;

uint32 count_time=100;
int alltime_count=0;
void Key_init(void)
{
	gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);                               // ��ʼ�� KEY1 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);                               // ��ʼ�� KEY2 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);                               // ��ʼ�� KEY3 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);                               // ��ʼ�� KEY4 ���� Ĭ�ϸߵ�ƽ ��������
	pit_ms_init(TIM7_PIT, 1 );                                                   //Ӳ����ʱ������100ms
    interrupt_set_priority(TIM7_IRQn, 1);
}
void Key_Scan(void)
{
    if(!gpio_get_level(KEY1))
	{
		key1_flag=1;
		key1_count=0;
	}
	if(!gpio_get_level(KEY2))
	{
		key2_flag=1;
		key2_count=0;
	}
	if(!gpio_get_level(KEY3))
	{
		key3_flag=1;
		key3_count=0;
	}
	if(!gpio_get_level(KEY4))
	{
		key4_flag=1;
		key4_count=0;
	}
}

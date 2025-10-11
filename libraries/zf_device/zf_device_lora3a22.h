
#ifndef CODE_ZF_DEVICE_LORA3A22_H_
#define CODE_ZF_DEVICE_LORA3A22_H_

#include "zf_common_headfile.h"

#define LORA3A22_UART_INDEX            (UART_6)              // ���崮��ң����ʹ�õĴ���
#define LORA3A22_UART_TX_PIN           (UART6_RX_C7)        // ң�������ջ���RX���� ���ӵ�Ƭ����TX����
#define LORA3A22_UART_RX_PIN           (UART6_TX_C6)        // ң�������ջ���TX���� ���ӵ�Ƭ����RX����
#define LORA3A22_UART_BAUDRATE         (115200)              // ָ�� lora3a22 ������ʹ�õĵĴ��ڲ�����
#define LORA3A22_UART_RTS_PIN          ( C13 )                             // ���ߴ��ڶ�Ӧģ��� RTS ����

#define LORA3A22_DATA_LEN              ( 18  )               // lora3a22֡��
#define LORA3A22_FRAME_STAR            ( 0XA3 )              // ֡ͷ��Ϣ



typedef struct
{
    uint8 head;                         // ֡ͷ
    uint8 sum_check;                    // ��У��

    int16 joystick[4];
	// joystick[0]:���ҡ������ֵ      
	// joystick[1]:���ҡ������ֵ
	// joystick[2]:�ұ�ҡ������ֵ      
	// joystick[3]:�ұ�ҡ������ֵ

    uint8 key[4];
	// ����1 �ɿ�0
    // key[0]-ҡ�����
    // key[1]-ҡ���ұ�
    // key[2]-���򰴼����
    // key[3]-���򰴼��ұ�

    uint8 switch_key[4];
    // switch_key[0]-��߲��뿪��_1
    // switch_key[1]-��߲��뿪��_2
    // switch_key[2]-�ұ߲��뿪��_1
    // switch_key[3]-�ұ߲��뿪��_2

}lora3a22_uart_transfer_dat_struct;


extern lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;
extern uint8   lora3a22_uart_data[LORA3A22_DATA_LEN];       // lora3a22����ԭʼ����
extern vuint8  lora3a22_finsh_flag;
extern vuint8  lora3a22_state_flag;                         // ң����״̬(1��ʾ�����������ʾʧ��)
extern uint16  lora3a22_response_time;


void lora3a22_uart_callback(void);

void lora3a22_init(void);

#endif

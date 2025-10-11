
#ifndef CODE_ZF_DEVICE_LORA3A22_H_
#define CODE_ZF_DEVICE_LORA3A22_H_

#include "zf_common_headfile.h"

#define LORA3A22_UART_INDEX            (UART_6)              // 定义串口遥控器使用的串口
#define LORA3A22_UART_TX_PIN           (UART6_RX_C7)        // 遥控器接收机的RX引脚 连接单片机的TX引脚
#define LORA3A22_UART_RX_PIN           (UART6_TX_C6)        // 遥控器接收机的TX引脚 连接单片机的RX引脚
#define LORA3A22_UART_BAUDRATE         (115200)              // 指定 lora3a22 串口所使用的的串口波特率
#define LORA3A22_UART_RTS_PIN          ( C13 )                             // 无线串口对应模块的 RTS 引脚

#define LORA3A22_DATA_LEN              ( 18  )               // lora3a22帧长
#define LORA3A22_FRAME_STAR            ( 0XA3 )              // 帧头信息



typedef struct
{
    uint8 head;                         // 帧头
    uint8 sum_check;                    // 和校验

    int16 joystick[4];
	// joystick[0]:左边摇杆左右值      
	// joystick[1]:左边摇杆上下值
	// joystick[2]:右边摇杆左右值      
	// joystick[3]:右边摇杆上下值

    uint8 key[4];
	// 按下1 松开0
    // key[0]-摇杆左边
    // key[1]-摇杆右边
    // key[2]-侧向按键左边
    // key[3]-侧向按键右边

    uint8 switch_key[4];
    // switch_key[0]-左边拨码开关_1
    // switch_key[1]-左边拨码开关_2
    // switch_key[2]-右边拨码开关_1
    // switch_key[3]-右边拨码开关_2

}lora3a22_uart_transfer_dat_struct;


extern lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;
extern uint8   lora3a22_uart_data[LORA3A22_DATA_LEN];       // lora3a22接收原始数据
extern vuint8  lora3a22_finsh_flag;
extern vuint8  lora3a22_state_flag;                         // 遥控器状态(1表示正常，否则表示失控)
extern uint16  lora3a22_response_time;


void lora3a22_uart_callback(void);

void lora3a22_init(void);

#endif

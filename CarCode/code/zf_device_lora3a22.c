#include "zf_device_lora3a22.h"

#pragma warning disable = 183

uint8   lora3a22_uart_data[LORA3A22_DATA_LEN]  = {0};               // 遥控器接收器原始数据


uint8   temp_lora3a22_uart_data[LORA3A22_DATA_LEN]  = {0};         // 遥控器接收器原始数据

vuint8  lora3a22_finsh_flag = 0;                                    // 表示成功接收到一帧遥控器数据
vuint8  lora3a22_state_flag = 1;                                    // 遥控器状态(1表示正常，否则表示失控)
uint16  lora3a22_response_time = 0;

lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;





//--------------------------------  -----------------------------------------------------------------------------------
// 函数简介     lora3a22串口回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     lora3a22_uart_callback();
// 备注信息     此函数需要在串口接收中断内进行调用
//-------------------------------------------------------------------------------------------------------------------
void lora3a22_uart_callback(void)
{
 static uint8 length = 0 ;
	uint8 i = 0;
	uint8 temp[8];

    uint8 parity_bit_sum  = 0, parity_bit  = 0;
    lora3a22_uart_data[length++] = uart_read_byte(LORA3A22_UART_INDEX);

    if((1 == length) && (LORA3A22_FRAME_STAR != lora3a22_uart_data[0]))
    {
        length =  0;
    }                                                             // 起始位判断

    if(LORA3A22_DATA_LEN <= length)                            	  // 数据长度判断
    {
        parity_bit = lora3a22_uart_data[1];
        lora3a22_uart_data[1] = 0;
        for(i = 0; i < LORA3A22_DATA_LEN; i ++)
        {
            parity_bit_sum += lora3a22_uart_data[i];
        }

        if (parity_bit_sum == parity_bit)                          // 和校验判断
        {

            lora3a22_finsh_flag = 1;
            lora3a22_state_flag = 1;
            lora3a22_response_time = 0;
            lora3a22_uart_data[1]= parity_bit;

			// 大小端不一样，需要交换
			temp[0] = lora3a22_uart_data[3];
			temp[1] = lora3a22_uart_data[2];
			temp[2] = lora3a22_uart_data[5];
			temp[3] = lora3a22_uart_data[4];
			temp[4] = lora3a22_uart_data[7];
			temp[5] = lora3a22_uart_data[6];
			temp[6] = lora3a22_uart_data[9];
			temp[7] = lora3a22_uart_data[8];
			                             
            // 将接收到的数据拷贝到结构体中
			memcpy((uint8*)&lora3a22_uart_data[2], temp, 8);
			
            memcpy(((uint8*)&lora3a22_uart_transfer), (uint8*)&lora3a22_uart_data, sizeof(lora3a22_uart_data));

        }
        else
        {
            lora3a22_finsh_flag = 0;
        }
        parity_bit_sum = 0;
        length = 0;
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     lora3a22初始化函数
// 参数说明     void
// 返回参数     void
// 使用示例     lora3a22_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

void lora3a22_init(void)
{

    uart_init(LORA3A22_UART_INDEX, LORA3A22_UART_BAUDRATE , LORA3A22_UART_RX_PIN, LORA3A22_UART_TX_PIN);


	
}

#include "zf_device_lora3a22.h"


uint8   lora3a22_uart_data[LORA3A22_DATA_LEN]  = {0};               // ң����������ԭʼ����

vuint8  lora3a22_finsh_flag = 0;                                    // ��ʾ�ɹ����յ�һ֡ң��������
vuint8  lora3a22_state_flag = 1;                                    // ң����״̬(1��ʾ�����������ʾʧ��)
uint16  lora3a22_response_time = 0;
static  fifo_struct                                     lora3a22_uart_fifo;
static  uint8                                           lora3a22_uart_buffer[WIRELESS_UART_BUFFER_SIZE];

static          uint8                                   lora3a22_uart_data1          = 0;
lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;

//--------------------------------  -----------------------------------------------------------------------------------
// �������     lora3a22���ڻص�����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     lora3a22_uart_callback();
// ��ע��Ϣ     �˺�����Ҫ�ڴ��ڽ����ж��ڽ��е���
//-------------------------------------------------------------------------------------------------------------------

void lora3a22_uart_callback(void )
{
    static uint8 length = 0 ;
    uint8  parity_bit_sum  = 0, parity_bit  = 0;

    lora3a22_uart_data[length++] = uart_read_byte(LORA3A22_UART_INDEX);

    if((1 == length) && (LORA3A22_FRAME_STAR != lora3a22_uart_data[0]))
    {
        length =  0;
    }                                                             // ��ʼλ�ж�

    if(LORA3A22_DATA_LEN <= length)                            	  // ���ݳ����ж�
    {
        parity_bit = lora3a22_uart_data[1];
        lora3a22_uart_data[1] = 0;
        for(int  i = 0; i < LORA3A22_DATA_LEN; i ++)
        {
            parity_bit_sum += lora3a22_uart_data[i];
        }

        if (parity_bit_sum == parity_bit)                          // ��У���ж�
        {
            lora3a22_finsh_flag = 1;
            lora3a22_state_flag = 1;
            lora3a22_response_time = 0;
            lora3a22_uart_data[1]= parity_bit;

            // �����յ������ݿ������ṹ����
            memcpy((uint8*)&lora3a22_uart_transfer, (uint8*)lora3a22_uart_data, \
            sizeof(lora3a22_uart_data));

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
// �������     lora3a22��ʼ������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     lora3a22_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------


void lora3a22_init(void)
{
    set_wireless_type(WIRELESS_UART, lora3a22_uart_callback);
    fifo_init(&lora3a22_uart_fifo, FIFO_DATA_8BIT, lora3a22_uart_buffer, WIRELESS_UART_BUFFER_SIZE);
    gpio_init(LORA3A22_UART_RTS_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    uart_init(LORA3A22_UART_INDEX, LORA3A22_UART_BAUDRATE , LORA3A22_UART_RX_PIN, LORA3A22_UART_TX_PIN);        // ��ʼ������
    uart_rx_interrupt(LORA3A22_UART_INDEX, 1);               // ʹ�ܴ��ڽ����ж�

}
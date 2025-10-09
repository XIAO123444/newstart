#include "zf_device_lora3a22.h"

#pragma warning disable = 183

uint8   lora3a22_uart_data[LORA3A22_DATA_LEN]  = {0};               // ң����������ԭʼ����


uint8   temp_lora3a22_uart_data[LORA3A22_DATA_LEN]  = {0};         // ң����������ԭʼ����

vuint8  lora3a22_finsh_flag = 0;                                    // ��ʾ�ɹ����յ�һ֡ң��������
vuint8  lora3a22_state_flag = 1;                                    // ң����״̬(1��ʾ�����������ʾʧ��)
uint16  lora3a22_response_time = 0;

lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;





//--------------------------------  -----------------------------------------------------------------------------------
// �������     lora3a22���ڻص�����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     lora3a22_uart_callback();
// ��ע��Ϣ     �˺�����Ҫ�ڴ��ڽ����ж��ڽ��е���
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
    }                                                             // ��ʼλ�ж�

    if(LORA3A22_DATA_LEN <= length)                            	  // ���ݳ����ж�
    {
        parity_bit = lora3a22_uart_data[1];
        lora3a22_uart_data[1] = 0;
        for(i = 0; i < LORA3A22_DATA_LEN; i ++)
        {
            parity_bit_sum += lora3a22_uart_data[i];
        }

        if (parity_bit_sum == parity_bit)                          // ��У���ж�
        {

            lora3a22_finsh_flag = 1;
            lora3a22_state_flag = 1;
            lora3a22_response_time = 0;
            lora3a22_uart_data[1]= parity_bit;

			// ��С�˲�һ������Ҫ����
			temp[0] = lora3a22_uart_data[3];
			temp[1] = lora3a22_uart_data[2];
			temp[2] = lora3a22_uart_data[5];
			temp[3] = lora3a22_uart_data[4];
			temp[4] = lora3a22_uart_data[7];
			temp[5] = lora3a22_uart_data[6];
			temp[6] = lora3a22_uart_data[9];
			temp[7] = lora3a22_uart_data[8];
			                             
            // �����յ������ݿ������ṹ����
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
// �������     lora3a22��ʼ������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     lora3a22_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------

void lora3a22_init(void)
{

    uart_init(LORA3A22_UART_INDEX, LORA3A22_UART_BAUDRATE , LORA3A22_UART_RX_PIN, LORA3A22_UART_TX_PIN);


	
}

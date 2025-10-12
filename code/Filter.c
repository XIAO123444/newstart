#include "Filter.h"
#include "math.h"

/*
------------------------------------------------------------------------------------------------------------------
�������     Ѱ����λ��
����˵��     int16�� x, y, z
���ز���     int16�� ��λ��
ʹ��ʾ��     median3(1, 2, 3) ���� 2
��ע��Ϣ     ��
-------------------------------------------------------------------------------------------------------------------
*/


int16 median3(int16 x, int16 y, int16 z) {
    if ((x <= y && y <= z) || (z <= y && y <= x)) return y;
    if ((y <= x && x <= z) || (z <= x && x <= y)) return x;
    return z;
}


/*------------------------------------------------------------------------------------------------------------------
�������     ��λ���˲���
����˵��     int16* data: ������������
            int16 size: �����С
���ز���     ��
ʹ��ʾ��     median_filter_inplace(data, size) �� data ���������λ���˲�
��ע��Ϣ     �ú�����ֱ���޸��������� data��    �˴�int16 size = sizeof(a)/sizeof(a[0]);
    ��Ҫȷ�������С���ڵ���3
-------------------------------------------------------------------------------------------------------------------
*/
void median_filter_inplace(int16* data, int16 size) {
    int16 prev_prev = data[0];
    int16 prev = data[1];
    
    for (int16 i = 2; i < size; i++) {
        int16 current = data[i];
        data[i] = median3(prev_prev, prev, current);
        prev_prev = prev;
        prev = current;
    }
}

/*------------------------------------------------------------------------------------------------------------------
�������     ��Ȩ�˲���
����˵��     int16* data: ������������
            int16 size: �����С
���ز���     ��
ʹ��ʾ��     weighted_filter_inplace(data, size) �� data ������м�Ȩ�˲�
��ע��Ϣ     �ú�����ֱ���޸��������� data
-------------------------------------------------------------------------------------------------------------------
*/
void weighted_filter_inplace(int16* data, int16 size) {
    float prev = (float)data[0];
    
    for (int16 i = 1; i < size; i++) {
        float current = data[i];
        float delta = fabsf(current - prev);
        float sum = fabsf(current) + fabsf(prev);
        float d = (sum > 0) ? delta / sum : 0;
        data[i] = (int16)(current*(1-d) + prev*d + 0.5);
        prev = data[i];
    }
}

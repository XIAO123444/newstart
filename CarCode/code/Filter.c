#include "Filter.h"
#include "math.h"

/*
------------------------------------------------------------------------------------------------------------------
函数简介     寻找中位数
参数说明     int16型 x, y, z
返回参数     int16型 中位数
使用示例     median3(1, 2, 3) 返回 2
备注信息     无
-------------------------------------------------------------------------------------------------------------------
*/


int16 median3(int16 x, int16 y, int16 z) {
    if ((x <= y && y <= z) || (z <= y && y <= x)) return y;
    if ((y <= x && x <= z) || (z <= x && x <= y)) return x;
    return z;
}


/*------------------------------------------------------------------------------------------------------------------
函数简介     中位数滤波器
参数说明     int16* data: 输入数据数组
            int16 size: 数组大小
返回参数     无
使用示例     median_filter_inplace(data, size) 对 data 数组进行中位数滤波
备注信息     该函数会直接修改输入数组 data，    此处int16 size = sizeof(a)/sizeof(a[0]);
    需要确保数组大小大于等于3
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
函数简介     加权滤波器
参数说明     int16* data: 输入数据数组
            int16 size: 数组大小
返回参数     无
使用示例     weighted_filter_inplace(data, size) 对 data 数组进行加权滤波
备注信息     该函数会直接修改输入数组 data
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

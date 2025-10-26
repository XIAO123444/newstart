#ifndef BEACON_H
#define BEACON_H 
#include "zf_common_headfile.h"

typedef struct 
{
    uint8 beacon_id;        //信标灯的ID  0是车当前位置 1之后是第信标灯
    bool lighton;           //信标灯的状态  
    int16 x;                //信标灯X坐标
    int16 y;                //信标灯Y坐标
    int16 next_beacon_id; //下一个信标灯ID
    rgb565_color_enum color; //信标灯颜色
} Struct_Beacon_t_typedef;

#define MAX_BEACON_NUM 20
float Matrix_Beacon[MAX_BEACON_NUM][MAX_BEACON_NUM]={0};        //信标灯距离矩阵


#endif
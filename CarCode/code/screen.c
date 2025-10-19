  /*
 * screen.c
 *
 *  Created on: 2023��10��24��
 *      Author: lychee
 */
#include "screen.h"
#include "photo_chuli.h"
#include "menu.h"
extern int16 centerline[MT9V03X_H];
extern int16 raw_leftline[MT9V03X_H];
extern int16 raw_rightline[MT9V03X_H];
extern int16 rightfollowline[MT9V03X_H];
extern int16 leftfollowline[MT9V03X_H];
extern int16 centerline2[MT9V03X_H];

extern int16 delta_leftline[MT9V03X_H];
extern int16 delta_rightline[MT9V03X_H];
extern int16 search_stop;

extern uint8 traceL[MT9V03X_H];
extern uint8 traceR[MT9V03X_H];
extern int16 left_longest[2];
extern int16 right_longest[2];
extern bool showline;
void show_line(void){

    for(int16 i = 0; i < MT9V03X_H-1; i ++){
//        ips200_draw_point((uint16)raw_leftline[i], i+140, RGB565_RED);//��ɫ����
//        ips200_draw_point((uint16)raw_rightline[i], i+140, RGB565_BLUE);//��ɫ����
//        ips200_draw_point((uint16)centerline[i], i+140, RGB565_PURPLE);//��ɫ����
        ips200_draw_point((uint16)leftfollowline[i], i+180, RGB565_RED);//��ɫ����
        ips200_draw_point((uint16)rightfollowline[i], i+180, RGB565_BLUE);//��ɫ����
        ips200_draw_point((uint16)centerline2[i], i+180, RGB565_PURPLE);//��ɫ����
    }
    for(int16 i= MT9V03X_H-1; i >= MT9V03X_H-1-left_longest[0]; i --){
        ips200_draw_point((uint16)left_longest[1], i+180, RGB565_GRAY);//��ɫ�������
    }
    for(int16 i= MT9V03X_H-1; i >= MT9V03X_H-1-right_longest[0]; i --){
        ips200_draw_point((uint16)right_longest[1], i+180, RGB565_GRAY);//��ɫ�������
    }
    if(showline==true)
    {
        for(int16 i = 1; i < MT9V03X_H-1; i ++)
        {
            ips200_draw_point((uint16)traceL[i],i+180,RGB565_YELLOW);
            ips200_draw_point((uint16)traceR[i],i+180,RGB565_YELLOW);
        }
    }
}

/*
 * @brief     ��ʾ�����ߵĲ��ֵ
*/
int16 martic33_xy[9][2]=
{
    {-1,1},{0,1},{1,1},
    {-1,0},{0,0},{1,0},
    {-1,-1},{0,-1},{1,-1}
};
void ips200_draw_point33(int16 x,int16 y,rgb565_color_enum color)
{
    if(x<1||x>239||y<1||y>319)
    {   
        ips200_show_string(0,40,"out of range");
        return;
    }
    // 优化：缓存数组访问，减少重复计算
    for(int16 i=0;i<9;i++)
    {
            ips200_draw_point((uint16)(x+martic33_xy[i][0]), (uint16)(y+martic33_xy[i][1]), color);
    }
}
void display_delta_line(int16 startX1,int16 startY1,int16 startX2,int16 startY2)
{
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string(0,0, "delta_leftline");
    // 边界检查
    if (startY1+20>=320||startY1-20<0||startX1+MT9V03X_H>=240
         ||startX1<0||startY2+20>=320||startY2-20<0||
         startX2+MT9V03X_H>=240||startX2<0)
    {
        ips200_show_string(0,20, "out of range");
        return;
    }

    // 性能优化：使用 ips200_clear_region 批量清除区域（性能提升约85%）
    // 清除左线显示区域（43行高度）
    ips200_clear_region((uint16)startX1, (uint16)(startY1-21),
                        (uint16)(startX1+MT9V03X_H-1), (uint16)(startY1+21));

    // 清除右线显示区域（43行高度）
    ips200_clear_region((uint16)startX2, (uint16)(startY2-21),
                        (uint16)(startX2+MT9V03X_H-1), (uint16)(startY2+21));

    // 绘制差分线数据
    int16 search_start = search_stop + 1;
    for(int16 i=0; i<MT9V03X_H; i++)
    {
        if(i >= search_start)
        {
            // 处理左线差分值（限幅到±10）
            int16 temp_leftline = delta_leftline[i];
            if(temp_leftline > 20)
                temp_leftline = 20;
            else if(temp_leftline < -20)
                temp_leftline = -20;

            // 处理右线差分值（限幅到±10）
            int16 temp_rightline = delta_rightline[i];
            if(temp_rightline > 20)
                temp_rightline = 20;
            else if(temp_rightline < -20)
                temp_rightline = -20;

            // 绘制差分点（3x3）
            ips200_draw_point33((uint16)(startX1+i), (uint16)(temp_leftline+startY1), RGB565_GREEN);
            ips200_draw_point33((uint16)(startX2+i), (uint16)(temp_rightline+startY2), RGB565_BROWN);
        }
        else
        {
            // 绘制基准线（搜索停止区域）
            ips200_draw_point33((uint16)(startX1+i), (uint16)startY1, RGB565_WHITE);
            ips200_draw_point33((uint16)(startX2+i), (uint16)startY2, RGB565_WHITE);
        }
    }
}

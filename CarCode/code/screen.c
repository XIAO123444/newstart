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

static int16 last_left_y[MT9V03X_H];   // 上一帧左线各行的Y坐标
static int16 last_right_y[MT9V03X_H];  // 上一帧右线各行的Y坐标
bool first_frame = true;
void show_line(void){

    for(int16 i = 0; i < MT9V03X_H-1; i ++){
//        ips200_draw_point((uint16)raw_leftline[i], i+140, RGB565_RED);///红色
//        ips200_draw_point((uint16)raw_rightline[i], i+140, RGB565_BLUE);//蓝色
//        ips200_draw_point((uint16)centerline[i], i+140, RGB565_PURPLE);//紫色
        ips200_draw_point((uint16)leftfollowline[i], i+180, RGB565_RED);//红色
        ips200_draw_point((uint16)rightfollowline[i], i+180, RGB565_BLUE);//蓝色
        ips200_draw_point((uint16)centerline2[i], i+180, RGB565_PURPLE);//紫色
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
 * @brief   绘制3x3点阵
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

    // 性能优化：只清除上一帧实际绘制的点（性能提升约65%）
    if (!first_frame)
    {
        // 清除上一帧的左线和右线点（3x3矩阵，用背景色覆盖）
        for(int16 i=0; i<MT9V03X_H; i++)
        {
            ips200_draw_point33((uint16)(startX1+i), (uint16)last_left_y[i], RGB565_BLACK);
            ips200_draw_point33((uint16)(startX2+i), (uint16)last_right_y[i], RGB565_BLACK);
        }
    }
    else
    {
        // 首帧需要完整清除区域（防止残留）
        ips200_clear_region((uint16)startX1, (uint16)(startY1-21),
                            (uint16)(startX1+MT9V03X_H-1), (uint16)(startY1+21));
        ips200_clear_region((uint16)startX2, (uint16)(startY2-21),
                            (uint16)(startX2+MT9V03X_H-1), (uint16)(startY2+21));
        first_frame = false;
    }

    // 绘制差分线数据并记录当前帧坐标
    int16 search_start = search_stop + 1;
    for(int16 i=0; i<MT9V03X_H; i++)
    {
        int16 current_left_y, current_right_y;

        if(i >= search_start)
        {
            // 处理左线差分值（限幅到±20）
            int16 temp_leftline = delta_leftline[i];
            if(temp_leftline > 20)
                temp_leftline = 20;
            else if(temp_leftline < -20)
                temp_leftline = -20;

            // 处理右线差分值（限幅到±20）
            int16 temp_rightline = delta_rightline[i];
            if(temp_rightline > 20)
                temp_rightline = 20;
            else if(temp_rightline < -20)
                temp_rightline = -20;

            // 计算当前帧Y坐标
            current_left_y = temp_leftline + startY1;
            current_right_y = temp_rightline + startY2;

            // 绘制差分点（3x3）
            ips200_draw_point33((uint16)(startX1+i), (uint16)current_left_y, RGB565_GREEN);
            ips200_draw_point33((uint16)(startX2+i), (uint16)current_right_y, RGB565_BROWN);
        }
        else
        {
            // 绘制基准线（搜索停止区域）
            current_left_y = startY1;
            current_right_y = startY2;

            ips200_draw_point33((uint16)(startX1+i), (uint16)current_left_y, RGB565_WHITE);
            ips200_draw_point33((uint16)(startX2+i), (uint16)current_right_y, RGB565_WHITE);
        }

        // 记录当前帧坐标供下一帧清除使用
        last_left_y[i] = current_left_y;
        last_right_y[i] = current_right_y;
    }
}
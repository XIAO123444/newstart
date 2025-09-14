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
extern int16 leftline[MT9V03X_H];
extern int16 rightline[MT9V03X_H];
extern int16 rightfollowline[MT9V03X_H];
extern int16 leftfollowline[MT9V03X_H];
extern int16 centerline2[MT9V03X_H];

extern uint8 traceL[MT9V03X_H];
extern uint8 traceR[MT9V03X_H];
extern int16 left_longest[2];
extern int16 right_longest[2];
extern bool showline;
void show_line(void){

    for(int16 i = 0; i < MT9V03X_H-1; i ++){
//        ips200_draw_point((uint16)leftline[i], i+140, RGB565_RED);//��ɫ����
//        ips200_draw_point((uint16)rightline[i], i+140, RGB565_BLUE);//��ɫ����
//        ips200_draw_point((uint16)centerline[i], i+140, RGB565_PURPLE);//��ɫ����
        ips200_draw_point((uint16)leftfollowline[i], i+140, RGB565_RED);//��ɫ����
        ips200_draw_point((uint16)rightfollowline[i], i+140, RGB565_BLUE);//��ɫ����
        ips200_draw_point((uint16)centerline2[i], i+140, RGB565_PURPLE);//��ɫ����
    }
    for(int16 i= MT9V03X_H-1; i >= MT9V03X_H-1-left_longest[0]; i --){
        ips200_draw_point((uint16)left_longest[1], i+140, RGB565_GRAY);//��ɫ�������
    }
    for(int16 i= MT9V03X_H-1; i >= MT9V03X_H-1-right_longest[0]; i --){
        ips200_draw_point((uint16)right_longest[1], i+140, RGB565_GRAY);//��ɫ�������
    }
    if(showline==true)
    {
        for(int16 i = 1; i < MT9V03X_H-1; i ++)
        {
            ips200_draw_point((uint16)traceL[i],i+140,RGB565_YELLOW);
            ips200_draw_point((uint16)traceR[i],i+140,RGB565_YELLOW);
        }
    }
}

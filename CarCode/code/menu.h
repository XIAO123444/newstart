#ifndef MENU_H__
#define MENU_H__


#include "zf_common_headfile.h"
typedef enum
{ straight,         //ֱ��
    crossm,         //����ʮ��·��
    crossl,         //��б��ʮ��
    crossr,         //��б��ʮ��
    islandl,            //������
    islandr,            //������
    scurve,             //S��
    curve,              //���
    speedup,            //���ٴ�            
    ramp,               //�µ�
    obstacle,           //�ϰ���
    blackprotect,       //���߱���
    stall,              //��ת              
    zebra               //������
    }  
enum_roadelementtypedef;
void output(void);
void car_init(void);
void Menu_control(void);
void Menu_Screen_Init(void);


#endif
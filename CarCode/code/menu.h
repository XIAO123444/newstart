#ifndef MENU_H__
#define MENU_H__


#include "zf_common_headfile.h"
//��·Ԫ��ö��
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
//�˵�ģʽö��
typedef enum
{   normal,
    edit_int,
    edit_float,
    edit_confirm,
    special_show_element1,          //����ͼ��ʾ����Ԫ��        
}enum_menu_mode;
//��·Ԫ�ؽṹ�壨���ڼ����Ϳ��ع��ܣ�
typedef struct  
{
    int16 straight;     //ֱ��
    int16 crossm;    //����ʮ��·��
    int16 crossl;   //��б��ʮ��
    int16 crossr;   //��б��ʮ��
    int16 islandl;       //������
    int16 islandr;      //������
    int16 scurve;      //S��
    int16 curve;        //���
    int16 speedup;     //���ٴ�
    int16 ramp;         //�µ�
    int16 obstacle;     //�ϰ���
    int16 blackprotect; //���߱���
    int16 stall;        //��ת
    int16 zebra;    //������
}struct_roadelementypedef;
//�˵��ṹ��{���ȼ������֣����꣬�������ݣ��������ݣ����ͣ�ö�٣���Ĭ��ִ�к���}
typedef struct 
{
    unsigned char priority;             //ҳ�����ȼ�
    char str[20];                       //����
    uint16 x;                           //��ʾ������
    uint16 y;                           //��ʾ������
    float *value_f;                      //��������
    int16 *value_i;                        //��������
    enum function
    {
        param_int,              //���Ϳɱ༭
        param_float,            //����ɱ༭
        confirm,                //ȷ��
        catlog,                 //Ŀ¼
        function,               //����
        param_int_readonly,     //����ֻ��
        param_float_readonly,   //����ֻ��
        on_off,                 //����
        roadgothrough           //����Ԫ��ͨ�����
    } type; //����:���Ͳ��������������Ŀ¼�� 
    void (*Operate_default)();          //Ĭ��ִ�к���

}MENU;

//�˵���Ϊö��
typedef enum {
    NOACTION,
    DOWN,
    UP,
    CONFIRM,
    BACK

}enum_Condition;  

void output(void);
void Menu_control(void);
void Menu_Screen_Init(void);


#endif
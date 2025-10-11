#ifndef MENU_H__
#define MENU_H__


#include "zf_common_headfile.h"
//��·Ԫ��ö��
typedef enum
{ straigh,         //ֱ��
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
{   normal,                         //��ͨ�˵���ʾ
    edit_int,                       //������������
    edit_float,                     //���ĸ�������
    edit_confirm,                   //ON/OFFȷ��
    special_show_element1,          //����ͼ��ʾ����Ԫ��      
    stop_debug_display              //ͣ��debug��ʾ
}enum_menu_mode;
typedef enum 
    {
        param_int,              //���Ϳɱ༭
        param_float,            //����ɱ༭
        confirm,                //ȷ��
        catlog,                 //Ŀ¼
        function,               //����
        param_int_readonly,     //����ֻ��
        param_float_readonly,   //����ֻ��
        on_off,                 //����
        chose1,                 //ֻѡһ�� 
        roadgothrough           //����Ԫ��ͨ�����
    }enum_function;
//��·Ԫ�ؽṹ�壨���ڼ����Ϳ��ع��ܣ�
typedef struct  
{
    int16 straigh;     //ֱ��
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
    enum_function type; //����:���Ͳ��������������Ŀ¼�� 
    void (*Operate_default)();          //Ĭ��ִ�к���

}MENU;
typedef struct 
{
    int16 gray_image;     //�Ҷ�ͼ��
    int16 OSTU_fast_image;//���ٴ��ͼ��
    int16 OTSU_dev_image;     //�ֿ���
}struct_imageshowcase;


//�˵���Ϊö��
typedef enum {
    NOACTION,
    DOWN,
    UP,
    CONFIRM,
    BACK

}enum_Condition;  

typedef enum 
{
    stop,       //ֹͣ
    remote,     //ң��
    Start_Calibrate,  //У׼
    Now_Calibrate,      //У׼��
    car_run_mode1,     //��ʻģʽ1������ƽ��
    car_run_mode2      //��ʻģʽ2��
}car_mode;

typedef enum 
{
    normal_debug,            //����,�޴���ͣ��
    timer_count_stop,             //��ʱͣ��
    blackprotect_stop,       //���߱���
    zebra_stop,              //������ͣ��
    liftup_stop,             //̧��
    gyro_intrg_pitch_stop,         //�����ǻ��ָ�����
    gyro_intrg_roll_stop,          //�����ǻ��ֺ����
    gyro_intrg_yaw_stop,           //�����ǻ���ƫ����
    
}stop_debug;

void output(void);
void Menu_control(void);
void Menu_Screen_Init(void);


#endif
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
        param_int8,             //�з���8λ�ɱ༭
        param_uint8,            //�޷���8λ�ɱ༭
        param_int16,              //���Ϳɱ༭
        param_uint16,           //�޷���16λ�ɱ༭
        param_int32,             //�з���32λ�ɱ༭
        param_uint32,          //�޷���32λ�ɱ༭
        param_uint8_readonly,    //�޷���8λֻ��
        param_int8_readonly,     //�з���8λֻ��
        param_uint16_readonly,   //�޷���16λֻ��
        param_int16_readonly,     //����ֻ��
        param_int32_readonly,     //�з���32λֻ��
        param_uint32_readonly,   //�޷���32λֻ��
        
        param_float,            //����ɱ༭
        param_double,           //˫���ȸ���ɱ༭
        param_float_readonly,   //����ֻ��
        param_double_readonly,   //˫���ȸ���ֻ��


        confirm,                //ȷ��
        catlog,                 //Ŀ¼
        function,               //����
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
//�˵������壬����ָ��ת��
typedef union 
{
    int16* param_int16;
    uint8* param_uint8;
    int8* param_int8;
    uint16* param_uint16;
    int32* param_int32;
    uint32* param_uint32;
    double* param_double;
    float* param_float;
}union_param;
//�˵��ṹ��{���ȼ������֣����꣬�������ݣ��������ݣ����ͣ�ö�٣���Ĭ��ִ�к���}
typedef struct 
{
    unsigned char priority;             //ҳ�����ȼ�
    char str[20];                       //����
    uint16 x;                           //��ʾ������
    uint16 y;                           //��ʾ������
    union_param param_union;
    enum_function type; //����:���Ͳ��������������Ŀ¼�� 
    void (*Operate_default)();          //Ĭ��ִ�к���

}MENU;
typedef struct 
{
    uint8 gray_image;     //�Ҷ�ͼ��
    uint8 OSTU_fast_image; //���ٴ��ͼ��
    uint8 OTSU_dev_image;  //�ֿ���
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
    timer_count_stop,        //��ʱͣ��
    blackprotect_stop,       //���߱���
    zebra_stop,              //������ͣ��
    liftup_stop,             //̧��
    remotestop,             //Զ�̿���ͣ��
    gyro_intrg_pitch_stop,         //�����ǻ��ָ�����
    gyro_intrg_roll_stop,          //�����ǻ��ֺ����
    gyro_intrg_yaw_stop,           //�����ǻ���ƫ����
    
}stop_debug;




void output(void);
void Menu_control(void);
void Menu_Screen_Init(void);


#endif
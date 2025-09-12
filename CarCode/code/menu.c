 #include "menu.h"
#include "pid_v.h"
#include "encoder.h"
#include "key.h"
#include "steer_pid.h"



#define ips200_x_max 240
#define ips200_y_max 320
int current_state=1;
int p=0;//��¼��ǰָ��
int p_nearby=0;//��¼������ָ��
uint8 input;
extern int status;
extern bool save_flag;      //�����־λ
extern bool start_flag;     //������־λ
extern bool stop_flag1;     //ͣ����־λ

bool show_flag=false;     //��ʾ��־λ

bool showline;
int16 threshold_up;  //�����ֵ����
int16 threshold_down; //�����ֵ����

int32 speed;
int32 forwardsight;
int32 forwardsight2;//ֱ���ж�ǰհ
int32 forwardsight3;//���ǰհ

float beilv;//����
//
//�˵�����
void show_image(void)
{
    show_flag=!show_flag; 
}

//�˵�����
enum menu_mode
{   normal,
    edit_int,
    edit_float
}menu_Mode=normal; //�˵�ģʽ
int stepper_int[5]={1,5,10,20,50};                  //���Ͳ���ֵ
float stepper_float[5]={0.01,0.1,1.0,10.0,100.0};   //�����Ͳ���ֵ
uint8 stepper_p_int=0;        //���Ͳ���ֵָ��
uint8 stepper_p_float=0;      //�����Ͳ���ֵָ��

//��ֹ��ָ��
int default_int=0;
float default_float=0.0;

//�Ӽ���װ����
void add_intparam(int* a)
{
    *a+=stepper_int[stepper_p_int];
}
void sub_intparam(int* a)
{
    *a-=stepper_int[stepper_p_int];
}
void add_floatparam(float* a)
{
    *a+=stepper_float[stepper_p_float];
}
void sub_floatparam(float* a)
{
    *a-=stepper_float[stepper_p_float];
}
//�Ӽ���װ����

//pid�ṹ�壬����
typedef struct 
{
    float p;
    float i;
    float d;
    float maxout;
}pidtest;

//��ʻ��Ԫ�ؼ�¼�ṹ��
typedef struct  
{
    uint8 straight;     //ֱ��
    uint8 crossm;    //����ʮ��·��
    uint8 crossl;   //��б��ʮ��
    uint8 crossr;   //��б��ʮ��
    uint8 islandl;       //������
    uint8 islandr;      //������
    uint8 scurve;      //S��
    uint8 curve;        //���
    uint8 speedup;     //���ٴ�
    uint8 ramp;         //�µ�
    uint8 obstacle;     //�ϰ���
    uint8 blackprotect; //���߱���
    uint8 stall;        //��ת
    uint8 zebra;    //������
}roadelement;
//ע��ע��ע��
//б��ʮ�������л�������ʮ�ֲ�����������Ԫ�ء�

enum roadelement_type
{   
    straight,
    crossm,
    crossl,
    crossr,
    islandl,
    islandr,
    scurve,
    curve,
    speedup,
    ramp,
    obstacle,
    blackprotect,
    stall,
    zebra
}roadelementType;//����Ԫ������

//�˵�����

//pid
pidtest pid_vtest=          {1.0,0.0,0.0,0.0};
pidtest pid_steer_straight= {0.0,0.0,0.0,0.0};
pidtest pid_steer_curve=    {0.0,0.0,0.0,0.0};
roadelement roadelement_record1={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
roadelement roadelement_record2={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
roadelement roadelement_record3={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
roadelement roadelement_record4={0,0,0,0,0, 0,0,0,0,0,0,0,0,0};
//���
int16 threshold_down=200;
int16 threshold_up=100;
//

//�˵��ṹ��
typedef struct 
{
    unsigned char priority;             //ҳ�����ȼ�
    char str[20];                       //����
    uint16 x;                           //��ʾ������
    uint16 y;                           //��ʾ������
    float *value_f;                      //��������
    int *value_i;                        //��������
    enum function
    {
        param_int,
        param_float,
        catlog,
        function,
        param_int_readonly,
        param_float_readonly,
        on_off,
    } type; //����:���Ͳ��������������Ŀ¼�� 
    void (*Operate_default)();          //Ĭ��ִ�к���

}MENU;
 

MENU menu[]=
{
    {1,"pidparam",0,20,&default_float,&default_int,catlog,NULL},
    //���ٶȻ�
    //�ǶȻ�
    //�ٶȻ�
        {2,"velocity_pid",0,20,&default_float,&default_int,catlog,NULL},
            {3,"p",      ips200_x_max-10 * 7, 20,  &pid_vtest.p, &default_int, param_float,      NULL},
            {3,"i",      ips200_x_max-10 * 7, 40,  &pid_vtest.i, &default_int, param_float,      NULL},
            {3,"d",      ips200_x_max-10 * 7, 60,  &pid_vtest.d, &default_int, param_float,      NULL},
            {3,"maxout", ips200_x_max-10 * 7, 80,  &pid_vtest.maxout, &default_int, param_float, NULL},
        {2,"steer_pid",0,40,&default_float,&default_int,catlog,NULL},
            {3,"p_s", ips200_x_max-10 * 7, 20,  &pid_steer_straight.p, &default_int, param_float, NULL},
            {3,"i_s", ips200_x_max-10 * 7, 40,  &pid_steer_straight.i, &default_int, param_float, NULL},
            {3,"d_s", ips200_x_max-10 * 7, 60,  &pid_steer_straight.d, &default_int, param_float, NULL},
            {3,"max_s", ips200_x_max-10 * 7, 80,  &pid_steer_straight.maxout, &default_int, param_float, NULL},
            {3,"p_c", ips200_x_max-10 * 7, 100,  &pid_steer_curve.p, &default_int, param_float, NULL},
            {3,"i_c", ips200_x_max-10 * 7, 120,  &pid_steer_curve.i, &default_int, param_float, NULL},
            {3,"d_c", ips200_x_max-10 * 7, 140,  &pid_steer_curve.d, &default_int, param_float, NULL},
            {3,"maxo", ips200_x_max-10 * 7, 160,  &pid_steer_curve.maxout, &default_int, param_float, NULL},
    {1,"image",0,40,&default_float,&default_int,catlog,NULL},
        {2,"show_image",0,20,&default_float,&default_int,function,NULL},
        {2,"OTSU_threshold",0,40,&default_float,&default_int,catlog,NULL},
            {3,"OTSU_up",100,20,&default_float,&default_int    ,param_int,NULL},
            {3,"OTSU_DOWN",100,40,&default_float,&default_int,param_int,NULL},
        {2,"image_point",0,60,&default_float,&default_int,catlog,NULL},
            {3,"crossroadall",0,20,&default_float,&default_int,catlog,NULL},       //ʮ�ִ���
                {4,"r_up_p",100,20,&default_float,&default_int,param_int_readonly,NULL},
                {4,"r_down_p",100,40,&default_float,&default_int,param_int_readonly,NULL},
                {4,"l_up_p",100,60,&default_float,&default_int,param_int_readonly,NULL},
                {4,"l_down_p",100,80,&default_float,&default_int,param_int_readonly,NULL},
            {3,"round",0,40,&default_float,&default_int,catlog,NULL},

        {2,"forwardsight",0,80,&default_float,&forwardsight,catlog,NULL},
            {3,"forwardsight2",100,20,&default_float,&forwardsight2,param_int,NULL},
            {3,"forwardsight3",100,40,&default_float,&forwardsight3,param_int,NULL},
    {1,"element_open",0,60,&default_float,&default_int,catlog,NULL},//����Ԫ�ؼ�¼

    {1,"element_gothrough",0,80,&default_float,&default_int,catlog,NULL},//����Ԫ�ؼ�¼

    {1,"flash",0,100,&default_float,&default_int,catlog,NULL},
    {1,"setting",0,120,&default_float,&default_int,catlog,NULL},
    {1,"end",0,140,&default_float,&default_int,catlog,NULL}//����ɾȥ

};

enum condition{
    NOACTION,
    DOWN,
    UP,
    CONFIRM,
    BACK

}condition;  
/*
------------------------------------------------------------------------------------------------------------------
�������    ��ʼ����Ļ 
����˵��     ��
���ز���     ��
ʹ��ʾ��     ֱ�ӵ���
��ע��Ϣ     ��
-------------------------------------------------------------------------------------------------------------------
*/
void Menu_Screen_Init(void)
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ�׵׺���
    ips200_init(IPS200_TYPE_SPI);    //����ͨ��ģʽΪSPIͨ��
}

/*
------------------------------------------------------------------------------------------------------------------
�������     ��Ļ��ʾ
����˵��     ��
���ز���     ��
ʹ��ʾ��     ֱ�ӵ���
��ע��Ϣ     ��
-------------------------------------------------------------------------------------------------------------------
*/
void output(void) 
{
    int target_priority=current_state-1;
    if(menu_Mode==edit_int)
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);    //����Ϊ��ɫ�׺���
        ips200_show_string(100,0,"len_i");
        ips200_show_int(160,0,stepper_int[stepper_p_int],3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ��ɫ�׺���

    }
    if(menu_Mode==edit_float)
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);    //����Ϊ��ɫ�׺���
        ips200_show_string(100,0,"len_f");
        ips200_show_float(160,0,stepper_float[stepper_p_float],3,3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ��ɫ�׺���

    }
    if (target_priority==0)//�����˵�
    {
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�
    ips200_show_string(0,0,"menu");//��������ַ�
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�
        for(int i=0;strcmp(menu[i].str, "end") != 0;i++)
        {
            if(menu[i].priority==1)
            {
                if(i==p)
                {
                    if(menu_Mode==normal)
                    {
                        ips200_show_string(0,menu[i].y,"->");//���ָ���ַ�
                        ips200_show_string(20,menu[i].y,menu[i].str);
                    }
                    else if(menu_Mode==edit_int||menu_Mode==edit_float)
                    {
                        ips200_set_color(RGB565_RED, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�
                        ips200_show_string(0,menu[i].y,"->");//���ָ���ַ�
                        ips200_show_string(20,menu[i].y,menu[i].str);
                        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ��ɫ�׺���
                    }

                }
                else
                {
                    ips200_show_string(20,menu[i].y,menu[i].str);
                }
             }
        }
    }
    else if(target_priority!=0)//�Ƕ����˵�
    {
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�
    ips200_show_string(0,0,menu[p_nearby].str);//����ϼ��ַ�
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�

        for(int i=p_nearby+1;menu[i].priority!=target_priority;i++)
        {
            if(menu[i].priority==current_state)
            {
                if(i==p)
                {
                    if(menu_Mode==normal)
                    {
                        ips200_show_string(0,menu[i].y,"->");//���ָ���ַ�
                        ips200_show_string(20,menu[i].y,menu[i].str);
                    }
                    else if(menu_Mode==edit_int||menu_Mode==edit_float)
                    {
                        ips200_set_color(RGB565_RED, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�
                        ips200_show_string(0,menu[i].y,"->");//���ָ���ַ�
                        ips200_show_string(20,menu[i].y,menu[i].str);
                        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ��ɫ�׺���
                    }
                    if(menu[i].type==param_float||menu[i].type==param_float_readonly)
                    {
                       ips200_show_float(menu[i].x,menu[i].y,*menu[i].value_f,3,3);
                    }
                    else if(menu[i].type==param_int||menu[i].type==param_int_readonly)
                    {
                       ips200_show_int(menu[i].x,menu[i].y,*menu[i].value_i,5);
                    }

                }
                else
                {
                    ips200_show_string(20,menu[i].y,menu[i].str);
                    if(menu[i].type==param_float||menu[i].type==param_float_readonly)
                    {
                       ips200_show_float(menu[i].x,menu[i].y,*menu[i].value_f,3,3);
                    }
                    else if(menu[i].type==param_int||menu[i].type==param_int_readonly)
                    {
                       ips200_show_int(menu[i].x,menu[i].y,*menu[i].value_i,5);
                    }

                }
            } 
        }
    }

}

/*
------------------------------------------------------------------------------------------------------------------
�������     �˵�����
����˵��     ��
���ز���     ��
ʹ��ʾ��     ֱ�ӵ���
��ע��Ϣ     
-------------------------------------------------------------------------------------------------------------------
*/
void Menu_control(void)
{
        output();
        status=0;
        condition = (enum condition)input; 
        if(input)
        {
            ips200_clear();
        }
        switch (condition)
        {
        case NOACTION:
                break;
            
        case DOWN:
            if(menu_Mode==edit_int)  //���ͱ༭ģʽ��
            {
                *menu[p].value_i-=stepper_int[stepper_p_int];
                break;
            }
            if(menu_Mode==edit_float)
            { 
                *menu[p].value_f-=stepper_float[stepper_p_float];
                break;
            }
            if (strcmp(menu[p].str, "end") != 0&&menu[p+1].priority>=menu[p].priority)
            {
                int temp=menu[p].priority;
                uint8 old_p=p;
                p++;
                while(menu[p].priority!=temp && strcmp(menu[p+1].str, "end") != 0)
                {
                    p++;
                    if(menu[p].priority<temp)
                    {
                        p=old_p;    //�ص�ԭλ��
                        ips200_show_string(0,180,"endorstart");

                        break;
                    }
                } 
                if(strcmp(menu[p].str,"end")==0)
                {
                    p=old_p;    //�ص�ԭλ��
                    ips200_show_string(0,180,"endorstart");

                    break;
                }
            }
            else
            {
                ips200_show_string(0,180,"endorstart");
            }
            break;
        case UP:
            if(menu_Mode==edit_int)
            {
                *menu[p].value_i+=stepper_int[stepper_p_int];
                break;
            }
            if(menu_Mode==edit_float)
            { 
                *menu[p].value_f+=stepper_float[stepper_p_float];
                break;
            }
            if(p!=0&&menu[p-1].priority>=menu[p].priority)
            {
                int temp=menu[p].priority;
                p--;
                while (menu[p].priority!=temp){p--;}
            }
            else
            {
                ips200_show_string(0,180,"endorstart");
            }
             

            break;
        case CONFIRM:
            if(menu[p+1].priority==current_state+1&&strcmp(menu[p+1].str,"end")!=0&&menu[p].type==catlog)//��Ŀ¼���
            {
                current_state++;
                p_nearby=p;
                p++;
                break;
             }
            if(menu_Mode==edit_int)
            {
                stepper_p_int=(stepper_p_int+1)%5;
                break;

            }
            if(menu_Mode==edit_float)
            {
                stepper_p_float=(stepper_p_float+1)%5;
                break;
            }
            if(menu[p].type==param_float)             //�������                      
            {
                menu_Mode=edit_float;
                break;
                //���븡�����༭ģʽ

            }
            if(menu[p].type==param_int)
            {
                menu_Mode=edit_int;
                break;
                //�������ͱ༭ģʽ
            }
            if(menu[p].type==function)
            {
                menu[p].Operate_default();
            }
            if(menu[p].type==param_float_readonly||menu[p].type==param_int_readonly)
            {
                ips200_show_string(0,180,"error_readonly");
            }
         

            break;
        case BACK:
        if(menu_Mode!=normal) //�༭ģʽ�°����ؼ��˳��༭ģʽ
        {
            menu_Mode=normal;
            break;
        }
        if(menu[p].priority!=1)
        {
            save_flag=true;
            current_state--;
            p=p_nearby;
            while (menu[p_nearby].priority!=current_state-1)
            {
                p_nearby--;
            }
        }
        else
        {
            ips200_show_string(0,180,"error");
        }

        default:
            break;
        }
        input=0;
        
}
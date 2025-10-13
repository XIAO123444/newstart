 #include "menu.h"
#include "encoder.h"
#include "key.h"
#include "steer_pid.h"
#include "balance.h"
#include "PID.h"
#include "flash.h"
#include "photo_chuli.h"
#include "zf_device_lora3a22.h"

bool showline; 


#define ips200_x_max 240
#define ips200_y_max 320
int current_state=1;
int p=0;                        //��¼��ǰָ��
int p_nearby=0;                 //��¼������ָ��
uint8 input;                    //�˵���������
extern int status;


extern uint8 flag;
bool show_flag=false;     //��ʾ��־λ,ȫ�ֱ���

int16 start_count=0;      //��������

//�˵�����
car_mode carmode=stop;                   //ȫ�ֱ��� ��״̬Ĭ��ֹͣ
stop_debug stopdebug=normal_debug;       //ȫ�ֱ��� ͣ��debugĬ������
enum_menu_mode menu_Mode=normal;         //ȫ�ֱ��� �˵�ģʽ

int16 default_int=0;            //�ļ��ڱ��� Ĭ�����ͣ���ֹ��ָ��
float default_float=0.0;        //�ļ��ڱ��� Ĭ�ϸ����ͣ���ֹ��ָ��

uint8 confirm_flag=false;      //ȷ�ϱ�־
int stepper_int[5]={1,5,10,20,50};                         //���Ͳ���ֵ
float stepper_float[6]={0.01,0.1,1.0,10.0,100.0,500.0};   //�����Ͳ���ֵ
uint8 stepper_p_int=0;        //���Ͳ���ֵָ��
uint8 stepper_p_float=0;      //�����Ͳ���ֵָ��

void add_intparam(int16* a)
{
    *a+=stepper_int[stepper_p_int];
}
void sub_intparam(int16* a)
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
//�Ӽ���װ�����������Ҫɾ�ˣ�������������
//�˵������˵������˵������˵������˵������˵������˵������˵�����
enum_roadelementtypedef roadelementType[50]={zebra,straigh,curve
    ,straigh,ramp,crossr,straigh,speedup
    ,obstacle,islandl,straigh,zebra};//ȫ�ֱ�����¼������Ԫ��
int16 element_num=12;       //ȫ�ֱ�������¼����Ԫ������

int32 speed;
//ǰհ��ʾ+���ڲ���         ����track.h
int16 forwardsight;                 //Ĭ��ǰհ
int16 forwardsight2;                //  ֱ���ж�ǰհ��������ע�������ǰհ��ͬ���������ֻ������ֳ����ٵģ�����
int16 forwardsight3;                //���ǰհ

//����balance.c
extern float filtering_angle;       //������ĽǶ�
extern int16 pitch_angle_integr_read;          //�����ǲ˵���ʾ������
extern int16 roll_angle_integr_read;           //����ǲ˵���ʾ������
extern int16 yaw_angle_integr_read;            //ƫ���ǲ˵���ʾ������
extern int16 raw_gyro_x ;                       //ԭʼ����������
extern int16 raw_gyro_y ;                       //ԭʼ����������
extern int16 raw_gyro_z ;                       //ԭʼ����������        

//����zf_device_lora3a22.c
extern lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;

//pid��ʾ+���ڲ���          ����pid.h
extern PID_t PID_gyro;          //���ٶȻ�
extern PID_t PID_angle;         //�ǶȻ�
extern PID_t PID_speed;         //�ٶȻ�  
extern PID_t PID_steer;         //ת��
extern PID_t PID_BLDC;          //��ѹ���Ȼ� 

struct_roadelementypedef roadelement_onoff={1,1,1,1,1,1,1,1,1,1,1,1,1,1};      //����Ԫ�ع��ܿ����ر�
struct_roadelementypedef roadelement_record={0,0,0,0,0,0,0,0,0,0,0,0,0,0};    //��¼����Ԫ��
struct_imageshowcase image ={0,1,0};            //��¼ͼ����ʾ
bool startbool=false; //��ʼ��־

//�����ʾ�����ڲ���          ʹ����photo_chuli.h
int16 threshold_down=100;       //�����ֵ����
int16 threshold_up=200;         //�����ֵ����  
int16 OTSU_calperxpage=5;       //ÿx��ͼƬ����һ�δ��

//����      photo_chuli.c���ֿ���
extern int16 threshold1;  // ����
extern int16 threshold2;  // ����
extern int16 threshold3;  // ����
extern int16 threshold4;  // ����
//
//�˵������˵������˵������˵������˵������˵������˵������˵�����


//�˵������˵������˵������˵������˵������˵������˵�����
void show_element(void)
{
    for (int16 i = 0; i < element_num; i++)
    {
        switch (roadelementType[i])
        {
            
        case straigh:
            ips200_show_string(72*(i%3),30*(i/3),"straigh");
            break;
        case crossm:
            ips200_show_string(72*(i%3),30*(i/3)," crossM "); // ����ʮ��
            break;
        case crossl:
            ips200_show_string(72*(i%3),30*(i/3)," crossL "); // ��бʮ��
            break;
        case crossr:
            ips200_show_string(72*(i%3),30*(i/3)," crossR "); // ��бʮ��
            break;
        case islandl:
            ips200_show_string(72*(i%3),30*(i/3),"islandL"); // ������
            break;
        case islandr:
            ips200_show_string(72*(i%3),30*(i/3),"islandR"); // ������
            break;
        case scurve:
            ips200_show_string(72*(i%3),30*(i/3)," S-curve"); // S��
            break;
        case curve:
            ips200_show_string(72*(i%3),30*(i/3)," curve "); // ���
            break;
        case speedup:
            ips200_show_string(72*(i%3),30*(i/3),"speedUp"); // ���ٴ�
            break;
        case ramp:
            ips200_show_string(72*(i%3),30*(i/3),"  ramp "); // �µ�
            break;
        case obstacle:
            ips200_show_string(72*(i%3),30*(i/3),"obstacle"); // �ϰ���
            break;
        case blackprotect:
            ips200_show_string(72*(i%3),30*(i/3),"blkProt"); // ���߱���
            break;
        case stall:
            ips200_show_string(72*(i%3),30*(i/3)," stall "); // ��ת
            break;
        case zebra:
            ips200_show_string(72*(i%3),30*(i/3),"  zebra "); // ������
            break;
        default:
            break;
        }
        if(i!=element_num-1)
        {
            ips200_set_color(RGB565_ORANGE, RGB565_BLACK);
            ips200_show_string(72*(i%3)+64,30*(i/3),">");
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);

        }

    }
}
//������ʾ����Ԫ��
void show_stopreason(void)
{
    ips200_show_string(0,0,"stop reason:");
    if(stopdebug==blackprotect_stop)
    {
        ips200_show_string(0,20,"blackprotect stop");
    }

    if(stopdebug==zebra_stop)
    {
        ips200_show_string(0,20,"zebra stop");
    }
    if(stopdebug==liftup_stop)
    {
        ips200_show_string(0,20,"liftup stop");
    }
    if (stopdebug==gyro_intrg_pitch_stop)
    {
        ips200_show_string(0,20,"gyro_intrg_pitch stop");
    }
    if (stopdebug==gyro_intrg_roll_stop)
    {
        ips200_show_string(0,20,"gyro_intrg_roll stop");
    }
    if (stopdebug==gyro_intrg_yaw_stop)         
    {
        ips200_show_string(0,20,"gyro_intrg_yaw stop");
    }
    if (stopdebug==timer_count_stop)          
    {
        ips200_show_string(0,20,"timer stop");
    }
 
}
//������ʾֹͣԭ��
void image_show()   {show_flag=true;}
void PID_clear()
{
	PID_gyro.error0 = 0;
    PID_gyro.errorint = 0;
	PID_angle.error0 = 0;
	PID_speed.errorint = 0;
	PID_steer.error0 = 0;
	PID_steer.errorint = 0; 
}
//������������ֹ��ը(��ĿӼǵò���)
void start_the_car() { carmode = car_run_mode1;start_count=0; angle_init();PID_clear();}//��ʼ
void Calibrate_BLDC()   {carmode=Start_Calibrate;}
void Remote_start()     {carmode =remote;start_count=0; angle_init();PID_clear();}//Զ�̿�ʼ

void pid_gyro_set0(){ PID_gyro.kp=0;PID_gyro.ki=0;PID_gyro.kd=0;PID_gyro.kd2=0;PID_gyro.maxout=5000;PID_gyro.minout=-5000;  ips200_show_string(0,180,"set 0 already");} 
void pid_angle_set0(){PID_angle.kp=0;PID_angle.ki=0;PID_angle.kd=0;PID_angle.kd2=0;PID_angle.maxout=5000;PID_angle.minout=-5000;ips200_show_string(0,180,"set 0 already");}     
void pid_V_set0(){PID_speed.kp=0;PID_speed.ki=0;PID_speed.kd=0;PID_speed.kd2=0;PID_speed.maxout=5000;PID_speed.minout=-5000;PID_speed.targ=400;ips200_show_string(0,180,"set 0 already");} 
void pid_steer_set0(){PID_steer.kp=0;PID_steer.ki=0;PID_steer.kd=0;PID_steer.kd2=0;PID_steer.maxout=5000;PID_steer.minout=-5000; ips200_show_string(0,180,"set 0 already");} 
void pid_BLDC_set0(){PID_BLDC.kp=0;PID_BLDC.ki=0;PID_BLDC.kd=0;PID_BLDC.kd2=0;PID_BLDC.maxout=0;PID_BLDC.minout=0; ips200_show_string(0,180,"set 0 already");}
void pid_all_set0(){pid_gyro_set0();pid_angle_set0();pid_V_set0();pid_steer_set0();pid_BLDC_set0();}
void pid_BLDC_mode_set(){pid_gyro_set0();PID_gyro.maxout=100;PID_gyro.minout=0;pid_angle_set0();PID_angle.maxout=40;
    PID_angle.minout=0;pid_V_set0();PID_speed.maxout=100;PID_speed.minout=0;
    pid_steer_set0();PID_steer.maxout=100;PID_steer.minout=0;pid_BLDC_set0();
    PID_BLDC.maxout=0;PID_BLDC.minout=0;}
//����洢 



//����洢
void codeload1(){}
void codeload2(){}
void codeload3(){}
void codeload4(){}
//�ṹ�������0��ʼ��(�в���)


 
//�˵��ṹ��
MENU menu[] = 
{
    {1,"start",                   0,               20, {.param_float=&default_float}, catlog,         NULL},
        {2,"car_go",              0,               20, {.param_float=&default_float}, function,       start_the_car},
        {2,"Calibrate",           0,               40, {.param_float=&default_float}, function,       Calibrate_BLDC},
        {2,"remote_start",        0,               60, {.param_float=&default_float}, function,       Remote_start},

    {1, "pidparam",               0,               40, {.param_float=&default_float}, catlog,         NULL},
        {2, "PID_gyro",           0,               20, {.param_float=&default_float}, catlog,         NULL},
            {3, "kp",          ips200_x_max-10 * 8,  20, {.param_float=&PID_gyro.kp}, param_float,   NULL},
            {3, "ki",          ips200_x_max-10 * 8,  40, {.param_float=&PID_gyro.ki}, param_float,   NULL},
            {3, "kd",          ips200_x_max-10 * 8,  60, {.param_float=&PID_gyro.kd}, param_float,   NULL},
            {3, "maxout",      ips200_x_max-10 * 8,  80, {.param_float=&PID_gyro.maxout}, param_float,  NULL},
            {3, "minout",      ips200_x_max-10 * 8, 100, {.param_float=&PID_gyro.minout}, param_float, NULL},
        {2, "PID_angle",      0,                   40, {.param_float=&default_float}, catlog,      NULL},
            {3, "kp",        ips200_x_max-10 * 8,  20, {.param_float=&PID_angle.kp}, param_float, NULL},
            {3, "ki",        ips200_x_max-10 * 8,  40, {.param_float=&PID_angle.ki}, param_float, NULL},
            {3, "kd",        ips200_x_max-10 * 8,  60, {.param_float=&PID_angle.kd}, param_float, NULL},
            {3, "maxout",    ips200_x_max-10 * 8,  80, {.param_float=&PID_angle.maxout}, param_float,NULL},
            {3, "minout",    ips200_x_max-10 * 8, 100, {.param_float=&PID_angle.minout}, param_float, NULL},
        {2, "PID_V",         0,                   60, {.param_float=&default_float}, catlog,      NULL},
            {3, "kp",        ips200_x_max-10 * 8,  20, {.param_float=&PID_speed.kp}, param_float, NULL},
            {3, "ki",        ips200_x_max-10 * 8,  40, {.param_float=&PID_speed.ki}, param_float, NULL},
            {3, "kd",        ips200_x_max-10 * 8,  60, {.param_float=&PID_speed.kd}, param_float, NULL},
            {3, "maxout",    ips200_x_max-10 * 8,  80, {.param_float=&PID_speed.maxout}, param_float,NULL},
            {3, "minout",    ips200_x_max-10 * 8, 100, {.param_float=&PID_speed.minout}, param_float, NULL},
            {3,"target",     ips200_x_max-10 * 8, 120, {.param_float=&PID_speed.targ}, param_float, NULL},
        {2, "PID_steer",      0,                  80, {.param_float=&default_float}, catlog,      NULL},
            {3, "kp",        ips200_x_max-10 * 8,  20, {.param_float=&PID_steer.kp}, param_float, NULL},
            {3, "ki",        ips200_x_max-10 * 8,  40, {.param_float=&PID_steer.ki}, param_float, NULL},
            {3, "kd",        ips200_x_max-10 * 8,  60, {.param_float=&PID_steer.kd}, param_float, NULL},
            {3, "kd2",       ips200_x_max-10 * 8,  80, {.param_float=&PID_steer.kd2}, param_float, NULL},
            {3, "maxout",    ips200_x_max-10 * 8, 100, {.param_float=&PID_steer.maxout}, param_float,NULL},
            {3, "minout",    ips200_x_max-10 * 8, 120, {.param_float=&PID_steer.minout}, param_float, NULL},
        {2, "PID_BLDC",      0,                  100, {.param_float=&default_float}, catlog,      NULL},
            {3,"kp",        ips200_x_max-10 * 8,  20, {.param_float=&PID_BLDC.kp}, param_float, NULL},
            {3,"ki",        ips200_x_max-10 * 8,  40, {.param_float=&PID_BLDC.ki}, param_float, NULL},
            {3,"kd",        ips200_x_max-10 * 8,  60, {.param_float=&PID_BLDC.kd}, param_float, NULL},
            {3,"kd2",       ips200_x_max-10 * 8,  80, {.param_float=&PID_BLDC.kd2}, param_float, NULL},
            {3,"maxout",    ips200_x_max-10 * 8, 100, {.param_float=&PID_BLDC.maxout}, param_float,NULL},
            {3,"minout",    ips200_x_max-10 * 8, 120, {.param_float=&PID_BLDC.minout}, param_float,NULL},
        {2, "allset0",       0,                 120, {.param_float=&default_float}, confirm,     pid_all_set0},
        {2, "PID_gyro_set0", 0,                 140, {.param_float=&default_float}, confirm,     pid_gyro_set0},
        {2, "PID_angle_set0",0,                 160, {.param_float=&default_float}, confirm,     pid_angle_set0},
        {2, "PID_V_set0",    0,                 180, {.param_float=&default_float}, confirm,     pid_V_set0},
        {2, "PID_steer_set0",0,                 200, {.param_float=&default_float}, confirm,     pid_steer_set0},
        {2,"PID_BLDC_modeset0",0,               220, {.param_float=&default_float}, confirm,     pid_BLDC_mode_set},
    
    {1, "image",              0,                  60, {.param_float=&default_float}, catlog,      NULL},
        {2, "ROLL_angle",    100,                 20, {.param_float=&filtering_angle}, param_float_readonly, NULL},
        {2, "display",       0,                  40, {.param_float=&default_float}, function,    image_show},
        {2, "show_image",    0,                  60, {.param_float=&default_float}, catlog,    NULL},
            {3, "show_grayimage",180,             20, {.param_float=&default_float}, chose1, NULL},
            {3, "show_ostuimage",180,             40, {.param_float=&default_float}, chose1, NULL},
            {3, "show_dev_image",180,             60, {.param_float=&default_float}, chose1, NULL},
        {2, "OTSU_threshold",0,                  80, {.param_float=&default_float}, catlog, NULL},
            {3, "OTSU_up",   100,                20, {.param_int16=&threshold_up}, param_int16, NULL},
            {3, "OTSU_DOWN",100,                 40, {.param_int16=&threshold_down}, param_int16, NULL},
            {3,"OTSU_perx", 100,                 60, {.param_int16=&OTSU_calperxpage}, param_int16, NULL},
            {3,"threshold1",100,                 80, {.param_int16=&threshold1}, param_int16, NULL},
            {3,"threshold2",100,                100, {.param_int16=&threshold2}, param_int16, NULL},
            {3,"threshold3",100,                120, {.param_int16=&threshold3}, param_int16, NULL},
            {3,"threshold4",100,                140, {.param_int16=&threshold4}, param_int16, NULL},
        {2, "image_point",   0,                 100, {.param_float=&default_float}, catlog, NULL},
            {3, "crossroadall",0,                20, {.param_float=&default_float}, catlog, NULL},
                {4, "r_up_p",100,                20, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "r_down_p",100,              40, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "l_up_p",100,                60, {.param_int16=&default_int}, param_int16_readonly, NULL},
                {4, "l_down_p",100,              80, {.param_int16=&default_int}, param_int16_readonly, NULL},
            {3, "round",     0,                  40, {.param_float=&default_float}, catlog, NULL},
        {2, "forwardsight",  0,                 120, {.param_float=&default_float}, catlog, NULL},
            {3, "forwardsight1",150,             20, {.param_int16=&forwardsight}, param_int16, NULL},
            {3, "forwardsight2",150,             40, {.param_int16=&forwardsight2}, param_int16, NULL},
            {3, "forwardsight3",150,             60, {.param_int16=&forwardsight3}, param_int16, NULL},
    
    {1, "debug",      0,                  80, {.param_float=&default_float}, catlog, NULL},
        {2,"gyro_info",      0,                 20, {.param_float=&default_float}, catlog, NULL},
            {3,"pit_intg",   100,               20, {.param_int16=&pitch_angle_integr_read}, param_int16_readonly, NULL},
            {3,"yaw_intg",   100,               40, {.param_int16=&yaw_angle_integr_read}, param_int16_readonly, NULL},
            {3,"roll_intg",  100,               60, {.param_int16=&roll_angle_integr_read}, param_int16_readonly, NULL},
            {3, "ROLL_angle",100,               80, {.param_float=&filtering_angle}, param_float_readonly, NULL},
            {3,"raw_gyro_x", 100,              100, {.param_int16=&raw_gyro_x}, param_int16_readonly, NULL},
            {3,"raw_gyro_y", 100,              120, {.param_int16=&raw_gyro_y}, param_int16_readonly, NULL},
            {3,"raw_gyro_z", 100,              140, {.param_int16=&raw_gyro_z}, param_int16_readonly, NULL},
            {3,"imu_gyro_x", 100,              160, {.param_int16=&imu660ra_gyro_x}, param_int16_readonly, NULL},
        {2,"remote_info",    0,                 40, {.param_float=&default_float}, catlog, NULL},
            {3,"l_stick_UD", 150,               20, {.param_int16=&lora3a22_uart_transfer.joystick[1]}, param_int16_readonly, NULL},
            {3,"l_stick_LR", 150,               40, {.param_int16=&lora3a22_uart_transfer.joystick[0]}, param_int16_readonly, NULL},
            {3,"r_stick_UD", 150,               60, {.param_int16=&lora3a22_uart_transfer.joystick[3]}, param_int16_readonly, NULL},
            {3,"r_stick_LR", 150,               80, {.param_int16=&lora3a22_uart_transfer.joystick[2]}, param_int16_readonly, NULL},
            {3,"l_stickey",  150,              100, {.param_uint8=&lora3a22_uart_transfer.key[0]}, param_uint8_readonly, NULL},
            {3,"r_stickey",  150,              120, {.param_uint8=&lora3a22_uart_transfer.key[1]}, param_uint8_readonly, NULL},
            {3,"l_key",      150,              140, {.param_uint8=&lora3a22_uart_transfer.key[2]}, param_uint8_readonly, NULL},
            {3,"r_key",      150,              160, {.param_uint8=&lora3a22_uart_transfer.key[3]}, param_uint8_readonly, NULL},
            {3,"Lswitch_key1",150,             180, {.param_uint8=&lora3a22_uart_transfer.switch_key[0]}, param_uint8_readonly, NULL},
            {3,"Lswitch_key2",150,             200, {.param_uint8=&lora3a22_uart_transfer.switch_key[1]}, param_uint8_readonly, NULL},
            {3,"Rswitch_key1",150,             220, {.param_uint8=&lora3a22_uart_transfer.switch_key[2]}, param_uint8_readonly, NULL},
            {3,"Rswitch_key2",150,             240, {.param_uint8=&lora3a22_uart_transfer.switch_key[3]}, param_uint8_readonly, NULL},

    {1, "element",  0,                  100, {.param_float=&default_float}, catlog, NULL},
        {2, "element_onoff", 0,                  20, {.param_float=&default_float}, catlog, NULL},
            {3, "crossl",    100,                20, {.param_int16=&roadelement_onoff.crossl}, on_off, NULL},
            {3, "crossr",    100,                40, {.param_int16=&roadelement_onoff.crossr}, on_off, NULL},
            {3, "crossm",    100,                60, {.param_int16=&roadelement_onoff.crossm}, on_off, NULL},
            {3, "islandl",   100,                80, {.param_int16=&roadelement_onoff.islandl}, on_off, NULL},
            {3, "islandR",   100,               100, {.param_int16=&roadelement_onoff.islandr}, on_off, NULL},
            {3, "scurve",    100,               120, {.param_int16=&roadelement_onoff.scurve}, on_off, NULL},
            {3, "speedup",   100,               140, {.param_int16=&roadelement_onoff.speedup}, on_off, NULL},
            {3, "ramp",      100,               160, {.param_int16=&roadelement_onoff.ramp}, on_off, NULL},
            {3, "obstacle",  100,               180, {.param_int16=&roadelement_onoff.obstacle}, on_off, NULL},
        {2, "element_count",0,                  40, {.param_float=&default_float}, catlog, NULL},
            {3, "straigh",  100,                20, {.param_int16=&roadelement_record.straigh}, param_int16_readonly, NULL},
            {3, "crossm",    100,                40, {.param_int16=&roadelement_record.crossm}, param_int16_readonly, NULL},
            {3, "crossl",    100,                60, {.param_int16=&roadelement_record.crossl}, param_int16_readonly, NULL},
            {3, "crossr",    100,                80, {.param_int16=&roadelement_record.crossr}, param_int16_readonly, NULL},
            {3, "islandl",   100,               100, {.param_int16=&roadelement_record.islandl}, param_int16_readonly, NULL},
            {3, "islandr",   100,               120, {.param_int16=&roadelement_record.islandr}, param_int16_readonly, NULL},
            {3, "scurve",    100,               140, {.param_int16=&roadelement_record.scurve}, param_int16_readonly, NULL},
            {3, "curve",     100,               160, {.param_int16=&roadelement_record.curve}, param_int16_readonly, NULL},
            {3, "speedup",   100,               180, {.param_int16=&roadelement_record.speedup}, param_int16_readonly, NULL},
            {3, "ramp",      100,               200, {.param_int16=&roadelement_record.ramp}, param_int16_readonly, NULL},
            {3, "obstacle",  100,               220, {.param_int16=&roadelement_record.obstacle}, param_int16_readonly, NULL},
            {3, "blackprotect",100,             240, {.param_int16=&roadelement_record.blackprotect}, param_int16_readonly, NULL},
            {3, "stall",     100,               260, {.param_int16=&roadelement_record.stall}, param_int16_readonly, NULL},
            {3, "zebra",     100,               280, {.param_int16=&roadelement_record.zebra}, param_int16_readonly, NULL},
        {2, "element_gothrough",0,              60, {.param_float=&default_float}, roadgothrough, NULL},
        {2, "record_clear", 0,                  80, {.param_float=&default_float}, function, NULL},
    
    {1, "flash",              0,                 120, {.param_float=&default_float}, catlog, NULL},
        {2, "code_load",    100,                 20, {.param_float=&default_float}, catlog, NULL},
            {3, "load1",    100,                 20, {.param_float=&default_float}, confirm, NULL},
            {3, "load2",    100,                 40, {.param_float=&default_float}, confirm, NULL},
            {3, "load3",    100,                 60, {.param_float=&default_float}, confirm, NULL},
            {3, "load4",    100,                 80, {.param_float=&default_float}, confirm, NULL},
        {2, "flash_load",   100,                 40, {.param_float=&default_float}, catlog, NULL},
            {3, "load1",    100,                 20, {.param_float=&default_float}, confirm, flash_load_config_1},
            {3, "load2",    100,                 40, {.param_float=&default_float}, confirm, flash_load_config_2},
            {3, "load3",    100,                 60, {.param_float=&default_float}, confirm, flash_load_config_3},
            {3, "load4",    100,                 80, {.param_float=&default_float}, confirm, flash_load_config_4},
            {3, "loaddefault",100,              100, {.param_float=&default_float}, confirm, flash_load_config_default},
        {2, "flash_save",   100,                 60, {.param_float=&default_float}, catlog, NULL},
            {3, "save1",    100,                 20, {.param_float=&default_float}, confirm, flash_save_config_1},
            {3, "save2",    100,                 40, {.param_float=&default_float}, confirm, flash_save_config_2},
            {3, "save3",    100,                 60, {.param_float=&default_float}, confirm, flash_save_config_3},
            {3, "save4",    100,                 80, {.param_float=&default_float}, confirm, flash_save_config_4},
        {2, "resetflash",   100,                 80, {.param_float=&default_float}, confirm, flash_reset},
    
    {1, "setting",            0,                 140, {.param_float=&default_float}, catlog, NULL},
    {1, "end",                0,                   0, {.param_float=&default_float}, catlog, NULL}
};



enum_Condition condition = NOACTION;//�˵���Ϊ��ʼ��
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
�������     ������ʾ
����˵��     X1Ϊ�ַ�����ʾ��ʼλ
                X2Ϊ��ֵ��ʾ��ʼλ
                YΪ��ʾ��
                typeΪ��ʾ����
                *aΪ���Ͳ�����ַ
                *pΪ�����Ͳ�����ַ
                strΪ��������
���ز���     ��
ʹ��ʾ��     display_fast(0, 100, 0, param_int16, &default_int,  "default");
��ע��Ϣ     ��0��0λ����ʾdefault����������0��100����ʾdefault_int���Ͳ���ֵ
-------------------------------------------------------------------------------------------------------------------
*/

union_param fast_show[7]={
{.param_float=&filtering_angle},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float},
{.param_float=&default_float}
};

void display_fast(int16 X1,int16 X2,int16 Y,enum_function type,union_param Union_param,char str[20])
{
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_show_string(X1,Y,str);
    switch (type)
    {
    case param_uint8:
        ips200_show_int(X2,Y,*Union_param.param_uint8,5);
        break;
    case param_int8:
        ips200_show_int(X2,Y,*Union_param.param_int8,5);    
        break;
    case param_uint16:
        ips200_show_int(X2,Y,*Union_param.param_uint16,5);
        break;
    case param_int16:
        ips200_show_int(X2,Y,*Union_param.param_int16,5);
        break;
    case param_uint32:
        ips200_show_int(X2,Y,*Union_param.param_uint32,10); 
        break;
    case param_int32:       
        ips200_show_int(X2,Y,*Union_param.param_int32,10); 
        break;
    case param_float:
        ips200_show_float(X2,Y,*Union_param.param_float,5,3); 
        break;
    case param_double:
        ips200_show_float(X2,Y,*Union_param.param_double,10,6); 
        break;    
    default:
        break;
    }

}
void outputscreen_fast()
{
    if(show_flag==false&&current_state==1&&menu_Mode==normal)//���ͼ����ʾû�����ڶ����˵�
    {    
        ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        ips200_show_string(0,160,"fast_show");
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);
        //Ҫ��ӿ�����ʾ������������

        display_fast(0, 60, 180, param_float,fast_show[0] , "ROLL_angle");
    }


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
    int16 target_priority=current_state-1;
    outputscreen_fast(); //������ʾ
    if(menu_Mode==edit_int)     //���ͱ༭ģʽ��
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);    //����Ϊ��ɫ�׺���
        ips200_show_string(100,0,"len_i");
        ips200_show_int(160,0,stepper_int[stepper_p_int],3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ��ɫ�׺���

    }
    if(menu_Mode==edit_float)       //����༭ģʽ��
    {
        ips200_set_color(RGB565_BROWN, RGB565_BLACK);    //����Ϊ��ɫ�׺���
        ips200_show_string(100,0,"len_f");
        ips200_show_float(160,0,stepper_float[stepper_p_float],3,3);
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ��ɫ�׺���

    }
    if(menu_Mode==edit_confirm)     //ȷ��ģʽ��
    {
        ips200_set_color(RGB565_ORANGE,RGB565_BLACK);
        ips200_show_string(20,0,"WARNING!WARNING!WARNING!");
        ips200_show_string(20,160,"PRESS BOTTON3 TO CONFIRM");
        ips200_show_string(20,300,"WARNING!WARNING!WARNING!");
        return;                     //��ǰ�˳�
    }
    if(menu_Mode==special_show_element1)//��ʾ����Ԫ��ģʽ��
    {
        show_element();
        return;
    }
    if(menu_Mode==stop_debug_display)//ֹͣ������ʾģʽ��
    {
        show_stopreason();
        return;                     //��ǰ�˳�
    }
    if(target_priority==0)         //�����˵�
    {
        ips200_set_color(RGB565_DustyBlue, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�
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
                        ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�
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
        ips200_set_color(RGB565_DustyBlue, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�
        ips200_show_string(0,0,menu[p_nearby].str);//����ϼ��ַ�
        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�

        for(int i=p_nearby+1;menu[i].priority!=target_priority;i++)
        {
            if(menu[i].priority==current_state)
            {
                if(i==p)
                {
                    if(menu_Mode==normal)       //��ͨ��ʾ��������ʾ��ͷ
                    {
                        ips200_show_string(0,menu[i].y,"->");//���ָ���ַ�
                        ips200_show_string(20,menu[i].y,menu[i].str);
                    }
                    else if(menu_Mode==edit_int||menu_Mode==edit_float) //�༭ģʽ�£��ı���ɫ    
                    {
                        ips200_set_color(RGB565_MAGENTA, RGB565_BLACK);//����Ϊ��ɫ�ڵ�
                        ips200_show_string(0,menu[i].y,"->");//���ָ���ַ�
                        ips200_show_string(20,menu[i].y,menu[i].str);
                        ips200_set_color(RGB565_WHITE, RGB565_BLACK);//����Ϊ�ڵװ���
                    }
                    if(menu[i].type==param_float||menu[i].type==param_float_readonly)
                    {
                       ips200_show_float(menu[i].x,menu[i].y,*menu[i].param_union.param_float,4,3);
                    }
                    else if(menu[i].type>=param_int8&&menu[i].type<=param_uint32_readonly)
                    {
                        int32 temp;
                        switch (menu[i].type)
                        {
                        case param_int8:
                            temp = (int32)(*menu[i].param_union.param_int8);
                            break;
                        case param_uint8:
                            temp = (int32)(*menu[i].param_union.param_uint8);
                            break;
                        case param_int16:
                            temp = (int32)(*menu[i].param_union.param_int16);   
                            break;
                        case param_uint16:
                            temp = (int32)(*menu[i].param_union.param_uint16);
                            break;
                        case param_int32:
                            temp = *menu[i].param_union.param_int32;
                            break;
                        case param_uint32:
                            temp = (int32)(*menu[i].param_union.param_uint32);
                            break;
                        case param_uint8_readonly:
                            temp = (int32)(*menu[i].param_union.param_uint8);
                            break;
                        case param_int8_readonly:
                            temp = (int32)(*menu[i].param_union.param_int8);
                            break;
                        case param_uint16_readonly:
                            temp = (int32)(*menu[i].param_union.param_uint16);
                            break;
                        case param_int16_readonly:
                            temp = (int32)(*menu[i].param_union.param_int16);   
                            break;
                        case param_int32_readonly:
                            temp = *menu[i].param_union.param_int32;    
                            break;
                        case param_uint32_readonly: 
                            temp = (int32)(*menu[i].param_union.param_uint32);
                            break;

                                
                        default:
                            break;
                        }
                       ips200_show_int(menu[i].x,menu[i].y,temp,5);
                    }
                    else if(menu[i].type==on_off||menu[i].type==chose1)
                    {
                        ips200_set_color(RGB565_ORANGE, RGB565_BLACK);    //����Ϊ��ɫ�ڵ�
                        if (*menu[i].param_union.param_int16)
                        {
                            ips200_show_string(menu[i].x,menu[i].y,"ON");

                        }
                        else
                        {
                            ips200_show_string(menu[i].x,menu[i].y,"OFF");
                        }
                        ips200_set_color(RGB565_WHITE, RGB565_BLACK);    //����Ϊ�ڵװ���

                    }

                }
                else
                {
                    ips200_show_string(20,menu[i].y,menu[i].str);
                    if(menu[i].type==param_float||menu[i].type==param_float_readonly)
                    {
                       ips200_show_float(menu[i].x,menu[i].y,*menu[i].param_union.param_float,4,3);
                    }
                    else if(menu[i].type>=param_int8&&menu[i].type<=param_uint32_readonly)
                    {
                        int32 temp;
                        switch (menu[i].type)
                        {
                        case param_int8:
                            temp = (int32)(*menu[i].param_union.param_int8);
                            break;
                        case param_uint8:
                            temp = (int32)(*menu[i].param_union.param_uint8);
                            break;
                        case param_int16:
                            temp = (int32)(*menu[i].param_union.param_int16);   
                            break;
                        case param_uint16:
                            temp = (int32)(*menu[i].param_union.param_uint16);
                            break;
                        case param_int32:
                            temp = *menu[i].param_union.param_int32;
                            break;
                        case param_uint32:
                            temp = (int32)(*menu[i].param_union.param_uint32);
                            break;
                        case param_uint8_readonly:
                            temp = (int32)(*menu[i].param_union.param_uint8);
                            break;
                        case param_int8_readonly:
                            temp = (int32)(*menu[i].param_union.param_int8);
                            break;
                        case param_uint16_readonly:
                            temp = (int32)(*menu[i].param_union.param_uint16);
                            break;
                        case param_int16_readonly:
                            temp = (int32)(*menu[i].param_union.param_int16);
                            break;
                        case param_int32_readonly:
                            temp = *menu[i].param_union.param_int32;    
                            break;
                        default:
                            break;
                        }
                       ips200_show_int(menu[i].x,menu[i].y,temp,5);
                    }
                    else if(menu[i].type==on_off||menu[i].type==chose1)
                    {
                        if (*menu[i].param_union.param_int16)
                        {
                            ips200_show_string(menu[i].x,menu[i].y,"ON");

                        }
                        else
                        {
                            ips200_show_string(menu[i].x,menu[i].y,"OFF");
                        }
                        
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
        condition = (enum_Condition)input; 
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
                switch (menu[p].type)
                {
                case param_int8:
                    *menu[p].param_union.param_int8-=stepper_int[stepper_p_int];
                    break;
                case param_uint8:
                    *menu[p].param_union.param_uint8-=stepper_int[stepper_p_int];
                    break;
                case param_int16:
                    *menu[p].param_union.param_int16-=stepper_int[stepper_p_int];
                    break;
                case param_uint16:
                    *menu[p].param_union.param_uint16-=stepper_int[stepper_p_int];
                    break;
                case param_int32:
                    *menu[p].param_union.param_int32-=stepper_int[stepper_p_int];
                    break;
                case param_uint32:
                    *menu[p].param_union.param_uint32-=stepper_int[stepper_p_int];
                    break;
                default:
                    break;
                }
                // *menu[p].param_union.param_int16-=stepper_int[stepper_p_int];
                return;
            }
            if(menu_Mode==edit_float)
            { 
                switch (menu[p].type)
                {
                case param_float:
                    *menu[p].param_union.param_float-=stepper_float[stepper_p_float];
                    break;
                case param_double:
                    *menu[p].param_union.param_double-=stepper_float[stepper_p_float];
                    break;
                
                default:
                    break;
                }
                return;
            }
            if (strcmp(menu[p].str, "end") != 0&&menu[p+1].priority>=menu[p].priority)        //���˵��ȼ�(���߲�Ҫ��)

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
        //��������ʵ��  ��������ʵ��  ��������ʵ��  ��������ʵ��  ��������ʵ��  ��������ʵ��  ��������ʵ��  
            if(menu_Mode==edit_int)         //���ͱ༭ģʽ��,��������������������
            {
                switch (menu[p].type)
                {
                case param_int8:
                    *menu[p].param_union.param_int8+=stepper_int[stepper_p_int];
                    break;
                case param_uint8:
                    *menu[p].param_union.param_uint8+=stepper_int[stepper_p_int];
                    break;
                case param_int16:
                    *menu[p].param_union.param_int16+=stepper_int[stepper_p_int];
                    break;
                case param_uint16:
                    *menu[p].param_union.param_uint16+=stepper_int[stepper_p_int];
                    break;
                case param_int32:
                    *menu[p].param_union.param_int32+=stepper_int[stepper_p_int];
                    break;
                case param_uint32:
                    *menu[p].param_union.param_uint32+=stepper_int[stepper_p_int];
                    break;
                default:
                    break;
                }
                return;
            }
            if(menu_Mode==edit_float)       //����༭ģʽ��,�������и�����������
            { 
                switch (menu[p].type)
                {
                case param_float:
                    *menu[p].param_union.param_float+=stepper_float[stepper_p_float];
                    break;
                case param_double:
                    *menu[p].param_union.param_double+=stepper_float[stepper_p_float];
                    break;
                
                default:
                    break;
                }
                return;
            }
        //��������ʵ��  ��������ʵ��  ��������ʵ��  ��������ʵ��  ��������ʵ��  ��������ʵ��  ��������ʵ��  
            if(p!=0&&menu[p-1].priority>=menu[p].priority)      //���˵��ȼ�(���߲�Ҫ��)
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
        //���˵��ȼ�(���߲�Ҫ��)
            if(menu[p+1].priority==current_state+1&&strcmp(menu[p+1].str,"end")!=0&&menu[p].type==catlog)       //��Ŀ¼���
            {
                current_state++;
                p_nearby=p;
                p++;
                break;
            }
        //�˵�ģʽ���
            if(menu_Mode==edit_int)                         //���ͱ༭ģʽ��
            {
                stepper_p_int=(stepper_p_int+1)%5;
                break;
            }
            if(menu_Mode==edit_float)                       //����༭ģʽ��
            {
                stepper_p_float=(stepper_p_float+1)%6;
                break;
            }
            if(menu_Mode==edit_confirm)                     //ȷ��ģʽ��
            {
                ips200_clear();                             //����
                menu[p].Operate_default();
                menu_Mode=normal;
                break;
            }
        //�˵��������
            if(menu[p].type==param_float||menu[p].type==param_double)                   //�˵��������                      
            {
                menu_Mode=edit_float;
                break;
                //���븡�����༭ģʽ
            }
            if(menu[p].type==param_int16||menu[p].type==param_int32
                ||menu[p].type==param_int8||menu[p].type==param_uint16
                ||menu[p].type==param_uint32||menu[p].type==param_uint8)                 //�˵��������
            {
                menu_Mode=edit_int;
                break;
                //�������ͱ༭ģʽ
            }
            if (menu[p].type==confirm)                  //ȷ�����
            {
                menu_Mode=edit_confirm;
                break;
            }
            if(menu[p].type==on_off)                    //�������
            {
                *menu[p].param_union.param_int16=1-*menu[p].param_union.param_int16;
                break;
            }
            if(menu[p].type==function)                  //�������
            {
                menu[p].Operate_default();
                break;
            }
            if(menu[p].type==roadgothrough)             //����Ԫ��ͨ�����
            {
                menu_Mode=special_show_element1;        //������ʾ����Ԫ��ģʽ
                break;
            }
            if(menu[p].type==param_float_readonly||menu[p].type==param_int16_readonly
                ||menu[p].type==param_int32_readonly||menu[p].type==param_uint16_readonly
                ||menu[p].type==param_uint32_readonly||menu[p].type==param_double_readonly
                ||menu[p].type==param_int8_readonly||menu[p].type==param_uint8_readonly)        //ֻ���������
            {
                ips200_show_string(0,180,"error_readonly");
            }
            if(menu[p].type==chose1)        //ͬһ������ͬ�ȼ�ͬ����ֻѡһ��
            {
                for(int i=p_nearby+1;menu[i].priority!=current_state-1;i++)
                {
                    if(menu[i].priority==current_state&&menu[i].type==chose1&&i!=p)
                    {
                        *menu[i].param_union.param_int16=0;
                    }
                    if(i==p)
                    {
                        *menu[p].param_union.param_int16=1;
                    }
                }
                break;
            }
            break;
        case BACK:
        if(menu_Mode==edit_float||menu_Mode==edit_confirm||menu_Mode==edit_int
        ||menu_Mode==special_show_element1||menu_Mode==stop_debug_display) //�༭ģʽ�°����ؼ��˳��༭ģʽ
        {
            menu_Mode=normal;
            break;
        }
        if(menu[p].priority==1)
        {
            flash_save_config_default();
            ips200_set_color(RGB565_PURPLE,RGB565_BLACK);
            ips200_show_string(0,300,"save default already");
            ips200_set_color(RGB565_WHITE,RGB565_BLACK);

            break;
        }
        if(menu[p].priority!=1)                 //���˵��ȼ�(���߲�Ҫ��)
        {
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

#include "zf_common_headfile.h"
#include "Balance.h"
#include "PID.h"
#include "menu.h"
uint8_t acc_ration=4;    //���ٶȼ����Ŷ�
uint8_t gyro_ration=4;   //���������Ŷ�
float filtering_angle=0; //������ĽǶ�
float angle_temp;        //�Ƕȼ����м����
float cycle_T=0.005;         //��������
int32 pitch_angle_integr=0;          //�����ǻ���
int32 roll_angle_integr=0;           //����ǻ���
int32 yaw_angle_integr=0;            //ƫ���ǻ���

int16 pitch_angle_integr_read;          //�����ǲ˵���ʾ������
int16 roll_angle_integr_read;           //����ǲ˵���ʾ������
int16 yaw_angle_integr_read;            //ƫ���ǲ˵���ʾ������  

imu_err_typdef imu_err={-2,2,0}; //imu���ṹ��
uint16 az_last=0; //�ϴμ��ٶȼ�Z������
#include <math.h>
//���� menu.h
extern car_mode carmode;  //����״̬
extern stop_debug stopdebug; //ͣ��debug
extern enum_menu_mode menu_Mode;         //�˵�ģʽ


// �Ľ��ĵ�ͨ�˲�����
void low_pass_filter(int16_t* filtered, int16_t new_data, float alpha) {
    *filtered = *filtered * (1 - alpha) + new_data * alpha;
}
int16 raw_gyro_x ;
int16 raw_gyro_y ;
int16 raw_gyro_z ;
int16 raw_acc_x ;
int16 raw_acc_y ;
int16 raw_acc_z ;
//��ʼ��
void angle_init(void)
{
    pitch_angle_integr=0;
    roll_angle_integr=0;
    yaw_angle_integr=0;
}
// �Ľ���IMU�˲�����

void imu_filter(){
    // ȥ����̬����ƫУ����
    raw_gyro_x = imu660ra_gyro_x + imu_err.gx_err;      //�����
    raw_gyro_y = imu660ra_gyro_y + imu_err.gy_err;      //������
    raw_gyro_z = imu660ra_gyro_z + imu_err.gz_err;      //ƫ����

    raw_acc_x = imu660ra_acc_x;
    raw_acc_y = imu660ra_acc_y;
    raw_acc_z = imu660ra_acc_z;
    // ������������΢С������
    if (abs(raw_gyro_x) <= 5) raw_gyro_x = 0;
    if (abs(raw_gyro_y) <= 5) raw_gyro_y = 0;
    if (abs(raw_gyro_z) <= 5) raw_gyro_z = 0;
    pitch_angle_integr += raw_gyro_y; //�����ǻ���
    roll_angle_integr += raw_gyro_x;  //����ǻ���
    yaw_angle_integr += raw_gyro_z;   //ƫ���ǻ���

    pitch_angle_integr_read = pitch_angle_integr / 1000;        //�����ǲ˵���ʾ������
    roll_angle_integr_read = roll_angle_integr / 1000;           //����ǲ˵���ʾ������
    yaw_angle_integr_read = yaw_angle_integr / 30000;             //ƫ���ǲ˵���ʾ������
    // Ӧ�õ�ͨ�˲�
    // low_pass_filter(&imu660ra_gyro_x, raw_gyro_x, 0.5);
    // low_pass_filter(&imu660ra_gyro_y, raw_gyro_y, 0.5);
    // low_pass_filter(&imu660ra_gyro_z, raw_gyro_z, 0.5);
    // low_pass_filter(&imu660ra_acc_x, raw_acc_x, 0.5);
    // low_pass_filter(&imu660ra_acc_y, raw_acc_y, 0.5);
    // low_pass_filter(&imu660ra_acc_z, raw_acc_z, 0.5);
}
void first_order_filtering(void)
{
    int16 gx, gy, gz, ax, ay, az;
    //��ʱ�����洢                                                      
    gx = raw_gyro_x;
    gy = raw_gyro_y;
    gz = raw_gyro_z;
    ax = raw_acc_x;
    ay = raw_acc_y;
    az = raw_acc_z;

    //һ�׻����˲���Ƕ�
    float gyro_temp;
    float acc_temp;
    gyro_temp = (-gy) * gyro_ration;
    acc_temp = (ax - angle_temp) * acc_ration;
    angle_temp += ((gyro_temp + acc_temp) * cycle_T);
    filtering_angle = angle_temp;
}
void lift_protection(void)
{
    int16 gx,gy,gz,ax,ay,az;
    //��ʱ�����洢                                                      
    gx=imu660ra_gyro_x;
    gy=imu660ra_gyro_y;
    gz=imu660ra_gyro_z;                                                        
    ax=imu660ra_acc_x;
    ay=imu660ra_acc_y;
    az=imu660ra_acc_z;

    uint8 acc_abnormal=abs(az-az_last)>8000; //���ٶ��쳣��־
    if(acc_abnormal) //������ٶ��쳣��Ƕ��쳣
    {//ͣ��
        carmode=stop;
        stopdebug=liftup_stop;
        menu_Mode=stop_debug_display;           //�˵��л���ͣ����ʾ
    }

    az_last=az; //�����ϴμ��ٶȼ�Z������
}

void gyro_protect(void)
{
    if( abs(pitch_angle_integr)>20000)//����Ŀ�
    {
        carmode=stop;
        stopdebug=gyro_intrg_pitch_stop;
        
    }
    if( abs(roll_angle_integr)>20000)//����Ŀ�
    {
        carmode=stop;
        stopdebug=gyro_intrg_roll_stop;
    }
    if( abs(yaw_angle_integr)>20000)//����Ŀ�
    {

        carmode=stop;
        stopdebug=gyro_intrg_yaw_stop;
        menu_Mode=stop_debug_display;           //�˵��л���ͣ����ʾ

    }

}
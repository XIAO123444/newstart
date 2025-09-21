#include "zf_common_headfile.h"
#include "Balance.h"
#include "PID.h"
#include "math.h"
#include "math.h"
uint8_t acc_ration=4;    //���ٶȼ����Ŷ�
uint8_t gyro_ration=4;   //���������Ŷ�
float filtering_angle=0; //������ĽǶ�
float angle_temp;        //�Ƕȼ����м����
float cycle_T=0.005;         //��������
float pitch_angle;          //�����
float roll_angle;           //������
imu_err_typdef imu_err={-6,3,-2}; //imu���ṹ��
uint16 az_last=0; //�ϴμ��ٶȼ�Z������
#include <math.h>

extern bool stop;
// �Ľ��ĵ�ͨ�˲�����
void low_pass_filter(int16_t* filtered, int16_t new_data, float alpha) {
    *filtered = *filtered * (1 - alpha) + new_data * alpha;
}

// �Ľ���IMU�˲�����
void imu_filter(){
    // ȥ����̬����ƫУ����
    int16 raw_gyro_x = imu660ra_gyro_x + imu_err.gx_err;
    int16 raw_gyro_y = imu660ra_gyro_y + imu_err.gy_err;
    int16 raw_gyro_z = imu660ra_gyro_z + imu_err.gz_err;
    int16 raw_acc_x = imu660ra_acc_x;
    int16 raw_acc_y = imu660ra_acc_y;
    int16 raw_acc_z = imu660ra_acc_z;
    // ������������΢С������
    if (fabs(raw_gyro_x) < 5) raw_gyro_x = 0;
    if (fabs(raw_gyro_y) < 5) raw_gyro_y = 0;
    if (fabs(raw_gyro_z) < 5) raw_gyro_z = 0;

    // Ӧ�õ�ͨ�˲�
    low_pass_filter(&imu660ra_gyro_x, raw_gyro_x, 0.5);
    low_pass_filter(&imu660ra_gyro_y, raw_gyro_y, 0.5);
    low_pass_filter(&imu660ra_gyro_z, raw_gyro_z, 0.5);
    low_pass_filter(&imu660ra_acc_x, raw_acc_x, 0.5);
    low_pass_filter(&imu660ra_acc_y, raw_acc_y, 0.5);
    low_pass_filter(&imu660ra_acc_z, raw_acc_z, 0.5);
}
void first_order_filtering(void)
{
    int16 gx, gy, gz, ax, ay, az;
    //��ʱ�����洢                                                      
    gx = imu660ra_gyro_x;
    gy = imu660ra_gyro_y;
    gz = imu660ra_gyro_z;
    ax = imu660ra_acc_x;
    ay = imu660ra_acc_y;
    az = imu660ra_acc_z;
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
        stop=true;
    }

    az_last=az; //�����ϴμ��ٶȼ�Z������
}

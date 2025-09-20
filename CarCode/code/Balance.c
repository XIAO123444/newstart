#include "zf_common_headfile.h"
#include "Balance.h"
#include "PID.h"
#include "math.h"
uint8_t acc_ration=4;    //���ٶȼ����Ŷ�
uint8_t gyro_ration=4;   //���������Ŷ�
float filtering_angle=0; //������ĽǶ�
float angle_temp;        //�Ƕȼ����м����
float cycle_T=0.005;         //��������
float pitch_angle;          //�����
float roll_angle;           //������
imu_err_typdef imu_err={-6,3,0}; //imu���ṹ��
uint16 az_last=0; //�ϴμ��ٶȼ�Z������

extern bool stop;
void first_order_filtering(void)
{
    int16 gx,gy,gz,ax,ay,az;
    //��ʱ�����洢                                                      
    gx=imu660ra_gyro_x;
    gy=imu660ra_gyro_y;
    gz=imu660ra_gyro_z;                                                        
    ax=imu660ra_acc_x;
    ay=imu660ra_acc_y;
    az=imu660ra_acc_z;
    //ȥ����̬���
    gx+=imu_err.gx_err;
    gy+=imu_err.gy_err;
    gz+=imu_err.gz_err;
    if(gx<5 && gx>-5)
	{
		gx=0;
	}
	if(gy<5 && gy>-5)
	{
		gy=0;
	}
	if(gz<5 && gz>-5)
	{
		gz=0;
	} 
    //һ�׻����˲���Ƕ�
    float gyro_temp;
	float acc_temp;
	gyro_temp=gx*gyro_ration;
	acc_temp=(ay-angle_temp)*acc_ration;
	angle_temp+=((gyro_temp+acc_temp)*cycle_T);
	filtering_angle=angle_temp;                                                     
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

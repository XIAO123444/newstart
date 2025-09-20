#include "zf_common_headfile.h"
#include "Balance.h"
#include "PID.h"
#include "math.h"
uint8_t acc_ration=4;    //加速度计置信度
uint8_t gyro_ration=4;   //陀螺仪置信度
float filtering_angle=0; //解算出的角度
float angle_temp;        //角度计算中间变量
float cycle_T=0.005;         //采样周期
float pitch_angle;          //横滚角
float roll_angle;           //俯仰角
imu_err_typdef imu_err={-6,3,0}; //imu误差结构体
uint16 az_last=0; //上次加速度计Z轴数据

extern bool stop;
void first_order_filtering(void)
{
    int16 gx,gy,gz,ax,ay,az;
    //临时变量存储                                                      
    gx=imu660ra_gyro_x;
    gy=imu660ra_gyro_y;
    gz=imu660ra_gyro_z;                                                        
    ax=imu660ra_acc_x;
    ay=imu660ra_acc_y;
    az=imu660ra_acc_z;
    //去除静态误差
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
    //一阶互补滤波算角度
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
    //临时变量存储                                                      
    gx=imu660ra_gyro_x;
    gy=imu660ra_gyro_y;
    gz=imu660ra_gyro_z;                                                        
    ax=imu660ra_acc_x;
    ay=imu660ra_acc_y;
    az=imu660ra_acc_z;

    uint8 acc_abnormal=abs(az-az_last)>8000; //加速度异常标志
    if(acc_abnormal) //如果加速度异常或角度异常
    {//停车
        stop=true;
    }

    az_last=az; //保存上次加速度计Z轴数据
}

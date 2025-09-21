#include "zf_common_headfile.h"
#include "Balance.h"
#include "PID.h"
#include "math.h"
#include "math.h"
uint8_t acc_ration=4;    //加速度计置信度
uint8_t gyro_ration=4;   //陀螺仪置信度
float filtering_angle=0; //解算出的角度
float angle_temp;        //角度计算中间变量
float cycle_T=0.005;         //采样周期
float pitch_angle;          //横滚角
float roll_angle;           //俯仰角
imu_err_typdef imu_err={-6,3,-2}; //imu误差结构体
uint16 az_last=0; //上次加速度计Z轴数据
#include <math.h>

extern bool stop;
// 改进的低通滤波函数
void low_pass_filter(int16_t* filtered, int16_t new_data, float alpha) {
    *filtered = *filtered * (1 - alpha) + new_data * alpha;
}

// 改进的IMU滤波函数
void imu_filter(){
    // 去除静态误差（零偏校正）
    int16 raw_gyro_x = imu660ra_gyro_x + imu_err.gx_err;
    int16 raw_gyro_y = imu660ra_gyro_y + imu_err.gy_err;
    int16 raw_gyro_z = imu660ra_gyro_z + imu_err.gz_err;
    int16 raw_acc_x = imu660ra_acc_x;
    int16 raw_acc_y = imu660ra_acc_y;
    int16 raw_acc_z = imu660ra_acc_z;
    // 死区处理（消除微小抖动）
    if (fabs(raw_gyro_x) < 5) raw_gyro_x = 0;
    if (fabs(raw_gyro_y) < 5) raw_gyro_y = 0;
    if (fabs(raw_gyro_z) < 5) raw_gyro_z = 0;

    // 应用低通滤波
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
    //临时变量存储                                                      
    gx = imu660ra_gyro_x;
    gy = imu660ra_gyro_y;
    gz = imu660ra_gyro_z;
    ax = imu660ra_acc_x;
    ay = imu660ra_acc_y;
    az = imu660ra_acc_z;
    //一阶互补滤波算角度
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

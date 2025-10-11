#include "zf_common_headfile.h"
#include "Balance.h"
#include "PID.h"
#include "menu.h"
uint8_t acc_ration=4;    //加速度计置信度
uint8_t gyro_ration=4;   //陀螺仪置信度
float filtering_angle=0; //解算出的角度
float angle_temp;        //角度计算中间变量
float cycle_T=0.005;         //采样周期
int32 pitch_angle_integr=0;          //俯仰角积分
int32 roll_angle_integr=0;           //横滚角积分
int32 yaw_angle_integr=0;            //偏航角积分

int16 pitch_angle_integr_read;          //俯仰角菜单显示的数据
int16 roll_angle_integr_read;           //横滚角菜单显示的数据
int16 yaw_angle_integr_read;            //偏航角菜单显示的数据  

imu_err_typdef imu_err={-2,2,0}; //imu误差结构体
uint16 az_last=0; //上次加速度计Z轴数据
#include <math.h>
//来自 menu.h
extern car_mode carmode;  //车的状态
extern stop_debug stopdebug; //停车debug
extern enum_menu_mode menu_Mode;         //菜单模式


// 改进的低通滤波函数
void low_pass_filter(int16_t* filtered, int16_t new_data, float alpha) {
    *filtered = *filtered * (1 - alpha) + new_data * alpha;
}
int16 raw_gyro_x ;
int16 raw_gyro_y ;
int16 raw_gyro_z ;
int16 raw_acc_x ;
int16 raw_acc_y ;
int16 raw_acc_z ;
//初始化
void angle_init(void)
{
    pitch_angle_integr=0;
    roll_angle_integr=0;
    yaw_angle_integr=0;
}
// 改进的IMU滤波函数

void imu_filter(){
    // 去除静态误差（零偏校正）
    raw_gyro_x = imu660ra_gyro_x + imu_err.gx_err;      //横滚角
    raw_gyro_y = imu660ra_gyro_y + imu_err.gy_err;      //俯仰角
    raw_gyro_z = imu660ra_gyro_z + imu_err.gz_err;      //偏航角

    raw_acc_x = imu660ra_acc_x;
    raw_acc_y = imu660ra_acc_y;
    raw_acc_z = imu660ra_acc_z;
    // 死区处理（消除微小抖动）
    if (abs(raw_gyro_x) <= 5) raw_gyro_x = 0;
    if (abs(raw_gyro_y) <= 5) raw_gyro_y = 0;
    if (abs(raw_gyro_z) <= 5) raw_gyro_z = 0;
    pitch_angle_integr += raw_gyro_y; //俯仰角积分
    roll_angle_integr += raw_gyro_x;  //横滚角积分
    yaw_angle_integr += raw_gyro_z;   //偏航角积分

    pitch_angle_integr_read = pitch_angle_integr / 1000;        //俯仰角菜单显示的数据
    roll_angle_integr_read = roll_angle_integr / 1000;           //横滚角菜单显示的数据
    yaw_angle_integr_read = yaw_angle_integr / 30000;             //偏航角菜单显示的数据
    // 应用低通滤波
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
    //临时变量存储                                                      
    gx = raw_gyro_x;
    gy = raw_gyro_y;
    gz = raw_gyro_z;
    ax = raw_acc_x;
    ay = raw_acc_y;
    az = raw_acc_z;

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
        carmode=stop;
        stopdebug=liftup_stop;
        menu_Mode=stop_debug_display;           //菜单切换到停车显示
    }

    az_last=az; //保存上次加速度计Z轴数据
}

void gyro_protect(void)
{
    if( abs(pitch_angle_integr)>20000)//待填的坑
    {
        carmode=stop;
        stopdebug=gyro_intrg_pitch_stop;
        
    }
    if( abs(roll_angle_integr)>20000)//待填的坑
    {
        carmode=stop;
        stopdebug=gyro_intrg_roll_stop;
    }
    if( abs(yaw_angle_integr)>20000)//待填的坑
    {

        carmode=stop;
        stopdebug=gyro_intrg_yaw_stop;
        menu_Mode=stop_debug_display;           //菜单切换到停车显示

    }

}
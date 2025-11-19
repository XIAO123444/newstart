/*
 * IMU.c
 *
 *  Created on: 2025年9月22日
 *      Author: Hz
 */

#include "IMU.h"
#include "math.h"


#define sampleFreq  100.f

//采样周期的一半，用于求解四元数微分方程时计算角增量
#define halfT 0.005f

//用于控制加速度计修正陀螺仪积分姿态的速度的kp与ki
#define kp                  20.00f
#define ki                  0.001f

//采样周期
#define cycle_T         0.005f//200hz

//采样周期的一半，用于求解四元数微分方程时计算角增量
#define half_T          0.0025f

//初始单位四元数
float q[4]={1.0,0.0,0.0,0.0};

//初始化数据偏差
float exInt = 0.0 ,eyInt = 0.0 ,ezInt = 0.0 ;

//初始化中间计算变量
float q0temp;
float q1temp;
float q2temp;
float q3temp;

float imu_data_acc_x;
float imu_data_acc_y;
float imu_data_acc_z;

float imu_data_gyro_x;
float imu_data_gyro_y;
float imu_data_gyro_z;

//初始化各轴角度数据
IMU_Angle_typedef imu_angle;

//初始化各轴初始零漂数据
float gyro_drift_x;
float gyro_drift_y;
float gyro_drift_z;

//原始yaw角
float yaw;

//imu初始化
void imu_init(void)
{
    while(1)
    {
       if(imu660ra_init())
           printf("\r\n IMU660RA init error.");                                 // IMU660RA 初始化失败
       else
           break;
    }
    ips200_show_string(0, 0,"IMU660RA init success");
    system_delay_ms(300);
    ips200_clear();
}

float fast_sqrt(float x)//平方根倒数速算法
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//陀螺仪数据初始化，去除零点漂移
void imu_data_processing(void)
{
    ips200_show_string(0, 20,"IMU660RA drift data processing");

    gyro_drift_x=0;
    gyro_drift_y=0;
    gyro_drift_z=0;

    for(uint16 i=0;i<1000;i++)//采样1000次
    {
        gyro_drift_x+=imu660ra_gyro_x;
        gyro_drift_y+=imu660ra_gyro_y;
        gyro_drift_z+=imu660ra_gyro_z;
    }

    gyro_drift_x/=1000;
    gyro_drift_y/=1000;
    gyro_drift_z/=1000;
    ips200_clear();
}
void gyro_drift_Set0(void)
{
    gyro_drift_x=0;
    gyro_drift_y=0;
    gyro_drift_z=0;
}

//使用加速度以及角速度
void imu_transform(void)//将原始数据转化为四元数
{
    mpu6050_get_acc();
    mpu6050_get_gyro();

    float alpha = 0.3;
    imu_data_acc_x = (imu660ra_acc_transition(((float) imu660ra_acc_x) * alpha)) + imu_data_acc_x * (1 - alpha);
    imu_data_acc_y = (imu660ra_acc_transition(((float) imu660ra_acc_y) * alpha)) + imu_data_acc_y * (1 - alpha);
    imu_data_acc_z = (imu660ra_acc_transition(((float) imu660ra_acc_z) * alpha)) + imu_data_acc_z * (1 - alpha);

    //陀螺仪角速度必须转换为弧度制角速度: deg/s -> rad/s
    imu_data_gyro_x = imu660ra_gyro_transition(((float) imu660ra_gyro_x)) * PI / 180;
    imu_data_gyro_y = imu660ra_gyro_transition(((float) imu660ra_gyro_y)) * PI / 180;
    imu_data_gyro_z = imu660ra_gyro_transition(((float) imu660ra_gyro_z)) * PI / 180;

    float ax,ay,az;
    float gx,gy,gz;

    //获取转化后的陀螺仪数据以及加速度计数据
    ax=imu_data_acc_x;
    ay=imu_data_acc_y;
    az=imu_data_acc_z;
    gx=imu_data_gyro_x;
    gy=imu_data_gyro_y;
    gz=imu_data_gyro_z;

    //姿态数据
    float vx, vy, vz;
    float ex, ey, ez;

    //取出四元数参数
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    //处于自由落体状态时不进行姿态解算，因为会产生分母无穷大的情况
    if( (ax == 0.0f) && (ay == 0.0f) && (az == 0.0f) )
           return;

    //加速度归一化用于计算误差
    float norm = fast_sqrt(ax*ax+ay*ay+az*az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    //提取利用角速度算出的姿态矩阵中的物体坐标系下的重力分量*
    vx = 2 * (q1*q3 - q0*q2);
    vy = 2 * (q2*q3 + q0*q1);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    //将加速度计获得的重力归一化数据与姿态矩阵的重力向量叉乘获取姿态误差
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    //PI互补滤波
    exInt += ki * ex;
    eyInt += ki * ey;
    ezInt += ki * ez;
    gx += kp * ex + exInt;
    gy += kp * ey + eyInt;
    gz += kp * ez + ezInt;

    //暂存当前值用于计算
    q0temp=q0;
    q1temp=q1;
    q2temp=q2;
    q3temp=q3;

    /*四元数参数迭代*/
    q0 = q0temp + (-q1temp*gx - q2temp*gy -q3temp*gz)*half_T;
    q1 = q1temp + (q0temp*gx + q2temp*gz -q3temp*gy)*half_T;
    q2 = q2temp + (q0temp*gy - q1temp*gz +q3temp*gx)*half_T;
    q3 = q3temp + (q0temp*gz + q1temp*gy -q2temp*gx)*half_T;

    /*四元数归一化*/
    norm = fast_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q[0] = q0 * norm;
    q[1] = q1 * norm;
    q[2] = q2 * norm;
    q[3] = q3 * norm;
}



//仅仅使用角速度
void imu_transform_gyro(void)
{
    imu660ra_get_acc();
    imu660ra_get_gyro();

    // 1. 陀螺仪数据转换为「弧度制角速度」
    float gx = imu660ra_gyro_transition((imu660ra_gyro_x-gyro_drift_x)/10*10) * PI / 180 ;
    float gy = imu660ra_gyro_transition((imu660ra_gyro_y-gyro_drift_y)/10*10) * PI / 180 ;
    float gz = imu660ra_gyro_transition((imu660ra_gyro_z-gyro_drift_z)/10*10) * PI / 180 ;

    if(gx<=0.006&&gx>=-0.006)gx=0;
    if(gy<=0.006&&gy>=-0.006)gy=0;
    if(gz<=0.006&&gz>=-0.006)gz=0;

    // 2. 暂存当前四元数（用于一阶龙格-库塔积分）
    float q0 = q[0];
    float q1 = q[1];
    float q2 = q[2];
    float q3 = q[3];

    // 3. 四元数微分方程积分（核心：q_dot = 0.5 * q × ω，ω为角速度纯四元数）
    float dt = cycle_T; // 采样时间（秒）
    q[0] += (-q1*gx - q2*gy - q3*gz) * 0.5f * dt;
    q[1] += (q0*gx + q2*gz - q3*gy) * 0.5f * dt;
    q[2] += (q0*gy - q1*gz + q3*gx) * 0.5f * dt;
    q[3] += (q0*gz + q1*gy - q2*gx) * 0.5f * dt;

    // 4. 四元数归一化（防止数值漂移，确保为「单位四元数」）
    float norm = sqrtf(q[0]*q[0] +
                       q[1]*q[1] +
                       q[2]*q[2] +
                       q[3]*q[3]);
    if (norm > 0.0f)
    { // 避免除零错误
        norm = 1.0f / norm;
        q[0] *= norm;
        q[1] *= norm;
        q[2] *= norm;
        q[3] *= norm;
    }
}

// 将-180~180度循环的yaw角转换为0~360度循环（顺时针递增，逆时针递减）
float yaw_to_360(float current_raw) {
    static float prev_raw = 0.0f;       // 上一次的原始角度（-180~180）
    static float converted = 0.0f;      // 转换后的角度（0~360）
    static bool is_first = true;        // 首次调用标志

    // 初始化：首次调用时同步原始角度与转换角度
    if (is_first) {
        prev_raw = current_raw;
        converted = (current_raw < 0) ? (current_raw + 360.0f) : current_raw;
        is_first = false;
        return converted;
    }

    // 1. 计算原始角度的变化量（处理跳变）
    float delta_raw = current_raw - prev_raw;
    // 当原始角度跳变（如170°→-170°，实际是逆时针转了20°）
    if (delta_raw > 180.0f) {
        delta_raw -= 360.0f;  // 修正为负向变化（逆时针）
    } else if (delta_raw < -180.0f) {
        delta_raw += 360.0f;  // 修正为正向变化（顺时针）
    }

    // 2. 根据变化量更新转换后的角度
    //  - 顺时针转：delta_raw为负 → converted递增（0→360）
    //  - 逆时针转：delta_raw为正 → converted递减（360→0）
    converted -= delta_raw;  // 核心映射：用减法实现方向反转

    // 3. 约束角度在0~360范围（处理循环）
    if (converted > 360.0f) {
        converted -= 360.0f;
    } else if (converted < 0.0f) {
        converted += 360.0f;
    }

    // 4. 保存当前原始角度，用于下一次计算
    prev_raw = current_raw;

    return converted;
}


//获取各轴角度数据
void imu_getangle(void)
{
    imu_transform_gyro();
    imu_angle.pitch=asinf(-2* q[1]*q[3]+2*q[0]*q[2])*57.2957;
    imu_angle.roll=atan2f(2*q[2]*q[3]+2*q[0]*q[1],-2*q[1]*q[1]-2*q[2]*q[2]+1)*57.2957;
    yaw=atan2f(2*q[1]*q[2]+2*q[0]*q[3],-2*q[2]*q[2]-2*q[3]*q[3]+1)*57.2957;
    imu_angle.yaw=yaw;
}





#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

typedef struct {
    float roll;
    float pitch;
    float yaw;
}IMU_Angle_typedef;


extern IMU_Angle_typedef imu_angle;

extern float yaw;

void imu_init(void);

void imu_getangle(void);

void imu_data_processing(void);

void gyro_drift_Set0(void);

#endif

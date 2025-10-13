#ifndef __BALANCE_H
#define __BALANCE_H
#include "zf_common_headfile.h"
typedef struct {
    int8 gx_err;
    int8 gy_err;
    int8 gz_err;
}imu_err_typdef;
void first_order_filtering(void);
void imu_filter();
void lift_protection(void);
void angle_init(void);
#endif
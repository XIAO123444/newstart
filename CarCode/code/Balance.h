#ifndef __BALANCE_H
#define __BALANCE_H
typedef struct {
    uint8 gx_err;
    uint8 gy_err;
    uint8 gz_err;
}imu_err_typdef;
void first_order_filtering(void);
void lift_protection(void);
#endif
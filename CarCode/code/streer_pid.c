#include "steer_pid.h"
#include "photo_chuli.h"
#include "track.h"
#include "PID.h"
#include "menu.h"
int32 gyro_z,gyro_y;


extern PID_t PID_steer;
void S_PID_CAL()
{
    PID_steer.actual=(float)output_middle4(); 
    gyro_z=-imu660ra_gyro_z;
	PID_gyro_update(&PID_steer,gyro_z);
}

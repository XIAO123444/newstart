#ifndef __INS_H
#define __INS_H
#include "zf_common_headfile.h"


typedef struct 
{   
    int32 intg_encoder;        //  记录编码器   都得是int型
    int32 intg_gyro_yaw;       //  记录偏航角   都得是int型 
    
}struct_INS;

#endif
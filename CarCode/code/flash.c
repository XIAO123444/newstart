#include "flash.h"
#include "steer_pid.h"
#include "menu.h"

extern struct pid_v PID_V;
extern struct steer_pid S_PID;
extern struct steer_pid S_PID1;
extern int speed;
extern int forwardsight;        //前瞻1，处理转向
extern int forwardsight2;       //前瞻2，提前看弯道
extern int forwardsight3;       //前瞻3，弯道前瞻
extern int16 threshold_up;       //大津法阈值上限
extern int16 threshold_down;     //大津法阈值下限
void flash_init(void)
{
//     flash_buffer_clear();
//     flash_read_page_to_buffer(100, 0);                            // 将数据从 flash 读取到缓冲区

//     flash_buffer_clear();
//     flash_read_page_to_buffer(100, 1);                            // 将数据从 flash 读取到缓冲区
//     S_PID.p                =flash_union_buffer[0].float_type;
//     S_PID.i                =flash_union_buffer[1].float_type;
//     S_PID.d                =flash_union_buffer[2].float_type;
//     S_PID.outputmax        =flash_union_buffer[3].float_type;
//     S_PID.outputmin        =flash_union_buffer[4].float_type;
//     flash_buffer_clear();
//     flash_read_page_to_buffer(100, 2);                            // 将数据从 flash 读取到缓冲区
//     speed                  =flash_union_buffer[0].int32_type;
//     forwardsight            =flash_union_buffer[1].int32_type;
//     forwardsight2            =flash_union_buffer[2].int32_type;
//     forwardsight3            =flash_union_buffer[3].int32_type;
//     threshold_up            =flash_union_buffer[4].int32_type;
//     threshold_down          =flash_union_buffer[5].int32_type;
//     flash_buffer_clear();
//     flash_read_page_to_buffer(99, 0);                            // 将数据从 flash 读取到缓冲区
//     S_PID1.p                =flash_union_buffer[0].float_type;  
//     S_PID1.i                =flash_union_buffer[1].float_type;
//     S_PID1.d                =flash_union_buffer[2].float_type;
//     S_PID1.outputmax        =flash_union_buffer[3].float_type;
//     S_PID1.outputmin        =flash_union_buffer[4].float_type;
//     flash_buffer_clear();
//     flash_read_page_to_buffer(99, 1);                            // 将数据从 flash 读取到缓冲区
    
}

#include "flash.h"
#include "steer_pid.h"
#include "menu.h"
#include "pid.h"
#include "BLDC.h"



extern int speed;

extern int16 threshold_up;       //大津法阈值上限
extern int16 threshold_down;     //大津法阈值下限

extern int16 forwardsight; //默认前瞻
extern int16 forwardsight2;//  直到判断前瞻！！！！注意这个和前瞻不同，用于三轮或者四轮车加速的！！！
extern int16 forwardsight3;//弯道前瞻

extern PID_t PID_gyro;          //角速度环
extern PID_t PID_angle;         //角度环
extern PID_t PID_speed;         //速度环  
extern PID_t PID_steer;         //转向环
extern PID_t PID_BLDC;          //负压风扇环

extern BLDC_Param bldc_param;
void flash_reset(void)
{
    flash_erase_page(100, 0);                                 // 擦除这一页
    flash_erase_page(100, 1);                                 // 擦除这一页
    flash_erase_page(100, 2);                                 // 擦除这一页
    flash_erase_page(99, 0);                                  // 擦除这一页
    
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     保存参数到 FLASH配置1
// 参数说明     void
// 返回参数     void
// 使用示例     flash_save_config1();
// 备注信息     0是default 1,2,3,4分别对应（100,1）（100,2）（100,3）（101,0）
//-------------------------------------------------------------------------------------------------------------------
void flash_save_config(int16 i)
{

        if(flash_check(100+i/4, i%4)){flash_erase_page(100+i/4, i%4);}
        flash_buffer_clear();
        //只有转向环d2
    
        // 100,0   
        //角速度环
        flash_union_buffer[0].float_type = PID_gyro.kp;
        flash_union_buffer[1].float_type = PID_gyro.ki;    
        flash_union_buffer[2].float_type = PID_gyro.kd;
        flash_union_buffer[3].float_type = PID_gyro.maxout;    
        flash_union_buffer[4].float_type = PID_gyro.minout;
        //角度环
        flash_union_buffer[5].float_type = PID_angle.kp;
        flash_union_buffer[6].float_type = PID_angle.ki;
        flash_union_buffer[7].float_type = PID_angle.kd;
        flash_union_buffer[8].float_type = PID_angle.maxout;
        flash_union_buffer[9].float_type = PID_angle.minout;
        //速度环
        flash_union_buffer[10].float_type = PID_speed.kp;
        flash_union_buffer[11].float_type = PID_speed.ki;
        flash_union_buffer[12].float_type = PID_speed.kd;
        flash_union_buffer[13].float_type = PID_speed.maxout;
        flash_union_buffer[14].float_type = PID_speed.minout;
        flash_union_buffer[24].float_type=  PID_speed.targ;
        //转向环
        flash_union_buffer[15].float_type = PID_steer.kp;
        flash_union_buffer[16].float_type = PID_steer.ki;
        flash_union_buffer[17].float_type = PID_steer.kd;
        flash_union_buffer[18].float_type = PID_steer.kd2;
        flash_union_buffer[19].float_type = PID_steer.maxout;
        flash_union_buffer[20].float_type = PID_steer.minout;
        flash_union_buffer[25].float_type = PID_steer.kd2;
        //转向环6个参数
        flash_union_buffer[26].float_type = PID_BLDC.kp;
        flash_union_buffer[27].float_type = PID_BLDC.ki;    
        flash_union_buffer[28].float_type = PID_BLDC.kd;
        flash_union_buffer[29].float_type = PID_BLDC.maxout;
        flash_union_buffer[30].float_type = PID_BLDC.minout;
        flash_union_buffer[31].float_type = PID_BLDC.kd2;

        //负压风扇环5个参数
        //PID21个参数
        flash_union_buffer[21].int16_type = forwardsight;
        flash_union_buffer[22].int16_type = forwardsight2;
        flash_union_buffer[23].int16_type = forwardsight3;
        //前瞻3个参数
        flash_union_buffer[24].int16_type = bldc_param.basic_duty;
        flash_union_buffer[25].int16_type = bldc_param.encoder_p;
        flash_union_buffer[26].int16_type = bldc_param.max_output;
        flash_union_buffer[27].int16_type = bldc_param.min_output;



        flash_write_page_from_buffer(100+i/4, i%4);        //flash写

}
void flash_save_config_default(void) { flash_save_config(0); }          //默认分区保存
void flash_save_config_1(void) { flash_save_config(1); }
void flash_save_config_2(void) { flash_save_config(2); }
void flash_save_config_3(void) { flash_save_config(3); }
void flash_save_config_4(void) { flash_save_config(4); }
void flash_load_config(int16 i)
{
    flash_buffer_clear();                                               //缓冲区清理
    flash_read_page_to_buffer(100+i/4, i%4);                            // 将数据从 flash 读取到缓冲区
    PID_gyro.kp = flash_union_buffer[0].float_type;
    PID_gyro.ki = flash_union_buffer[1].float_type;
    PID_gyro.kd = flash_union_buffer[2].float_type;
    PID_gyro.maxout = flash_union_buffer[3].float_type;
    PID_gyro.minout = flash_union_buffer[4].float_type;
    
    // 角度环参数读取
    PID_angle.kp = flash_union_buffer[5].float_type;
    PID_angle.ki = flash_union_buffer[6].float_type;
    PID_angle.kd = flash_union_buffer[7].float_type;
    PID_angle.maxout = flash_union_buffer[8].float_type;
    PID_angle.minout = flash_union_buffer[9].float_type;
    
    // 速度环参数读取
    PID_speed.kp = flash_union_buffer[10].float_type;
    PID_speed.ki = flash_union_buffer[11].float_type;
    PID_speed.kd = flash_union_buffer[12].float_type;
    PID_speed.maxout = flash_union_buffer[13].float_type;
    PID_speed.minout = flash_union_buffer[14].float_type;
    PID_speed.targ=flash_union_buffer[24].float_type;//24
    
    // 转向环参数读取
    PID_steer.kp = flash_union_buffer[15].float_type;
    PID_steer.ki = flash_union_buffer[16].float_type;
    PID_steer.kd = flash_union_buffer[17].float_type;
    PID_steer.kd2 = flash_union_buffer[18].float_type; // 注意：保存时有kd2，读取也应有
    PID_steer.maxout = flash_union_buffer[19].float_type;
    PID_steer.minout = flash_union_buffer[20].float_type;
    PID_steer.kd2=flash_union_buffer[25].float_type;
    // 转向环6个参数读取
    PID_BLDC.kp = flash_union_buffer[26].float_type;
    PID_BLDC.ki = flash_union_buffer[27].float_type;
    PID_BLDC.kd = flash_union_buffer[28].float_type;
    PID_BLDC.maxout = flash_union_buffer[29].float_type;
    PID_BLDC.minout = flash_union_buffer[30].float_type;
    PID_BLDC.kd2 = flash_union_buffer[31].float_type;

    //PID21个参数读取

    forwardsight=flash_union_buffer[21].int16_type;
    forwardsight2=flash_union_buffer[22].int16_type;
    forwardsight3=flash_union_buffer[23].int16_type;
    //前瞻3个参数读取
    bldc_param.basic_duty=flash_union_buffer[24].int16_type;
    bldc_param.encoder_p=flash_union_buffer[25].int16_type;
    bldc_param.max_output=flash_union_buffer[26].int16_type;
    bldc_param.min_output=flash_union_buffer[27].int16_type;        
    //bldc4个参数读取


} 
void flash_load_config_default(void) { flash_load_config(0); }      //默认分区加载
void flash_load_config_1(void) { flash_load_config(1); }
void flash_load_config_2(void) { flash_load_config(2); }
void flash_load_config_3(void) { flash_load_config(3); }
void flash_load_config_4(void) { flash_load_config(4); }
#include "speed.h"
#include "track.h"
#include "photo_chuli.h"
#include "menu.h"
#include "steer_pid.h"

extern bool start_flag;
extern float error;
extern int32 speed;             //基础速度
extern int32 speed_stragety;    //决策速度
extern int32 forwardsight;
extern int32 forwardsight2;
extern int32 forwardsight3;
extern int32 forwardsight_stragety;
bool addspeed_flag;
uint8 car_situation=0; 

void Velocity_Control(void)//赛道类型判别，来选定速度 
{ 
    
    if(start_flag==true)
    {
        addspeed_flag=output_addspeedflag(); //判断是否加速
        if((error<6||error>-6)&&addspeed_flag==true)
        {
            car_situation=0;//弯道 
        }  
        else 
        { 
            car_situation=1;
        }   
        if(car_situation==0)//直道 
        { 
            speed_stragety=speed*1.5 ; 
            forwardsight_stragety=forwardsight;
            
        }
        if(car_situation==1)
        {  
            speed_stragety=speed ; 
            forwardsight_stragety=forwardsight3;
        }
        
          
    }

}

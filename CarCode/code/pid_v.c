 #include "zf_common_headfile.h"
#include "encoder.h"

 float ERR=0.0,LastERR=0.0,LastLastERR=0.0;
 float pwm_ccr=0,add_ccr=0;
 float i1,d1;

 

  struct pid_v
{
    float output_max;       //输出限幅   
    float p;            //比例系数
    float i;            //积分系数
    float d;            //微分系数
    float i_max;        //积分限幅
    float d_max;        //微分限幅
    
}PID_V;
void pid_vparam_init(void)
{
    PID_V.d=0;
    PID_V.d_max=0.1;
    PID_V.i=1.8;
    PID_V.i_max=10;
    PID_V.output_max=100;
    PID_V.p=60;
}

struct pid_v* PID_vget_param(void)
{
    return &PID_V;
}
void PID_vset_param(struct pid_v* p)
{
    PID_V.p         =p          ->p;
    PID_V.d         =p          ->d;
    PID_V.d_max     =p          ->d_max;
    PID_V.i         =p          ->i;
    PID_V.i_max     =p          ->i_max;
    PID_V.output_max=p          ->output_max;

}

void pidv_init(void)
{
    ERR=0.0;
    LastERR=0.0;
    i1=0.0;
    d1=0.0;

}

int pid_V_comon(int target)
{
    ERR=(float)(target*2)-(float)(Encoder_GetInfo_L()+Encoder_GetInfo_R());
    d1=ERR-LastERR;
    i1+=ERR;
    if(i1>=4000)
    {
        i1=4000;
    }
    if(i1<=-4000)
    {
        i1=-4000;
    }

    int result=(int)(PID_V.p*ERR+PID_V.i*i1+PID_V.d*d1);
        if(result>=5000)
    {
        result=5000;
    }    
        if(result<=-5000)
    {
        result=-5000;
    }
    return result;
}




void pid_add_p(void){
    
    PID_V.p+=0.1;
}
void pid_add_i(void) {
    PID_V.i += 0.1;  


}
void pid_add_d(void) {
    PID_V.d += 0.1;  

}    
void pid_add_i_max(void) {
    PID_V.i_max += 0.1; 
}
void pid_add_d_max(void) {
    PID_V.d_max += 0.1;
}
void pid_add_output_max(void) {
    PID_V.output_max += 1.0;
}
void pid_sub_p(void) {
    if (PID_V.p > 0.1) {  
        PID_V.p -= 0.1;   
        if (PID_V.p < 0) PID_V.p = 0;  
    }
}
void pid_sub_i(void) {
    if (PID_V.i > 0.1) {
        PID_V.i -= 0.1;
        if (PID_V.i < 0) PID_V.i = 0;
    }
}
void pid_sub_d(void) {
    if (PID_V.d > 0.1) {
        PID_V.d -= 0.1;
        if (PID_V.d < 0) PID_V.d = 0;
        // 微分限幅保护
        }
}

void pid_sub_d_max(void) {
    if (PID_V.d_max > 0.1) {
        PID_V.d_max -= 0.1;
        if (PID_V.d_max < 0) PID_V.d_max = 0;
    }
}
void pid_sub_i_max(void) {
    if (PID_V.i_max > 0.1) {
        PID_V.i_max -= 0.1;
        if (PID_V.i_max < 0) PID_V.i_max = 0;
    }
}
void pid_sub_output_max(void) {
    if (PID_V.output_max > 0.1) {  // 输出限幅通常需大于执行机构最小需求
        PID_V.output_max -= 0.1;
    }
}








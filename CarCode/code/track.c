
#include "track.h"
#include "photo_chuli.h"
#include "buzzer.h"
#include "speed.h"

// 外部变量声明
uint8 cross_flag=0;


extern int32 forwardsight3;
extern int16 centerline[MT9V03X_H];      // 中心线数组（图像高度维度）
extern int16 leftline[MT9V03X_H];       // 左边界线数组 
extern int16 rightline[MT9V03X_H];      // 右边界线数组
extern int16 rightfollowline[MT9V03X_H]; // 右边界跟踪线
extern int16 leftfollowline[MT9V03X_H];  // 左边界跟踪线
extern uint8 pix_per_meter;             // 像素/米比例系数
int16 centerline2[MT9V03X_H];           // 二次计算的中心线

extern int16 boundry_start_left; //左边界起始点
extern int16 boundry_start_right; //右边界起始点

extern int16 leftlostpoint[2];   //左丢线数和左丢线点0为丢线数，1为丢线点
extern int16 rightlostpoint[2];  //右丢线数和左丢线点0为丢线数，1为丢线点
extern int16 bothlostpoint[2];   //同时丢线数和左丢线点0为丢线数，1为丢线

extern int16 search_stop; // 终止点
extern int16 search_stop1; // 终止点1
// 边界点检测结果
extern int16 Right_Down_Find;  // 右下边界点行号
extern int16 Left_Down_Find;   // 左下边界点行号
extern int16 Right_Up_Find;    // 右上边界点行号
extern int16 Left_Up_Find;     // 左上边界点行号
int16 lastrightupfind=0; // 上次右上点
int16 lastrightdownfind=0; // 上次右下点
int16 lastleftupfind=0; // 上次左上点
int16 lastleftdownfind=0; // 上次左下点
extern int16 right_budandiao;       // 右不单调点
extern int16 left_budandiao;        // 左不单调点
int16 v_point;


extern int16 right_down_guai[2];   // 右下拐点
extern int16 right_up_guai[2];     // 右上拐点 
extern int16 left_down_guai[2];    // 左下拐点
extern int16 left_up_guai[2];      // 左上拐点


extern int16 left_start_point;  //左起点
extern int16 right_start_point; //右起点

int16 continuity_pointLeft[2]={0,0}; // 左不连续点[0]存某行，[1]存某列
int16 continuity_pointRight[2]={0,0}; // 右不连续点 [0]存某行，[1]存某列


//加权控制
const uint8 Weight[MT9V03X_H]=
{
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,             
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,           
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 6, 6, 6, 7, 7, 10, 10, 10,

        15, 15, 20, 20, 20, 25, 25, 20, 20, 20,
        15, 15, 10, 10, 10, 7, 7, 6, 6, 6,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,         
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1 
    
};

// 圆环标志
float right_dxbudandiao;            // 右不单调点斜率

extern uint8 leftline_num;         //左线点数量
extern uint8 rightline_num;        //右线点数量

extern int32 forwardsight2;
extern int32 forwardsight_stragety;

uint8 crossconfirm=0;

int16 output_middle2(void) {
    int16 result;
    if(search_stop<forwardsight_stragety)            //如果终止点远于 前视距离
    {
        result=centerline2[forwardsight_stragety];   
        return result;                      
        
    }

    result=centerline2[search_stop];
    return centerline2[search_stop];
} 
 
float output_middle3(void) {

   
    int i;
    float err=0;
    float weight_count=0;
    //常规误差
    for(i=MT9V03X_H-1;i>=MT9V03X_H-search_stop-1;i--)//常规误差计算
    {
        err+=(MT9V03X_W/2-((leftfollowline[i]+rightfollowline[i])>>1))*Weight[i];//右移1位，等效除2
        weight_count+=Weight[i];
    }
    err=err/weight_count;
    return err;//注意此处，误差有正负，还有小数，注意数据类型


} 

int16 output_middle4(void) 
{
    int16 result; 
    if(search_stop1>MT9V03X_H-1-3)
    {
        return MT9V03X_W/2;         //防止数组越界
    }
    
    if(search_stop1<forwardsight3)            //如果终止点远于 前视距离
    {
        result=(centerline2[forwardsight3]+centerline2[forwardsight3+1]+centerline2[forwardsight3+2])/3;        // 取前视距离的平均值 
        return result;                      
        
    }

    result=(centerline2[search_stop1]+centerline2[search_stop1+1]+centerline2[search_stop1+2])/3;          // 取终止点的平均值                  
    return result; // 返回终止点的平均值
} 

enum mark {
    straight,    // 直道行驶
    crossroad,   // 十字路口
    crossroadL, // 左斜入十字
    crossroadR, // 右斜入十字
    round_1,
    round_2,   // 入环补直线a
    round_3,   // 圆环补斜线（未使用）
    round_4,   // 入环行驶
    round_5,   // 左拐点补斜线
    round_6,    // 出环补直线
    round_7    // 出环补斜线
};
enum mark carstatus_now = straight;  // 当前车辆状态

bool output_addspeedflag(void)
{
    if (search_stop1<forwardsight2&&abs(centerline2[forwardsight2]-MT9V03X_W/2)<10&&carstatus_now!=round_1&&carstatus_now!=round_2&&carstatus_now!=round_3&&carstatus_now!=round_4&&carstatus_now!=round_5&&carstatus_now!=round_6&&carstatus_now!=round_7) // 前视距离小于终止点且中心线接近中线
     // 说明前方道路平坦，可以加速
     // 注意：这里的15是一个经验值，可以根据实际情况调整
    { 
        return true; // 满足加速条件
    } else
     {
        return false; // 不满足加速条件
    }

    
}

int32 encodercounter=0;



void centerline2_change(void) {
    for(int16 i=MT9V03X_H-1; i>search_stop; i--) {
                centerline2[i] = (rightfollowline[i] + leftfollowline[i]) / 2;

    }
    for(int16 i=search_stop;i>=0;i--)
    {
        centerline2[i]=MT9V03X_W/2;
    }
}

void element_check(void) {     
    // 更新左右跟踪线 
    memcpy(leftfollowline, leftline, sizeof(leftline));
    memcpy(rightfollowline, rightline, sizeof(rightline));


    centerline2_change();
    v_point=find_vpoint(5,MT9V03X_W-5); // 查找v点
    continuity_pointLeft[0]=continuity_left(MT9V03X_H-1,search_stop+2); // 左连续性判断
    continuity_pointRight[0]=continuity_right(MT9V03X_H-1,search_stop+2); // 右连续性判断
    continuity_pointLeft[1]=leftline[continuity_pointLeft[0]]; // 左连续性点列
    continuity_pointRight[1]=rightline[continuity_pointRight[0]]; // 右连续性点列
    Find_Up_Point(MT9V03X_H-1, search_stop); // 查找上半段边界点
    Find_Down_Point(MT9V03X_H-1, search_stop); //查找下半段边界点
    if(Left_Down_Find <= Left_Up_Find) {Left_Down_Find = 0;}
    if(Right_Down_Find <= Right_Up_Find){ Right_Down_Find = 0;}
    left_budandiao=montonicity_left(MT9V03X_H-1,search_stop+6); // 左不单调点
    right_budandiao=montonicity_right(MT9V03X_H-1,search_stop+6); // 右不单调点

////    /*---------- 直道状态检测 ----------*/
    if(carstatus_now == straight) 
    {

        //十字判断
        // if(search_stop<13)
        // {
        //     if(continuity_pointLeft[0] != 0 && continuity_pointRight[0] != 0 && Right_Up_Find != 0 && Left_Up_Find != 0&&(Right_Up_Find>search_stop-2&&Left_Up_Find>search_stop-2))//左不连续点找到 且右不连续点找到，且左上拐点找到且右上拐点找到，此时为正入十字
        //     {
        //         carstatus_now = crossroad; // 进入十字路口状态
        //         return;
        //     }
        //     if(continuity_pointLeft[0] != 0&& Left_Up_Find != 0&&Left_Up_Find>search_stop-2&&rightlostpoint[0]>50&&abs(left_budandiao-Left_Up_Find)>5)//左不连续点找到 且右不连续点未找到，且左上拐点找到，此时为左斜入十字后续加入左丢线点
        //     {
        //         carstatus_now = crossroadL; // 进入十字路口状态
        //         return; 
        //     }

        //     if( continuity_pointRight[0] != 0 && Right_Up_Find != 0&&Right_Up_Find>search_stop-2&&leftlostpoint[0]>50)//左不连续点未找到 右不连续点找到，且右上拐点找到，此时为右斜入十字
        //     {
        //         carstatus_now = crossroadR; // 进入十字路口状态
        //         return; 
        //     }
        // }
        // //圆环判断
        // if(right_budandiao&&Right_Down_Find>35&&rightlostpoint[0]>10&&rightlostpoint[0]<80&&search_stop<3&&leftlostpoint[0]<10&&Left_Up_Find==0)
        // //右不单调点存在，右下拐点存在，右丢线点数量在10-40之间，左丢线点小于10，且搜索终止点小于10
        // {
        //     carstatus_now=round_1; // 进入入环补直线状态
        //     BUZZ_START();
        //     return;
        // }

        // //ips200_show_string(0,300,"straig");
    }

//    /*---------- 十字路口状态处理 ----------*/
  if(carstatus_now == crossroad) {
       int start_down_point=5;
       int16 temp1_L=0;//记录第一个左拐点
       int16 temp1_R=0;//记录第一个右拐点 

      //        // 确定下半段边界点（取左右下点


      /* 边界线拟合策略 */
      if(Left_Down_Find != 0 && Right_Down_Find != 0) {
          // 情况1：左右下点均有效 → 双边界直线拟合
          add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
                     Right_Up_Find-2, rightline[Right_Up_Find-2]);        // 右边界拟合
          add_Lline_k(leftline[Left_Down_Find], Left_Down_Find,   
                     Left_Up_Find-2, leftline[Left_Up_Find-2]);           // 左边界拟合
        //   //ips200_show_string(0,300,"cross1");
        //    printf("cross1");
      }
      else if(Left_Down_Find == 0 && Right_Down_Find != 0) {
          // 情况2：仅右下点有效 → 右边界拟合+左边界延长
          add_Rline_k(rightline[Right_Down_Find], Right_Down_Find,        // 右边界拟合
                     Right_Up_Find, rightline[Right_Up_Find]);
          lenthen_Left_bondarise(Left_Up_Find);                       //
        //   //ips200_show_string(0,300,"cross2");
      }
      else if(Left_Down_Find != 0 && Right_Down_Find == 0) {
          // 情况3：仅左下点有效 → 左边界拟合+右边界延长
          lenthen_Right_bondarise(Right_Up_Find);
          add_Lline_k(leftline[Left_Down_Find], Left_Down_Find, 
                     Left_Up_Find, leftline[Left_Up_Find]);
        //    printf("cross3");
            //ips200_show_string(0,300,"cross3");
      }
      else {
          // 情况4：无有效下点 → 双边界延长
          lenthen_Right_bondarise(Right_Up_Find);
          lenthen_Left_bondarise(Left_Up_Find);
        //    printf("cross4");
        ////ips200_show_string(0,300,"cross4");
      }

      // 异常处理：突变点失效时恢复原始边界
      if(Right_Up_Find == 0) memcpy(rightfollowline, rightline, sizeof(rightline));
      if(Left_Up_Find == 0) memcpy(leftfollowline, leftline, sizeof(leftline));
      centerline2_change();
      if(Right_Up_Find==0||Left_Up_Find==0)
          {
          carstatus_now = straight;
          return;
      }
      
  }
  if(carstatus_now == crossroadL) {

              //ips200_show_string(0,300,"crossL");


    if(Left_Up_Find&&Right_Up_Find)                                     //如果左上点和右上点都有效
    {
        carstatus_now = crossroad;                                      // 进入十字路口状态
        return;
    }
    if(Left_Up_Find!=0&&Left_Down_Find!=0)                              //如果找到了左上点和左下点
    {
        add_Lline_k(leftline[Left_Down_Find], Left_Down_Find,   
        Left_Up_Find, leftline[Left_Up_Find]);           
        centerline2_change();
    }
    if(Left_Up_Find!=0&&Left_Down_Find==0)                              //如果只找到了左上点
    {
        lenthen_Left_bondarise(Left_Up_Find);                           //延长左边界
        centerline2_change();

    }
    if(Left_Up_Find==0)
    {
        carstatus_now = straight;                                      // 如果左上点未找到，返回直道状态
    }
    
  }
  if(carstatus_now == crossroadR)  {
        //ips200_show_string(0,300,"crossR");


    if(Left_Up_Find&&Right_Up_Find)                                     //如果左上点和右上点都有效
    {
        carstatus_now = crossroad;                                      // 进入十字路口状态
        return;
    }
    if(Right_Up_Find!=0&&Right_Down_Find!=0)                            //如果找到了右上点和右下点
    {
        add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
        Right_Up_Find, rightline[Right_Up_Find]);
        centerline2_change();
    }
    if(Right_Up_Find!=0&&Right_Down_Find==0)                            //如果只找到了右上点
    {
        lenthen_Right_bondarise(Right_Up_Find);                         //延长右边界
        centerline2_change();

    }
    if(Right_Up_Find==0)
    {
        carstatus_now = straight;                                      // 如果右上点未找到，返回直道状态
    }
  }

   /*---------- 圆环预识别状态处理 ----------*/
   if(carstatus_now == round_1)
    {
        Right_Up_Find=Find_Right_Up_Point(MT9V03X_H-1, search_stop); // 查找右上拐点,使用更新的函数,
        Right_Down_Find=Find_Right_Down_Point(MT9V03X_H-2, search_stop); // 查找右下拐点,使用更新的函数
        //使用更新的更精确的函数找上下拐点

        //ips200_show_string(0,300,"round1");
        if(Right_Up_Find>10&&Right_Up_Find<35&&Right_Down_Find==0)//右下点没找到
        {
            lastrightupfind=Right_Up_Find; //记录上 次右上点
            BUZZ_START();
            carstatus_now = round_2;
            return; 
        }
        else    
        {   
            trace_right_bu(1,MT9V03X_H-1); //右单调下补右线
            centerline2_change();

        }
          if(Right_Down_Find>35&&right_budandiao)
        { 
            trace_right_bu(1,MT9V03X_H-1 ); //右单调下补右线
            //注:这里为了考虑到让他走直线现这么搞着
            centerline2_change();
        } 

   }
   if(carstatus_now == round_2)
   { 
        //ips200_show_string(0,300,"round2");
        Right_Up_Find   =Find_Right_Up_Point(MT9V03X_H-1, search_stop); // 查找右上拐点,使用更新的函数,
        Right_Down_Find =Find_Right_Down_Point(MT9V03X_H-2, search_stop); // 查找右下拐点,使用更新的函数
        if(Right_Up_Find)//找到上点
        {  
            search_stop1=Right_Up_Find;
            add_Lline_k(rightline[Right_Up_Find]+10,Right_Up_Find,Right_Up_Find+30,leftline[Right_Up_Find+30]+10); //右上点补直线
            centerline2_change();
        }
        if(rightline[Right_Up_Find]<MT9V03X_W/2)
        {
            left_start_point=MT9V03X_W/2;
            BUZZ_START();
            carstatus_now=round_3;
        }
   }
   if (carstatus_now == round_3) 
   {
        Left_Down_Find=Find_Left_Down_Point(MT9V03X_H-1, search_stop-3); // 查找左下拐点,使用更新的函数,
        Left_Up_Find=Find_Left_Up_Point(MT9V03X_H-2, search_stop); // 查找左上拐点,使用更新的函数,
        if (Left_Up_Find)
        {
            add_Lline_k(leftline[Left_Up_Find],Left_Up_Find,MT9V03X_H-1,leftline[MT9V03X_H-1]);
        } 
        centerline2_change();
 
        if(leftlostpoint[0]<=10)
        {
            left_start_point=0;
            carstatus_now=round_4;
            BUZZ_START();
        }   
        //ips200_show_string(0,300,"round3");
        
    }
    if(carstatus_now == round_4)
    {
        //ips200_show_string(0,300,"round4");
        left_budandiao=montonicity_left(MT9V03X_H-1,search_stop+6); // 左不单调点最少加6
        if(left_budandiao)
        {
             BUZZ_START();
            carstatus_now=round_5; // 进入左拐点补斜线状态
            return;

        }

    }
    if (carstatus_now==round_5) 
    {
        //ips200_show_string(0,300,"round5");
        left_budandiao=montonicity_left(MT9V03X_H-1,search_stop+6); // 左不单调点最少加6
        // Right_Up_Find=Find_Right_Up_Point(MT9V03X_H-1, search_stop); // 查找右上拐点,使用更新的函数
        // lenthen_Left_bondarise_bottom(left_budandiao); // 延长左边界到底部
        draw_Lline_k(leftline[left_budandiao],left_budandiao,search_stop,-1); // 左不单调点补直线
        draw_Rline_k(MT9V03X_W-1,search_stop,rightlostpoint[1],0); // 右不单调点补直线
        if(left_budandiao==0) // 如果左不单调点为0
        {
            carstatus_now=round_6; // 进入出环补直线状态
            BUZZ_START();
        }
        centerline2_change();
    }
    if(carstatus_now==round_6)
    {
        //ips200_show_string(0,300,"round6");

        Find_Right_Up_Point(MT9V03X_H-1, search_stop); // 查找右上拐点,使用更新的函数,
        draw_Lline_k(0,MT9V03X_H-1,search_stop,-2);// 左边界补直线

        centerline2_change(); 

        if(Right_Up_Find||search_stop<20)
        {
            BUZZ_START();
            carstatus_now=round_7; // 进入直道状态
        }

    }
    if(carstatus_now==round_7)
    {
        //ips200_show_string(0,300,"round7");
        centerline2_change();
        if(right_start_point>60)
        {
            carstatus_now=straight;
            BUZZ_START();
        }
    }

    
   
    // //ips200_show_int(200,260,search_stop1,3); // 显示搜索终止点1
    // //ips200_show_int(50,280,search_stop,3);      // 显示截止行
    // //ips200_show_string(80,220,"l_con");
    // //ips200_show_int(120,220,continuity_pointLeft[0],3); // 显示左不连续点
    // //ips200_show_string(150,220,"r_con");
    // //ips200_show_int(200,220,continuity_pointRight[0],3); // 显示右不连续点
    // //ips200_show_string(0,220,"r_bdd");         //右不单调点
    // //ips200_show_int(50,220,right_budandiao,3); // 显示右不单调点
    // //ips200_show_string(0,220,"r_bdd");         //右不单调点
    // //ips200_show_string(80,240,"l_bdd");         //截止行
    // //ips200_show_int(120,240,left_budandiao,3); // 显示v点
    // //ips200_show_string(0,280,"s_stop");         //截止行
    // //ips200_show_string(80,280,"l_up");          //左上拐点
    // //ips200_show_int(120,280,Left_Up_Find,3);    
    // //ips200_show_string(70,300,"r_up");          //右上拐点
    // //ips200_show_int(120,300,Right_Up_Find,3);
    // //ips200_show_string(160,260,"sto1");       //搜索终止点1
    // //ips200_show_string(150,280,"L_down");       //左下拐点
    // //ips200_show_int(200,280,Left_Down_Find,3);
    // //ips200_show_string(150,300,"R_down");       //右下拐点
    // //ips200_show_int(200,300,Right_Down_Find,3);
    // //ips200_show_string(0,260,"L_lost");       //左丢线点
    // //ips200_show_int(50,260,leftlostpoint[0],3); // 显示左丢线点
    // //ips200_show_string(80,260,"R_lost");       //右丢
    // //ips200_show_int(130,260,rightlostpoint[0],3); // 显示右丢线点

    
}



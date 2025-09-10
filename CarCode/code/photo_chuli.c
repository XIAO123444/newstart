 /*
 * camera.c
 *
 *  Created on: 2023年10月24日
 *      Author: lychee
 */
#include "photo_chuli.h"
#include "math.h"
#include "track.h"
int16 centerline[MT9V03X_H];
int16 leftline[MT9V03X_H];
int16 rightline[MT9V03X_H];
int16 rightfollowline[MT9V03X_H];
int16 leftfollowline[MT9V03X_H];


int16 leftlostpoint[2]={0,0};   //左丢线数和左丢线点0为丢线数，1为丢线索引
int16 rightlostpoint[2]={0,0};  //右丢线数和左丢线点0为丢线数，1为丢线索引
int16 bothlostpoint[2]={0,0};   //同时丢线数和左丢线点0为丢线数，1为丢线索引

int16 white_point_count[MT9V03X_W]={0}; //每列白点计数
int16 white_point_count1[MT9V03X_W]   ={0}; //每列白点计数滤波 1   


int16 left_longest[2]={0,0};  //左最长白列数和左最长白列点0长度，1为W索引
int16 right_longest[2]={0,0};  //右最长白列数和左最长白列点0长度，1为W索引
int16 left_start_point=0;  //左起点
int16 right_start_point=MT9V03X_W-1; //右起点

//y点处理
int8 leftorright=0;                        //从左还是右找y点 左为-1 右为1 0为无效
int16 white_y_point=-1;                     //找y点 ,没找到为-1 
int16 leftblackpoint_index=-1;    //找左y黑点，没找到为-1
int16 rightblackpoint_index=-1;   //找右y黑点，没找到为-1

int16 boundry_start_left=0; //左边界起始点
int16 boundry_start_right=0; //右边界起始点

int16 search_stop=0; //终止点
int16 search_stop1=0; //终止点1

uint16 left_lost_flag[MT9V03X_H];//左丢线数组   0-未丢线，1-丢线
uint16 right_lost_flag[MT9V03X_H];//右丢线数组  0-未丢线，1-丢线
uint16 both_lost_flag[MT9V03X_H];//同时丢线数组 0-未丢线，1-丢线

//十字↓↓↓↓
int16 Right_Down_Find=0;    //右下点
int16 Left_Down_Find=0;     //左下点
int16 Right_Up_Find=0;      //右上点
int16 Left_Up_Find=0;       //左上点

//十字↑↑↑↑
//车状态
enum mark {
    straight,    // 直道行驶
    crossroad,   // 十字路口
    round_2,   // 入环补直线
    round_3,   // 圆环补斜线（未使用）
    round_4,   // 入环行驶
    round_5,   // 左拐点补斜线
    round_6    // 出环补直线
};
extern enum  mark carstatus_now;
//锁列
extern int16 bailie_lock_crossroad;
extern int16 bailieright_lock_round;
extern bool start_flag; //发车标志位

//线点与丢线↓↓↓↓
uint8 leftline_num;         //左线点数量
uint8 rightline_num;        //右线点数量


//圆环↓↓↓↓
int16 right_down_guai   =0;            //右下拐点
int16 right_up_guai     =0;            //右上拐点
int16 left_down_guai    =0;            //左下拐点
int16 left_up_guai      =0;            //左上拐点

int16 left_budandiao       =0;     //左不单调
int16 right_budandiao      =0;     //右不单调
//圆环↑↑↑↑



//差比和↓↓↓↓
int16 sar_thre = 17;//差比和阈值
//差比和↑↑↑↑
uint8 pix_per_meter = 20;//每米的像素数

extern int16 threshold_up;  //大津法阈值上限
extern int16 threshold_down; //大津法阈值下限

extern bool stop_flag1;

float dx1[5]={0};
float dx2[5]={0};

int16 right_down_line =0;
/*
// //这里都是差比和↓↓↓↓↓↓↓
// 
// ------------------------------------------------------------------------------------------------------------------
// 函数简介     差比和寻找边界点
// 参数说明     无
// 返回参数     无
// 使用示例     直接调用
// 备注信息     无
// -------------------------------------------------------------------------------------------------------------------

// void image_boundary_process(void){
//     uint8 row;//行
//     //uint8 col = MT9V03X_W/2;//列
//     uint8 start_col = MT9V03X_W / 2;//各行起点的列坐标,默认为MT9V03X_W / 2
//     //清零之前的计数
//     leftline_num = 0;
//     rightline_num = 0;

//     for(row = MT9V03X_H - 1; row >= 1; row--){
//         //选用上一行的中点作为下一行计算起始点，节省速度，同时防止弯道的左右两边均出现与画面一侧
//         if(row != MT9V03X_H - 1){
//             start_col = (uint8)(0.4 * centerline[row] + 0.3 * start_col + 0.1 * MT9V03X_W);//一阶低通滤波，防止出现噪点影响下一行的起始点


//         }
//         else if(row == MT9V03X_H - 1){
//             start_col = MT9V03X_W / 2;
//         }
//         if(start_col<MT9V03X_W/2-30){start_col=MT9V03X_W/2-30;}
//         if(start_col>MT9V03X_W/2+30){start_col=MT9V03X_W/2+30;}
//         //逐行作差比和 
//         difsum_left(row,start_col);
//         difsum_right(row,start_col); 
//         centerline[row] = 0.5 * (rightline[row] + leftline[row]);
//     }
// }
//
// ------------------------------------------------------------------------------------------------------------------
// 函数简介     差比和寻找左侧边界点
// 参数说明     
// 返回参数     
// 使用示例     
// 备注信息     
// -------------------------------------------------------------------------------------------------------------------

// void difsum_left(uint8 y,uint8 x){
//     float sum,dif,sar;//和，差，比
//     uint8 col;//列
//     uint8 mov = 3;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
//     //计算第x行的左边界
//     leftline[y] = 0;//未找到左边界时输出为0
//     for(col = x; col >= mov + 1; col -= mov){
//         dif = (float)((mt9v03x_image[y][col] - mt9v03x_image[y][col - mov - 1])<<8);//左移8位即乘256，可避免浮点数乘，加快速度
//         sum = (float)((mt9v03x_image[y][col] + mt9v03x_image[y][col - mov - 1]));
//         sar = fabs(dif / sum);//求取差比和
//         if(sar > sar_thre){//差比和大于阈值代表深浅色突变
//             leftline[y] = (int16)(col - mov);
//             leftline_num ++;//左线点计数+
//             break;//找到边界后退出
//         }
//     }
// }

// ------------------------------------------------------------------------------------------------------------------
// 函数简介     差比和寻找右侧边界点
// 参数说明     
// 返回参数     
// 使用示例     
// 备注信息     
// -------------------------------------------------------------------------------------------------------------------
// 
// void difsum_right(uint8 y,uint8 x){
//     float sum,dif,sar;//和，差，比
//     uint8 col;//列
//     uint8 mov = 3;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
//     //计算第x行的左边界
//     rightline[y] = MT9V03X_W - 1;//未找到右边界时输出为187
//     for(col = x; col <= MT9V03X_W - mov - 1; col += mov){
//         dif = (float)((mt9v03x_image[y][col] - mt9v03x_image[y][col + mov + 1])<<8);//左移8位即乘256，可避免浮点数乘，加快速度
//         sum = (float)((mt9v03x_image[y][col] + mt9v03x_image[y][col + mov + 1]));
//         sar = fabs(dif / sum);//求取差比和
//         if(sar > sar_thre){//差比和大于阈值代表深浅色突变
//             rightline[y] = (int16)(col + mov);
//             rightline_num ++;//右线点计数+
//             break;//找到边界后退出
//         }
//     }
// }


// //这里都是差比和↑↑↑↑↑↑↑
*/
//大津法↓↓↓↓↓↓↓↓↓↓
uint8 dis_image[MT9V03X_H][MT9V03X_W];
int16 image_threshold = 46;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     快速大津求阈值，来自山威
  @param     image       图像数组
             col         列 ，宽度
             row         行，长度
  @return    null
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//山威快速大津
  @note      据说比传统大津法快一点，实测使用效果差不多
-------------------------------------------------------------------------------------------------------------------*/
int my_adapt_threshold(uint8 *image, uint16 col, uint16 row)   //注意计算阈值的一定要是原图像
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //统计灰度级中每个像素在整幅图像中的个数
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //将当前的点的像素值作为计数数组的下标
            gray_sum+=(int)data[i * width + j];       //灰度值总和
        }
    }
    //计算每个像素值的点在整幅图像中的比例
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {
        w0 += pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和   即背景部分的比例
        u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //背景平均灰度
        u1 = u1tmp / w1;              //前景平均灰度
        u = u0tmp + u1tmp;            //全局平均灰度
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = j;
        }
        if (deltaTmp < deltaMax)
        {
            break;
        }
    }
	if(threshold<threshold_down)
	{
		return threshold_down ;
	}
	if(threshold>threshold_up)
	{
		return threshold_up;
	}
    return threshold;
}
void set_b_imagine(int threshold)
{
		for(int16 i=0;i<MT9V03X_H;i++)
	{
		for(int16 j=0;j<MT9V03X_W;j++)
		{
			dis_image[i][j]=(mt9v03x_image[i][j]>threshold)?255:0;
		}
	}
}

void difsum_left1(uint8 y,uint8 x)
	{
    uint8 col;//列
    uint8 mov = 1; //每次作差后的移动量,默认为2，可以根据画面分辨率调整
    //计算第x行的左边界
    leftline[y] = 0;//未找到左边界时输出为0
    for(col = x; col >= mov + 1; col -= mov)
	{
		if(dis_image[y][col] - dis_image[y][col - mov]>0)
		{
			leftline[y] = col;
			leftline_num ++;//左线点计数+
            return;
		}

	}
    leftlostpoint[0]++;
    left_lost_flag[y]=1;
    //何为丢线？在
    if(leftlostpoint[1]==0)
    {
        leftlostpoint[1]=y;
    }
}
void difsum_right1(uint8 y,uint8 x)
{
	uint8 col;//列
    uint8 mov = 1;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
    //计算第x行的左边界
    rightline[y] = MT9V03X_W-1;//未找到左边界时输出为0
    for(col = x; col <= MT9V03X_W - mov - 1; col += mov)
	{
		if(dis_image[y][col] - dis_image[y][col + mov]>5)
		{
			rightline[y] = col ;
			rightline_num ++;//右边线点计数+
			return;//找到边界后退出
		}

	}
    rightlostpoint[0]++;
    right_lost_flag[y]=1;

    if(rightlostpoint[1]==0)
    {
        rightlostpoint[1]=y;
    }
}
/*
-------------------------------------------------------------------------------------------------------------------
函数简介     参数初始化
参数说明     无
返回参数     无
使用示例     param_init();
备注信息     无
-------------------------------------------------------------------------------------------------------------------
*/
void param_init(void)
{
    leftline_num = 0;
    rightline_num = 0;


    leftorright=0;                        //从左还是右找y点
    white_y_point=0;                     //找y点
    leftblackpoint_index=-1;            //找左y黑点
    rightblackpoint_index=-1;           //找右y黑点
    left_longest[0]=1;      //左最长白列清零
    right_longest[0]=1;     //右最长白列清零
    leftlostpoint[0]=0;      //左丢线数清零
    rightlostpoint[0]=0;     //右丢线数清零
    bothlostpoint[0]=0;      //同时丢线数清零

    left_longest[1]=0;      //左最长白列索引清零
    right_longest[1]=0;     //右最长白列索引清零
    leftlostpoint[1]=0;      //左丢线点清零
    rightlostpoint[1]=0;     //右丢线点清零
    bothlostpoint[1]=0;      //同时丢线点清零

    //白线计数清零和左右线清零
    for(int16 i=0;i<MT9V03X_H;i++)
    {
        leftline[i]=0;              //左线清0
        rightline[i]=MT9V03X_W-1;   //右线清0
        right_lost_flag[i]=0;     //右丢线清0   
        left_lost_flag[i]=0;      //左丢线清0   
        both_lost_flag[i]=0;      //同时丢线清0
        centerline[i]=0;          //中线清0
    }
    for(int16 i=0;i<MT9V03X_W;i++)
    {
        white_point_count[i]=0;     //白点计数置0
        white_point_count1[i]=0;    //白点计数滤波1置0
    }
}

int16 Threshold= 4 ;
int16 Thresholdnum=10;

/*
-------------------------------------------------------------------------------------------------------------------
函数简介    图象y点处理
参数说明     内置左起点，右起点，可以在圆环时候用
返回参数     white_y_point;leftorright;(1在右，-1在左)
使用示例     find_y_point();
备注信息     无
-------------------------------------------------------------------------------------------------------------------
*/  
void find_y_point(void)
{
    for(int i=left_start_point+15;i<right_start_point-15;i++)
    {
        bool bigbreak=false;
        leftblackpoint_index=-1;
        rightblackpoint_index=-1;
        for(int j=1;i-j>=0&&j<16;j++)//向左找
        {
            if(white_point_count[i]>white_point_count[i-j])
            {
                bigbreak=true;
                break;
            } 
            if(white_point_count[i]<white_point_count[i-j]&&white_point_count[i]<white_point_count[i-j-3])
            {
                leftblackpoint_index=i-j+1;
                break;
            }

        }
        if(bigbreak)
        {
            continue;
        }
        for(int j=1;i+j<=MT9V03X_W-1&&j<16;j++)
        {
            if(white_point_count[i]>white_point_count[i+j])
            {
                bigbreak=true; 
                break;
            }
            if(white_point_count[i]<white_point_count[i+j]&&white_point_count[i]<white_point_count[i+j+3])
            {
                rightblackpoint_index=i+j-1;
                break;
            }

        }
        if(bigbreak)
        {
            continue;
        }
        if(leftblackpoint_index!=-1&&rightblackpoint_index!=-1&&leftblackpoint_index>left_start_point+15&&rightblackpoint_index<right_start_point-15)
        {
            
            break;
        }
    }

    int16 right_interg=white_point_count[rightblackpoint_index+1]+white_point_count[rightblackpoint_index+2]+
        white_point_count[rightblackpoint_index+3]+white_point_count[rightblackpoint_index+4]+white_point_count[rightblackpoint_index+8]
    -white_point_count[rightblackpoint_index]*5;
    int16 left_interg =white_point_count[leftblackpoint_index-1]+white_point_count[leftblackpoint_index-2]
        +white_point_count[leftblackpoint_index-3]+white_point_count[leftblackpoint_index-4]+white_point_count[leftblackpoint_index-8]   
    -white_point_count[leftblackpoint_index]*5;
    if(left_interg>right_interg)
    {
        white_y_point=leftblackpoint_index;
        leftorright=-1;
    }
    else if(left_interg<right_interg)
    {
        white_y_point=rightblackpoint_index;
        leftorright=1;
    }
    else
    {
        leftorright=0;
    }
    
    
}

void image_boundary_process2(void)
	{
    uint8 row;//行
    param_init();
    //最长白列计数
    for(int16 i=left_start_point;i<right_start_point;i+=1)
    {
        for(int16 j=MT9V03X_H-1;j>0;j--)
        {
            if(dis_image[j][i]==255) 
            {
                white_point_count[i]++;     //白点计数
            }
            else
            {
                break;                  //遇到黑色白线结束
            }
        }
    }
    memcpy(white_point_count1, white_point_count, sizeof(white_point_count)); //将白点计数复制到白点计数滤波1
    find_y_point(); 
    if(leftorright==0)
    {
        ips200_show_string(0,240,"middle");
        ips200_show_int(60,240,0,3);
        for(int16 i=left_start_point;i<right_start_point;i+=1)       //寻找最长左白列
        {

            if(white_point_count1[i]>left_longest[0])
            {
                left_longest[0]=white_point_count1[i];           
                left_longest[1]=i;
            }
        }
        for(int16 i=right_start_point;i>left_start_point;i-=1)       //寻找最  长右白列
        {

            if(white_point_count1[i]>right_longest[0]) 
            {
                right_longest[0]=white_point_count1[i];         
                right_longest[1]=i;
            }
        }
    }
    else if(leftorright==1) //如果是右侧找y点
    {
        ips200_show_string(0,240,"right ");
        ips200_show_int(60,240,white_y_point,3);
        for(int16 i=white_y_point;i<MT9V03X_W-1;i+=1)       //寻找最长左白列
        {

            if(white_point_count1[i]>left_longest[0])
            {
                left_longest[0]=white_point_count1[i];           
                left_longest[1]=i;
            }
        }
        for(int16 i=MT9V03X_W-1;i>white_y_point;i-=1)       //寻找最  长右白列
        {

            if(white_point_count1[i]>right_longest[0]) 
            {
                right_longest[0]=white_point_count1[i];         
                right_longest[1]=i;
            }
        }
    }
    else if(leftorright==-1) //如果是左侧找y点
    {
                ips200_show_string(0,240,"left ");
        ips200_show_int(60,240,white_y_point,3);
        for(int16 i=white_y_point;i>=0;i-=1)       //寻找最长左白列
        {

            if(white_point_count1[i]>left_longest[0])
            {
                left_longest[0]=white_point_count1[i];           
                left_longest[1]=i;
            }
        }
        for(int16 i=0;i<white_y_point;i+=1)       //寻找最  长右白列
        {

            if(white_point_count1[i]>right_longest[0]) 
            {
                right_longest[0]=white_point_count1[i];         
                right_longest[1]=i;
            }
        }
    }
    

    search_stop=(right_longest[0]< left_longest[0])?(MT9V03X_H-right_longest[0]-1):(MT9V03X_H-1-left_longest[0]); //由于是从屏幕下往上，所以是选大的
    search_stop1=search_stop; //搜索终止点1
    if(search_stop==-1)
    {
        search_stop=0;//防止越界
    }
    if(search_stop>=MT9V03X_H-1||search_stop<0) //如果最长白列小于10行，说明没有白线
    {
        return; //没有白线，直接返回 
    }
    else
    {
        for(row = MT9V03X_H - 1; row > search_stop; row--)
        {
            difsum_left1(row,left_longest[1]); //使用最长白列的起点作为起点寻找左线
            difsum_right1(row,right_longest[1]); //使用最长白列的起点作为起点寻找右线
        }
        
        
    }
      //测赛宽
    // printf("\n");
    // printf("rightline");

    // for(int16 i=0;i<MT9V03X_H;i++)
    // {
    //     printf("%d,",rightline[i]);
    // }
    // printf("\n");
    // printf("leftline");
    // for(int16 i=0;i<MT9V03X_H;i++)
    // {
    //     printf("%d,",leftline[i]);
    // }      
    for(row = MT9V03X_H - 1; row > search_stop; row--)
    {
        if(right_lost_flag[row]==1&&left_lost_flag[row]==1) //同时丢线
        {
            bothlostpoint[0]++;
            bothlostpoint[1]=row;
            both_lost_flag[row]=1;
        }
        if(boundry_start_left==0&&leftline[row]!=0) {boundry_start_left=row;}   //左边界起始点
        if(boundry_start_right==0&&rightline[row]!=MT9V03X_W-1) {boundry_start_right=row;} //右边界起始点
    }
}


//大津法↑↑↑↑↑↑↑↑↑↑
//斑马线处理和保护在这里
void black_protect_check(void)
{
    int16 sum =0;
    for(int16 i=70;i>20;i--)
    {
        if(mt9v03x_image[ MT9V03X_H - 1][i]<threshold_down)
        {
            sum++;
        }

    }
        for(int16 i=70;i<120;i++)
    {
        if(mt9v03x_image[ MT9V03X_H - 1][i]<threshold_down)
        {
            sum++; 
        }

    }
            if (sum>100*0.8)
        {
            start_flag=false;
            stop_flag1=true;
        }
}
void banmaxian_check(void)
{
	int16 count=0;
    int16 sum =0;
    bool black=0;
    for(int i=MT9V03X_H-1;i>=MT9V03X_H-3;i--)
            {
                for(int j=0;j<=MT9V03X_W-1-3;j++)
                {
                    if(dis_image[i][j]==255&&dis_image[i][j+1]==0&&dis_image[i][j+2]==0)
                    {
                        count++;
                    }
                }
                if(count>=10)//如果黑色计数大于等于40，认为是斑马线
                {
                    
                    stop_flag1=true;
                    start_flag=false;
                }
            }
}


/*
------------------------------------------------------------------------------------------------------------------
函数简介      输出中点位置：这里是从底向上数第四个
参数说明     无
返回参数     int16，中点的横坐标
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
int16 output_middle(void)
{
    return centerline[MT9V03X_H-15];
}
//十字判断
//十字判断
//十字判断
//十字判断
//十字判断
//十字判断

/*-------------------------------------------------------------------------------------------------------------------
  @brief     找下面的两个拐点，供十字使用
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int16 start,int16 end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Down_Point(int16 start,int16 end)
{
    int16 i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;
    if(start<end)               //从下往上找，大的索引在图象下面
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-3)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=MT9V03X_H-1-3;
    if(end>=MT9V03X_H-5)
        end=MT9V03X_H-5;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find==0&&//只找第一个符合条件的点
            (leftline[i]>0)&&//左边界点不能为0
           abs(leftline[i]-leftline[i+1])<=3&&//角点的阈值可以更改
           abs(leftline[i+1]-leftline[i+2])<=3&&
           abs(leftline[i+2]-leftline[i+3])<=3&&
            ((leftline[i]-leftline[i-2])>=3)&&
            ((leftline[i]-leftline[i-3])>=5)&&
            ((leftline[i]-leftline[i-4])>=5))
        {
            Left_Down_Find=i+2;//获取行数即可
            if(Left_Down_Find==start+2)
            {
                Left_Down_Find=0;//如果是起始行，说明没有找到
            }
        }
        if(Right_Down_Find==0&&//只找第一个符合条件的点
           abs(rightline[i]-rightline[i+1])<=3&&//角点的阈值可以更改
           abs(rightline[i+1]-rightline[i+2])<=3&&
           abs(rightline[i+2]-rightline[i+3])<=3&&
            rightline[i]<MT9V03X_W-1&&//右边界点不能为MT9V03X_W-1
              ((rightline[i]-rightline[i-2])<=-3)&&
              ((rightline[i]-rightline[i-3])<=-5)&&
              ((rightline[i]-rightline[i-4])<=-5))
        {
            Right_Down_Find=i+2;
            if(Right_Down_Find==start+2)
            {
                Right_Down_Find=0;//如果是起始行，说明没有找到
            }
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0)//两个找到就退出
        {
            break;
        }
    }

}
 
/*-------------------------------------------------------------------------------------------------------------------
  @brief     找上面的两个拐点，供十字使用,从上往下找
  @param     搜索的范围起点，终点
  @return    修改两个全局变量
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int16 start,int16 end)
  @note      运行完之后查看对应的变量，注意，没找到时对应变量将是0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Up_Point(int16 start,int16 end)
{
    int16 i,t;
    Left_Up_Find=0;
    Right_Up_Find=0;
    if(start>end)
    {
        t=start;
        start=end;
        end=t;
    }
    //start<end由上往下
    if(end>=MT9V03X_H-5)
        end=MT9V03X_H-5;
    if(end<=5)//及时最长白列非常长，也要舍弃部分点，防止数组越界
        end=5;
    if(start<=5)//下面5行数据不稳定，不能作为边界点来判断，舍弃
        start=5;
    if(start<search_stop+5)
    {
        start =search_stop+5;
    }

    for(i=start;i<=end;i++)
    { 
        if(Left_Up_Find==0&&//只找第一个符合条件的点
           abs(leftline[i]-leftline[i-1])<=3&&
           abs(leftline[i-1]-leftline[i-2])<=3&&
           abs(leftline[i-2]-leftline[i-3])<=3&&
           leftline[i-2]-leftline[i]>-2&&
            leftline[i-3]-leftline[i]>-3&&
              ((leftline[i]-leftline[i+2])>=3)&&
              ((leftline[i]-leftline[i+3])>=5)&&
              ((leftline[i]-leftline[i+4])>=10))
        {

            Left_Up_Find=i-2;//获取行数即可
            if(Left_Up_Find==start-2)
            {
                Left_Up_Find=0;//如果是起始行，说明没有找到
            }
        }
        if(Right_Up_Find==0&&//只找第一个符合条件的点
           abs(rightline[i]-rightline[i-1])<=3&&//下面两行位置差不多
           abs(rightline[i-1]-rightline[i-2])<=3&&
           abs(rightline[i-2]-rightline[i-3])<=3&&
           rightline[i-2]-rightline[i]<2&&
           rightline[i-3]-rightline[i]<3&&
              ((rightline[i]-rightline[i+2]<=-3))&&
              ((rightline[i]-rightline[i+3])<=-5)&&
              ((rightline[i]-rightline[i+4])<=-10))
        {
            Right_Up_Find=i-2;//获取行数即可 
            if(Right_Up_Find==start-2)
            {
                Right_Up_Find=0;//如果是起始行，说明没有找到
            }
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//下面两个找到就出去
        {
            break;
        }
    }
 
     
}

/*
------------------------------------------------------------------------------------------------------------------
函数简介     滑动平均滤波左
参数说明     无
返回参数     无
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
void dx1_left_average(float dx)
{
    for(uint8 i=1;i<5;i++)
    {
        dx1[i-1]=dx1[i];
    }
    dx1[4]=dx;
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     滑动平均滤波右
参数说明     无
返回参数     无
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
void dx2_right_average(float dx)
{
    for(uint8 i=1;i<5;i++)
    {
        dx2[i-1]=dx2[i];
    }
    dx2[4]=dx;
}

//圆环判断
//1.找圆环

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左下角点检测
  @param     起始行，终止行
  @return    返回角点所在的行数，找不到返回0
  Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int16 Find_Left_Down_Point(int16 start,int16 end)//找左下角点，返回值是角点所在的行数
 {
    int16 i,t;
    int16 left_down_line=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>MT9V03X_H-2)
    {
        start=MT9V03X_H-2;//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
    }
    if(end<4)
    {
        end=4;//及时最长白列非常长，也要舍弃部分点，防止数组越界    
    }
    if(leftline[start]<=0&&leftline[start-1]<=0&&leftline[start-2]<=0&&leftline[start-3]<=0)//如果起始点就都丢线，没有拐点判断的意义
    {
        return 0;
    }
    for(int i=start;i>=end;i--)//由下至上找角点 
    {
        if(((leftline[i]-leftline[i-2])>=8||leftline[i-2]==0)&&
           ((leftline[i]-leftline[i-3])>=15||leftline[i-3]==0)&&
           ((leftline[i]-leftline[i-4])>=15||leftline[i-4]==0))
        {
            left_down_line=i+1;//获取行数即可
            break;
        }
    }
    return left_down_line;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测（圆环）
  @param     起始行，终止行
  @return    返回角点所在的行数，找不到返回0
  Sample     right_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      这个新判断对图象的干净程度要求较高，角点检测阈值可根据实际值更改
-------------------------------------------------------------------------------------------------------------------*/
int16 Find_Right_Down_Point(int16 start,int16 end)
{
    int16 i,t;
    int16 right_down_line=0;
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>MT9V03X_H-3)
    {
        start=MT9V03X_H-3;//下面5行上面5行数据不稳定，不能作为边界点来判断，舍弃
    }
    if(end<5)
    {
        end=5;//及时最长白列非常长，也要舍弃部分点，防止数组越界    
    }
    if(rightline[start]>=MT9V03X_W-1&&rightline[start-1]>=MT9V03X_W-1&&rightline[start-2]>=MT9V03X_W-1&&rightline[start-3]>=MT9V03X_W-1)//如果起始点就都丢线，没有拐点判断的意义
    {
        return 0;
    }
    for(int i=start;i>=end;i--)
    {
        if(rightline[i]<=rightline[i+1]&&
            rightline[i+1]<=rightline[i+2]&&
            ((rightline[i]-rightline[i-2])<=-8)&&
           ((rightline[i]-rightline[i-3])<=-15||rightline[i-3]==MT9V03X_W-1)&&
           ((rightline[i]-rightline[i-4])<=-15||rightline[i-4]==MT9V03X_W-1))
        {
            right_down_line=i+1;//获取行数即可
            break;
        }
    }
    return right_down_line;
}

int16 Find_Right_Up_Point(int16 start,int16 end)//找四个角点，返回值是角点所在的行数
{
    int16 i,t;
    int16 right_up_line=0;
    if(start>end)
    {
        t=start;
        start=end;
        end=t;
    }    
    if(start<=3)//及时最长白列非常长，也要舍弃部分点，防止数组越界
    {start=3;}
    if(end>=MT9V03X_H-1-4)
    {end=MT9V03X_H-1-4;}
    if(rightline[start]==MT9V03X_W-1&&rightline[start+1]==MT9V03X_W-1&&rightline[start+2]==MT9V03X_W-1)
    {
        return 0;//如果起始点就都丢线，没有拐点判断的意义
    }
    for(i=start;i<=end;i++)
    {
        if(rightline[i]==MT9V03X_W-1)//如果右边界点是MT9V03X_W-1，说明没有找到
        {
            break;  //直接作废
        }
        if(rightline[i]>=rightline[i-1]&&
            rightline[i-1]>=rightline[i-2]&&
           ((rightline[i]-rightline[i+2])<=-8)&&
           ((rightline[i]-rightline[i+3])<=-15||rightline[i+3]==MT9V03X_W-1)&&
           ((rightline[i]-rightline[i+4])<=-15||rightline[i+4]==MT9V03X_W-1))
        {
            right_up_line=i;//获取行数即可
            break;
        }
    }
    return right_up_line;
}

int16 Find_Left_Up_Point(int16 start,int16 end)//找四个角点，返回值是角点所在的行数
{
    int16 i,t;
    int16 left_up_line=0;
    if(start>end)
    {
        t=start; 
        start=end;
        end=t;
    }    
    if(start<=1)//及时最长白列非常长，也要舍弃部分 点，防止数组越界
    {start=1;}
    if(end>=MT9V03X_H-1-4)
    {end=MT9V03X_H-1-4;}
    if(leftline[start]==0&&leftline[start+1]==0&&leftline[start+2]==0)
    {
        return 0;//如果起始点就都丢线，没有拐点判断的意义
    }
    for(i=start;i<=end;i++)//由上至下找角点
    {
        if(((leftline[i]-leftline[i+2])>=8)&&
           ((leftline[i]-leftline[i+3])>=15||leftline[i+3]==0)&&
           ((leftline[i]-leftline[i+4])>=15||leftline[i+4]==0))
        {
            left_up_line=i;//获取行数即可
            break;
        }
    }
    return left_up_line;
}

int16 find_vpoint(int16 start,int16 end)
{
    uint16 i;
    uint16 result=0;
    if (start>end)
    {
        int16 t=start;
        start=end;
        end=t;
    }
    if(start<5)
    {
        start=5; //防止数组越界
    }
    if(end>MT9V03X_H-6)
    {
        end=MT9V03X_H-6; //防止数组越界
    }
    for(int16 i=start;i<=end;i++)
    {
        if(white_point_count[i]<white_point_count[i+2]&&white_point_count[i]<white_point_count[i-2]&&
           white_point_count[i]<white_point_count[i+3]&&white_point_count[i]<white_point_count[i-3]&&
           white_point_count[i]<white_point_count[i+4]&&white_point_count[i]<white_point_count[i-4]&&
           white_point_count[i]-white_point_count[i+1]>-6&&
           white_point_count[i]-white_point_count[i-1]>-6&&
           white_point_count[i]-white_point_count[i+2]>-12&&
           white_point_count[i]-white_point_count[i-2]>-12&&
           white_point_count[i]-white_point_count[i+3]>-18&&
           white_point_count[i]-white_point_count[i-3]>-18&&
           white_point_count[i]-white_point_count[i+4]>-24&&
            white_point_count[i]-white_point_count[i-4]>-24    
        )
        {
            result=i;
             return result; //找到第一个满足条件的点就返回
        }
        
    }
    return result; //如果没有找到，返回0
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     确定连续性，用于处理右侧线。
参数说明     uint8起点和终点
返回参数     中点位置
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
int16 continuity_right(uint8 start,uint8 end)
{
    int16 i;
    int16 continuity_change_flag=0;
    if(start<end)
    {
        uint8 t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-2)//数组越界保护
        start=MT9V03X_H-2;
    if(end<=1)
    {
        end=1;
    }
    for(i=start;i>=end;i--)
    {
        if(abs(rightline[i]-rightline[i-1])>=5)//连续性阈值是5，可更改
       {
            continuity_change_flag=i;                                         //在i处不连续了

            break;
       }

    }

    return continuity_change_flag;
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     确定从start到end连续性，用于处理左侧线。
参数说明     uint8起点和终点
返回参数     中点位置
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/

int16 continuity_left(uint8 start,uint8 end)
{
    int16 i;
    int16 continuity_change_flag=0;
    if(start<end)
    {
        uint8 t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-2)//数组越界保护
        start=MT9V03X_H-2;
    if(end<=1)
    {
        end=1;
    }

    for(i=start;i>=end;i--)
    {
        if(abs(leftline[i]-leftline[i-1])>=5)//连续性阈值是5，可更改
       {

            continuity_change_flag=i;                                         //在i处不连续了

            break;
       }

    }
    return continuity_change_flag;
}
//单调性变化s
int16 montonicity_right (uint8 start,uint8 end)
{            

    int16 i;
    int16 result=0;

    if(start<end)
    {
        uint8 t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-6)//数组越界保护
        start=MT9V03X_H-6;
    if(end<=6)//这个不能改
    {
        end=6;
    }
    for(i=end;i<=start;i++)
    {

        if(rightline[i] <rightline[i+5]&&rightline[i] <rightline[i-5]&&rightline[i+1]!=MT9V03X_W-1&&rightline[i-1]!=MT9V03X_W-1)
        {
            result=i;
            return result;
        }
    }
    return 0;

}
int16 montonicity_left(uint8 start, uint8 end) 
{
    int16 i;
    int16 result = 0;

    // 确保start >= end（反向遍历）
    if (start < end) {
        uint8 t = start;
        start = end;
        end = t;
    }

    // 数组越界保护（与右单调性对称）
    if (start >= MT9V03X_H - 6) {
        start = MT9V03X_H - 6;
    }
    if (end <= 6) {  // 左单调性需保留5的偏移量
        end = 6;
    }

    // 反向遍历（从高到低）
    for (i = end; i <= start; i++) {

        if (leftline[i] > leftline[i + 5] && leftline[i] > leftline[i - 5]) {
            result = i;
            return result;
        }
    }
    return 0;
}




/*
------------------------------------------------------------------------------------------------------------------
以下都是补线
以下都是补线
以下都是补线
以下都是补线
以下都是补线
-------------------------------------------------------------------------------------------------------------------
*/


/*
------------------------------------------------------------------------------------------------------------------
函数简介     补线左
参数说明     起点x，y终点y，增长率dx
返回参数     无
使用示例     
备注信息     增长率dx等于x1-x2/y1-y2
-------------------------------------------------------------------------------------------------------------------
*/
void draw_Lline_k(int16 startx, int16 starty, int16 endy, float dx) {
    int16 step = (starty < endy) ? 1 : -1;
    if (dx == 0) {

        for (int16 i =starty; i != endy; i += step) {
            leftfollowline[i] = startx;
        }
        return;
    }
    for (int16 i = starty; i != endy; i += step) {

        int16 temp=startx + (int16)((float)(i - starty) * dx );   
        if(temp<0) temp=0; //防止越界
        if(temp>MT9V03X_W-1) temp=MT9V03X_W-1; //防止越界
        leftfollowline[i] = temp;

    }
}

/*
------------------------------------------------------------------------------------------------------------------
函数简介     补线右
参数说明     起点x，y终点y，增长率dx
返回参数     无  
使用示例     
备注信息     增长率dx等于x1-x2/y1-y2
-------------------------------------------------------------------------------------------------------------------
*/
void draw_Rline_k(int16 startx, int16 starty, int16 endy, float dx) {


    int16 step = (starty < endy) ? 1 : -1;

    if (dx == 0) 
    {                
        for (int16 i = starty; i != endy; i += step) 
        {
            rightfollowline[i] = startx;
        }
        return;
    }
    for (int16 i = starty; i != endy; i += step) {
        int16 temp=startx + (int16)((float)(i - starty) * dx );
        if(temp<0) temp=0; //防止越界
        if(temp>MT9V03X_W-1) temp=MT9V03X_W-1; //防止越界    
        rightfollowline[i] = temp;
    }
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     右两点间连线
参数说明     起点xy，终点xy
返回参数     无
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
void add_Rline_k(int16 startx, int16 starty, int16 endy,int16 endx)
{
    if(endy!=starty)
    {
        float dx=(float)(endx-startx)/(float)(endy-starty);
        draw_Rline_k(startx,starty,endy,dx);
    }
    else
    {
        return;
    }
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     左两点间连线
参数说明     起点xy，终点xy
返回参数     无
使用示例     
备注信息     能除以0那你真牛逼
-------------------------------------------------------------------------------------------------------------------
*/
void add_Lline_k(int16 startx, int16 starty, int16 endy,int16 endx)
{
    if(endy!=starty)
    {
        float dx=(float)(endx-startx)/(float)(endy-starty);
        draw_Lline_k(startx,starty,endy,dx);
    }
    else{
         return;
    }
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     自上而下补左线
参数说明     起点
返回参数     无
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
void lenthen_Left_bondarise(int16 start)
{
    if(start<7){start=7;}
    if(start>MT9V03X_H-1){start=MT9V03X_H-1;}
    float dx=(float)(leftline[start]-leftline[start-5])/5;
    dx1_left_average(dx);
    float dx_average=(dx1[0]+dx1[1]+dx1[2]+dx1[3]+dx1[4])/5;
    for(int16 i=start;i<MT9V03X_H-1;i++)
    {
        if((float)leftline[start]+(float)(dx_average*(i-start))<0)
        {
            leftfollowline[i]=0;
        }
        else if((float)leftline[start]+dx_average*(float)(i-start)>(float)MT9V03X_W-2)
        {
            leftfollowline[i]=MT9V03X_W-1;
        }
        else
        {
            leftfollowline[i]=(int16)((float)leftline[start]+dx_average*(float)(i-start));
        }
    }
}
 /*
------------------------------------------------------------------------------------------------------------------
函数简介     自上而下补右线
参数说明     起点
返回参数     无
使用示例      
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/  
void lenthen_Right_bondarise(int16 start)  
{
    if(start<7){start=7;}
    if(start>MT9V03X_H-1){start=MT9V03X_H-1;}
    float dx=(float)(rightline[start]-rightline[start-5])/5;
    dx2_right_average(dx);
    float dx_average=(dx2[0]+dx2[1 ]+dx2[2]+dx2[3]+dx2[4])/5;
    for(int16 i=start;i<MT9V03X_H-1;i++)
    {
        if((float)rightline[start]+dx_average*(i-start)>MT9V03X_W-2)
        {
            rightfollowline[i]=MT9V03X_W-1;
        }
        else if((float)rightline[start]+dx_average*(float)(i-start)<0)
        {
            rightfollowline[i]=0;
        }
        else 
        {
            rightfollowline[i]=(int16)((float)rightline[start]+dx_average*(float)(i-start));
        }
    }
}
/*
------------------------------------------------------------------------------------------------------------------
函数简介     自下而上补左线
参数说明     起点
返回参数     无
使用示例     
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/
void lenthen_Left_bondarise_bottom(int16 start)
{
    if(start<0){start=0;}
    if(start>MT9V03X_H-8){start=MT9V03X_H-8;}
    float dx=-(float)(leftline[start]-leftline[start+5])/5;
    for(int16 i=start;i>=0;i--)
    {
        if((float)leftline[start]+(float)(dx*(i-start))<0)
        {
            leftfollowline[i]=0;
        }
        else if((float)leftline[start]+dx*(float)(i-start)>(float)MT9V03X_W-2)
        {
            leftfollowline[i]=MT9V03X_W-1;
        }
        else
        {
            leftfollowline[i]=(int16)((float)leftline[start]+dx*(float)(i-start));
        }
    }
}

/*
------------------------------------------------------------------------------------------------------------------
函数简介     自下而上补右线
参数说明     起点
返回参数     无
使用示例      
备注信息     
-------------------------------------------------------------------------------------------------------------------
*/  
void lenthen_Right_bondarise_bottom(int16 start)  
{
    if(start<0){start=0;}
    if(start>MT9V03X_H-8){start=MT9V03X_H-8;}
    float dx=(float)(rightline[start]-rightline[start+5])/5;

    for(int16 i=start;i>=0;i--)
    {
        if((float)rightline[start]+dx*(i-start)>MT9V03X_W-2)
        {
            rightfollowline[i]=MT9V03X_W-1;
        }
        else if((float)rightline[start]+dx*(float)(i-start)<0)
        {
            rightfollowline[i]=0;
        }
        else 
        {
            rightfollowline[i]=(int16)((float)rightline[start]+dx*(float)(i-start));
        }
    }
}

uint8 traceL[MT9V03X_H]=        //左边界轨迹
{0,74,73,74,73,72,72,71,70,70,69,69,68,67,67,66,65,65,64,64,63,62,62,62,61,60,59,58,58,57,56,56,55,55,54,53,53,52,51,51,50,50,49,48,48,47,47,46,46,45,44,43,43,42,41,40,40,39,39,38,37,37,36,35,35,34,33,33,32,31,31,30,29,29,28,28,27,26,26,25,24,24,23,22,22,21,21,20,19,19};
uint8 traceR[MT9V03X_H]=        //右边界轨迹
{179,106,106,108,108,109,109,110,111,112,113,113,114,115,115,116,117,117,118,119,119,120,121,121,122,123,124,124,125,126,126,127,128,128,129,130,131,131,132,133,134,134,135,136,137,137,138,139,139,140,141,141,142,143,144,144,145,146,147,147,148,149,149,150,151,152,152,153,154,155,155,156,157,157,158,159,159,160,161,162,162,163,164,165,165,166,167,167,168,169};

uint8 trace_middle[MT9V03X_H]=         //赛道宽度
{179, 32, 33, 34, 35, 37, 37, 39, 41, 42, 44, 44, 46, 48, 48, 50, 52, 52, 54, 55, 56, 58, 59, 59, 61, 63, 65, 66, 67, 69, 70, 71, 73, 73, 75, 77, 78, 79, 82, 83, 84, 84, 86, 88, 89, 90, 92, 93, 93, 95, 97, 98, 99, 102, 103, 104, 105, 107, 108, 109, 111, 112, 113, 115, 116, 118, 119, 120, 122, 124, 125, 126, 128, 128, 130, 131, 132, 134, 135, 137, 138, 139, 141, 143, 143, 145, 146, 147, 149, 150};
/*------------------------------------------------------------------------------------------------------------------
函数简介     右边界轨迹 
参数说明     无
返回参数     无
使用示例    
备注信息     右边界轨迹是左边界轨迹加上中线
-------------------------------------------------------------------------------------------------------------------
*/
void trace_right_bu(int16 start,int16 end)
{
    if (start<0||end<0||start>=MT9V03X_H||end>=MT9V03X_H) //防止越界
    {
        return;
    }
    if (start>end) //如果起点大于终点，交换
    {
        int16 temp=start;
        start=end;
        end=temp;
    }
    
    for(int16 i=start;i<end;i++)
    {
        int16 temp=leftline[i]+trace_middle[i];
        if(temp>MT9V03X_W-1)
        {
            temp=MT9V03X_W-1;
        }
        rightfollowline[i]=temp; //右边界轨迹
        
    }
}

/*------------------------------------------------------------------------------------------------------------------
函数简介     左边界轨迹
参数说明     无
返回参数     无
使用示例
备注信息     左边界轨迹是右边界轨迹减去赛道宽度
-------------------------------------------------------------------------------------------------------------------
*/
void trace_left_bu(int16 start,int16 end)
{
    if (start<0||end<0||start>=MT9V03X_H||end>=MT9V03X_H) //防止越界
    {
        return;
    }
    if (start>end) //如果起点大于终点，交换
    {
        int16 temp=start;
        start=end;
        end=temp;
    }
    for(int16 i=start;i<end;i++)
    {
        int16 temp=rightline[i]-trace_middle[i];
        if(temp<0)
        {
            temp=0;
        }
        leftfollowline[i]=temp; //左边界轨迹
    }
}

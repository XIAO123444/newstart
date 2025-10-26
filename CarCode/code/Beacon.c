#include "Beacon.h"
int16 available_beacon_num=0; //记录信标灯数量
int16 on_beacon_num_count=0; //记录点亮的信标灯数量
float R=0;      //翼展半径
Struct_Beacon_t_typedef Beacon_raw_info[MAX_BEACON_NUM]; //信标灯信息原始结构体数组
Struct_Beacon_t_typedef Beacon_show_info[MAX_BEACON_NUM]; //用于显示的信标灯信息结构体数组
#define ips200_x_max (240)
#define ips200_y_max (320)

/*——-------------------------------------------------------------------------------------------------
函数简介     计算信标灯距离矩阵
参数说明     无
返回参数     无
使用示例        caculate_beacon_distance_matrix();
备注信息
---------------------------------------------------------------------------------------------------*/

void caculate_beacon_distance_matrix()
{
    for(int16 i=0;i<available_beacon_num;i++)
    {
        for(int16 j=0;j<available_beacon_num;j++)
        {
            if(i==j)
            {
                Matrix_Beacon[i][j]=0;
            }
            else
            {
                int16 delta_x=Beacon_show_info[i].x-Beacon_show_info[j].x;
                int16 delta_y=Beacon_show_info[i].y-Beacon_show_info[j].y;
                Matrix_Beacon[i][j]=sqrt((float)(delta_x*delta_x+delta_y*delta_y));
            }
        }
    }
}
int8 show_circle[][2] = {{-2,0},{-2,1},{-2,-1},{-1,2},{0,2},{1,2},{2,1},{2,0},{2,-1},{1,-2},{0,-2},{-1,-2}};

/*——-------------------------------------------------------------------------------------------------
函数简介     显示单个信标灯点
参数说明     x               信标灯X坐标
参数说明     y               信标灯Y坐标
参数说明     color           信标灯颜色
返回参数     无
使用示例        show_beacon_point(120,160,RGB565_RED);
备注信息
---------------------------------------------------------------------------------------------------*/

void show_beacon_point(int16 x,int16 y,rgb565_color_enum color)
{

    for(int i=0;i<12;i++)
    {
        if(x>2&&x<ips200_x_max-2&&y>2&&y<ips200_y_max-2)
        {
            ips200_draw_point((uint16)(x+show_circle[i][0]), (uint16)(y+show_circle[i][1]), color);
        }
    }
}


/*——-------------------------------------------------------------------------------------------------
函数简介     显示所有信标灯信息
参数说明     无
返回参数     无
使用示例        show_beacon_info();
备注信息
---------------------------------------------------------------------------------------------------*/
void calculate_beacon_trace(void)
{

    
}
void show_beacon_info()
{
    for(int i=0;i<available_beacon_num;i++)
    {   
        show_beacon_point(Beacon_show_info[i].x,Beacon_show_info[i].y,Beacon_show_info[i].color);
    }
}

 /*
 * camera.c
 *
 *  Created on: 2023��10��24��
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


int16 leftlostpoint[2]={0,0};   //�����������ߵ�0Ϊ��������1Ϊ��������
int16 rightlostpoint[2]={0,0};  //�Ҷ����������ߵ�0Ϊ��������1Ϊ��������
int16 bothlostpoint[2]={0,0};   //ͬʱ�����������ߵ�0Ϊ��������1Ϊ��������

int16 white_point_count[MT9V03X_W]={0}; //ÿ�а׵����
int16 white_point_count1[MT9V03X_W]   ={0}; //ÿ�а׵�����˲� 1   


int16 left_longest[2]={0,0};  //�����������������е�0���ȣ�1ΪW����
int16 right_longest[2]={0,0};  //�����������������е�0���ȣ�1ΪW����
int16 left_start_point=0;  //�����
int16 right_start_point=MT9V03X_W-1; //�����

//y�㴦��
int8 leftorright=0;                        //����������y�� ��Ϊ-1 ��Ϊ1 0Ϊ��Ч
int16 white_y_point=-1;                     //��y�� ,û�ҵ�Ϊ-1 
int16 leftblackpoint_index=-1;    //����y�ڵ㣬û�ҵ�Ϊ-1
int16 rightblackpoint_index=-1;   //����y�ڵ㣬û�ҵ�Ϊ-1

int16 boundry_start_left=0; //��߽���ʼ��
int16 boundry_start_right=0; //�ұ߽���ʼ��

int16 search_stop=0; //��ֹ��
int16 search_stop1=0; //��ֹ��1

uint16 left_lost_flag[MT9V03X_H];//��������   0-δ���ߣ�1-����
uint16 right_lost_flag[MT9V03X_H];//�Ҷ�������  0-δ���ߣ�1-����
uint16 both_lost_flag[MT9V03X_H];//ͬʱ�������� 0-δ���ߣ�1-����

//ʮ�֡�������
int16 Right_Down_Find=0;    //���µ�
int16 Left_Down_Find=0;     //���µ�
int16 Right_Up_Find=0;      //���ϵ�
int16 Left_Up_Find=0;       //���ϵ�

//ʮ�֡�������
//��״̬
enum mark {
    straight,    // ֱ����ʻ
    crossroad,   // ʮ��·��
    round_2,   // �뻷��ֱ��
    round_3,   // Բ����б�ߣ�δʹ�ã�
    round_4,   // �뻷��ʻ
    round_5,   // ��յ㲹б��
    round_6    // ������ֱ��
};
extern enum  mark carstatus_now;
//����
extern int16 bailie_lock_crossroad;
extern int16 bailieright_lock_round;
extern bool start_flag; //������־λ

//�ߵ��붪�ߡ�������
uint8 leftline_num;         //���ߵ�����
uint8 rightline_num;        //���ߵ�����


//Բ����������
int16 right_down_guai   =0;            //���¹յ�
int16 right_up_guai     =0;            //���Ϲյ�
int16 left_down_guai    =0;            //���¹յ�
int16 left_up_guai      =0;            //���Ϲյ�

int16 left_budandiao       =0;     //�󲻵���
int16 right_budandiao      =0;     //�Ҳ�����
//Բ����������



//��Ⱥ͡�������
int16 sar_thre = 17;//��Ⱥ���ֵ
//��Ⱥ͡�������
uint8 pix_per_meter = 20;//ÿ�׵�������

extern int16 threshold_up;  //�����ֵ����
extern int16 threshold_down; //�����ֵ����

extern bool stop_flag1;

float dx1[5]={0};
float dx2[5]={0};

int16 right_down_line =0;
/*
// //���ﶼ�ǲ�Ⱥ͡�������������
// 
// ------------------------------------------------------------------------------------------------------------------
// �������     ��Ⱥ�Ѱ�ұ߽��
// ����˵��     ��
// ���ز���     ��
// ʹ��ʾ��     ֱ�ӵ���
// ��ע��Ϣ     ��
// -------------------------------------------------------------------------------------------------------------------

// void image_boundary_process(void){
//     uint8 row;//��
//     //uint8 col = MT9V03X_W/2;//��
//     uint8 start_col = MT9V03X_W / 2;//��������������,Ĭ��ΪMT9V03X_W / 2
//     //����֮ǰ�ļ���
//     leftline_num = 0;
//     rightline_num = 0;

//     for(row = MT9V03X_H - 1; row >= 1; row--){
//         //ѡ����һ�е��е���Ϊ��һ�м�����ʼ�㣬��ʡ�ٶȣ�ͬʱ��ֹ������������߾������뻭��һ��
//         if(row != MT9V03X_H - 1){
//             start_col = (uint8)(0.4 * centerline[row] + 0.3 * start_col + 0.1 * MT9V03X_W);//һ�׵�ͨ�˲�����ֹ�������Ӱ����һ�е���ʼ��


//         }
//         else if(row == MT9V03X_H - 1){
//             start_col = MT9V03X_W / 2;
//         }
//         if(start_col<MT9V03X_W/2-30){start_col=MT9V03X_W/2-30;}
//         if(start_col>MT9V03X_W/2+30){start_col=MT9V03X_W/2+30;}
//         //��������Ⱥ� 
//         difsum_left(row,start_col);
//         difsum_right(row,start_col); 
//         centerline[row] = 0.5 * (rightline[row] + leftline[row]);
//     }
// }
//
// ------------------------------------------------------------------------------------------------------------------
// �������     ��Ⱥ�Ѱ�����߽��
// ����˵��     
// ���ز���     
// ʹ��ʾ��     
// ��ע��Ϣ     
// -------------------------------------------------------------------------------------------------------------------

// void difsum_left(uint8 y,uint8 x){
//     float sum,dif,sar;//�ͣ����
//     uint8 col;//��
//     uint8 mov = 3;//ÿ���������ƶ���,Ĭ��Ϊ2�����Ը��ݻ���ֱ��ʵ���
//     //�����x�е���߽�
//     leftline[y] = 0;//δ�ҵ���߽�ʱ���Ϊ0
//     for(col = x; col >= mov + 1; col -= mov){
//         dif = (float)((mt9v03x_image[y][col] - mt9v03x_image[y][col - mov - 1])<<8);//����8λ����256���ɱ��⸡�����ˣ��ӿ��ٶ�
//         sum = (float)((mt9v03x_image[y][col] + mt9v03x_image[y][col - mov - 1]));
//         sar = fabs(dif / sum);//��ȡ��Ⱥ�
//         if(sar > sar_thre){//��Ⱥʹ�����ֵ������ǳɫͻ��
//             leftline[y] = (int16)(col - mov);
//             leftline_num ++;//���ߵ����+
//             break;//�ҵ��߽���˳�
//         }
//     }
// }

// ------------------------------------------------------------------------------------------------------------------
// �������     ��Ⱥ�Ѱ���Ҳ�߽��
// ����˵��     
// ���ز���     
// ʹ��ʾ��     
// ��ע��Ϣ     
// -------------------------------------------------------------------------------------------------------------------
// 
// void difsum_right(uint8 y,uint8 x){
//     float sum,dif,sar;//�ͣ����
//     uint8 col;//��
//     uint8 mov = 3;//ÿ���������ƶ���,Ĭ��Ϊ2�����Ը��ݻ���ֱ��ʵ���
//     //�����x�е���߽�
//     rightline[y] = MT9V03X_W - 1;//δ�ҵ��ұ߽�ʱ���Ϊ187
//     for(col = x; col <= MT9V03X_W - mov - 1; col += mov){
//         dif = (float)((mt9v03x_image[y][col] - mt9v03x_image[y][col + mov + 1])<<8);//����8λ����256���ɱ��⸡�����ˣ��ӿ��ٶ�
//         sum = (float)((mt9v03x_image[y][col] + mt9v03x_image[y][col + mov + 1]));
//         sar = fabs(dif / sum);//��ȡ��Ⱥ�
//         if(sar > sar_thre){//��Ⱥʹ�����ֵ������ǳɫͻ��
//             rightline[y] = (int16)(col + mov);
//             rightline_num ++;//���ߵ����+
//             break;//�ҵ��߽���˳�
//         }
//     }
// }


// //���ﶼ�ǲ�Ⱥ͡�������������
*/
//��򷨡�������������������
uint8 dis_image[MT9V03X_H][MT9V03X_W];
int16 image_threshold = 46;
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ���ٴ������ֵ������ɽ��
  @param     image       ͼ������
             col         �� �����
             row         �У�����
  @return    null
  Sample     threshold=my_adapt_threshold(mt9v03x_image[0],MT9V03X_W, MT9V03X_H);//ɽ�����ٴ��
  @note      ��˵�ȴ�ͳ��򷨿�һ�㣬ʵ��ʹ��Ч�����
-------------------------------------------------------------------------------------------------------------------*/
int my_adapt_threshold(uint8 *image, uint16 col, uint16 row)   //ע�������ֵ��һ��Ҫ��ԭͼ��
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j;
    int pixelSum = width * height/4;
    int threshold = 0;
    uint8* data = image;  //ָ���������ݵ�ָ��
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }
    uint32 gray_sum=0;
    //ͳ�ƻҶȼ���ÿ������������ͼ���еĸ���
    for (i = 0; i < height; i+=2)
    {
        for (j = 0; j < width; j+=2)
        {
            pixelCount[(int)data[i * width + j]]++;  //����ǰ�ĵ������ֵ��Ϊ����������±�
            gray_sum+=(int)data[i * width + j];       //�Ҷ�ֵ�ܺ�
        }
    }
    //����ÿ������ֵ�ĵ�������ͼ���еı���
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
    }
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
    for (j = 0; j < GrayScale; j++)
    {
        w0 += pixelPro[j];  //��������ÿ���Ҷ�ֵ�����ص���ռ����֮��   ���������ֵı���
        u0tmp += j * pixelPro[j];  //�������� ÿ���Ҷ�ֵ�ĵ�ı��� *�Ҷ�ֵ
        w1=1-w0;
        u1tmp=gray_sum/pixelSum-u0tmp;
        u0 = u0tmp / w0;              //����ƽ���Ҷ�
        u1 = u1tmp / w1;              //ǰ��ƽ���Ҷ�
        u = u0tmp + u1tmp;            //ȫ��ƽ���Ҷ�
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
    uint8 col;//��
    uint8 mov = 1; //ÿ���������ƶ���,Ĭ��Ϊ2�����Ը��ݻ���ֱ��ʵ���
    //�����x�е���߽�
    leftline[y] = 0;//δ�ҵ���߽�ʱ���Ϊ0
    for(col = x; col >= mov + 1; col -= mov)
	{
		if(dis_image[y][col] - dis_image[y][col - mov]>0)
		{
			leftline[y] = col;
			leftline_num ++;//���ߵ����+
            return;
		}

	}
    leftlostpoint[0]++;
    left_lost_flag[y]=1;
    //��Ϊ���ߣ���
    if(leftlostpoint[1]==0)
    {
        leftlostpoint[1]=y;
    }
}
void difsum_right1(uint8 y,uint8 x)
{
	uint8 col;//��
    uint8 mov = 1;//ÿ���������ƶ���,Ĭ��Ϊ2�����Ը��ݻ���ֱ��ʵ���
    //�����x�е���߽�
    rightline[y] = MT9V03X_W-1;//δ�ҵ���߽�ʱ���Ϊ0
    for(col = x; col <= MT9V03X_W - mov - 1; col += mov)
	{
		if(dis_image[y][col] - dis_image[y][col + mov]>5)
		{
			rightline[y] = col ;
			rightline_num ++;//�ұ��ߵ����+
			return;//�ҵ��߽���˳�
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
�������     ������ʼ��
����˵��     ��
���ز���     ��
ʹ��ʾ��     param_init();
��ע��Ϣ     ��
-------------------------------------------------------------------------------------------------------------------
*/
void param_init(void)
{
    leftline_num = 0;
    rightline_num = 0;


    leftorright=0;                        //����������y��
    white_y_point=0;                     //��y��
    leftblackpoint_index=-1;            //����y�ڵ�
    rightblackpoint_index=-1;           //����y�ڵ�
    left_longest[0]=1;      //�����������
    right_longest[0]=1;     //�����������
    leftlostpoint[0]=0;      //����������
    rightlostpoint[0]=0;     //�Ҷ���������
    bothlostpoint[0]=0;      //ͬʱ����������

    left_longest[1]=0;      //���������������
    right_longest[1]=0;     //���������������
    leftlostpoint[1]=0;      //���ߵ�����
    rightlostpoint[1]=0;     //�Ҷ��ߵ�����
    bothlostpoint[1]=0;      //ͬʱ���ߵ�����

    //���߼������������������
    for(int16 i=0;i<MT9V03X_H;i++)
    {
        leftline[i]=0;              //������0
        rightline[i]=MT9V03X_W-1;   //������0
        right_lost_flag[i]=0;     //�Ҷ�����0   
        left_lost_flag[i]=0;      //������0   
        both_lost_flag[i]=0;      //ͬʱ������0
        centerline[i]=0;          //������0
    }
    for(int16 i=0;i<MT9V03X_W;i++)
    {
        white_point_count[i]=0;     //�׵������0
        white_point_count1[i]=0;    //�׵�����˲�1��0
    }
}

int16 Threshold= 4 ;
int16 Thresholdnum=10;

/*
-------------------------------------------------------------------------------------------------------------------
�������    ͼ��y�㴦��
����˵��     ��������㣬����㣬������Բ��ʱ����
���ز���     white_y_point;leftorright;(1���ң�-1����)
ʹ��ʾ��     find_y_point();
��ע��Ϣ     ��
-------------------------------------------------------------------------------------------------------------------
*/  
void find_y_point(void)
{
    for(int i=left_start_point+15;i<right_start_point-15;i++)
    {
        bool bigbreak=false;
        leftblackpoint_index=-1;
        rightblackpoint_index=-1;
        for(int j=1;i-j>=0&&j<16;j++)//������
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
    uint8 row;//��
    param_init();
    //����м���
    for(int16 i=left_start_point;i<right_start_point;i+=1)
    {
        for(int16 j=MT9V03X_H-1;j>0;j--)
        {
            if(dis_image[j][i]==255) 
            {
                white_point_count[i]++;     //�׵����
            }
            else
            {
                break;                  //������ɫ���߽���
            }
        }
    }
    memcpy(white_point_count1, white_point_count, sizeof(white_point_count)); //���׵�������Ƶ��׵�����˲�1
    find_y_point(); 
    if(leftorright==0)
    {
        ips200_show_string(0,240,"middle");
        ips200_show_int(60,240,0,3);
        for(int16 i=left_start_point;i<right_start_point;i+=1)       //Ѱ��������
        {

            if(white_point_count1[i]>left_longest[0])
            {
                left_longest[0]=white_point_count1[i];           
                left_longest[1]=i;
            }
        }
        for(int16 i=right_start_point;i>left_start_point;i-=1)       //Ѱ����  ���Ұ���
        {

            if(white_point_count1[i]>right_longest[0]) 
            {
                right_longest[0]=white_point_count1[i];         
                right_longest[1]=i;
            }
        }
    }
    else if(leftorright==1) //������Ҳ���y��
    {
        ips200_show_string(0,240,"right ");
        ips200_show_int(60,240,white_y_point,3);
        for(int16 i=white_y_point;i<MT9V03X_W-1;i+=1)       //Ѱ��������
        {

            if(white_point_count1[i]>left_longest[0])
            {
                left_longest[0]=white_point_count1[i];           
                left_longest[1]=i;
            }
        }
        for(int16 i=MT9V03X_W-1;i>white_y_point;i-=1)       //Ѱ����  ���Ұ���
        {

            if(white_point_count1[i]>right_longest[0]) 
            {
                right_longest[0]=white_point_count1[i];         
                right_longest[1]=i;
            }
        }
    }
    else if(leftorright==-1) //����������y��
    {
                ips200_show_string(0,240,"left ");
        ips200_show_int(60,240,white_y_point,3);
        for(int16 i=white_y_point;i>=0;i-=1)       //Ѱ��������
        {

            if(white_point_count1[i]>left_longest[0])
            {
                left_longest[0]=white_point_count1[i];           
                left_longest[1]=i;
            }
        }
        for(int16 i=0;i<white_y_point;i+=1)       //Ѱ����  ���Ұ���
        {

            if(white_point_count1[i]>right_longest[0]) 
            {
                right_longest[0]=white_point_count1[i];         
                right_longest[1]=i;
            }
        }
    }
    

    search_stop=(right_longest[0]< left_longest[0])?(MT9V03X_H-right_longest[0]-1):(MT9V03X_H-1-left_longest[0]); //�����Ǵ���Ļ�����ϣ�������ѡ���
    search_stop1=search_stop; //������ֹ��1
    if(search_stop==-1)
    {
        search_stop=0;//��ֹԽ��
    }
    if(search_stop>=MT9V03X_H-1||search_stop<0) //��������С��10�У�˵��û�а���
    {
        return; //û�а��ߣ�ֱ�ӷ��� 
    }
    else
    {
        for(row = MT9V03X_H - 1; row > search_stop; row--)
        {
            difsum_left1(row,left_longest[1]); //ʹ������е������Ϊ���Ѱ������
            difsum_right1(row,right_longest[1]); //ʹ������е������Ϊ���Ѱ������
        }
        
        
    }
      //������
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
        if(right_lost_flag[row]==1&&left_lost_flag[row]==1) //ͬʱ����
        {
            bothlostpoint[0]++;
            bothlostpoint[1]=row;
            both_lost_flag[row]=1;
        }
        if(boundry_start_left==0&&leftline[row]!=0) {boundry_start_left=row;}   //��߽���ʼ��
        if(boundry_start_right==0&&rightline[row]!=MT9V03X_W-1) {boundry_start_right=row;} //�ұ߽���ʼ��
    }
}


//��򷨡�������������������
//�����ߴ���ͱ���������
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
                if(count>=10)//�����ɫ�������ڵ���40����Ϊ�ǰ�����
                {
                    
                    stop_flag1=true;
                    start_flag=false;
                }
            }
}


/*
------------------------------------------------------------------------------------------------------------------
�������      ����е�λ�ã������Ǵӵ����������ĸ�
����˵��     ��
���ز���     int16���е�ĺ�����
ʹ��ʾ��     
��ע��Ϣ     
-------------------------------------------------------------------------------------------------------------------
*/
int16 output_middle(void)
{
    return centerline[MT9V03X_H-15];
}
//ʮ���ж�
//ʮ���ж�
//ʮ���ж�
//ʮ���ж�
//ʮ���ж�
//ʮ���ж�

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ������������յ㣬��ʮ��ʹ��
  @param     �����ķ�Χ��㣬�յ�
  @return    �޸�����ȫ�ֱ���
             Right_Down_Find=0;
             Left_Down_Find=0;
  Sample     Find_Down_Point(int16 start,int16 end)
  @note      ������֮��鿴��Ӧ�ı�����ע�⣬û�ҵ�ʱ��Ӧ��������0
-------------------------------------------------------------------------------------------------------------------*/
void Find_Down_Point(int16 start,int16 end)
{
    int16 i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;
    if(start<end)               //���������ң����������ͼ������
    {
        t=start;
        start=end;
        end=t;
    }
    if(start>=MT9V03X_H-1-3)//����5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start=MT9V03X_H-1-3;
    if(end>=MT9V03X_H-5)
        end=MT9V03X_H-5;
    if(end<=5)
       end=5;
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find==0&&//ֻ�ҵ�һ�����������ĵ�
            (leftline[i]>0)&&//��߽�㲻��Ϊ0
           abs(leftline[i]-leftline[i+1])<=3&&//�ǵ����ֵ���Ը���
           abs(leftline[i+1]-leftline[i+2])<=3&&
           abs(leftline[i+2]-leftline[i+3])<=3&&
            ((leftline[i]-leftline[i-2])>=3)&&
            ((leftline[i]-leftline[i-3])>=5)&&
            ((leftline[i]-leftline[i-4])>=5))
        {
            Left_Down_Find=i+2;//��ȡ��������
            if(Left_Down_Find==start+2)
            {
                Left_Down_Find=0;//�������ʼ�У�˵��û���ҵ�
            }
        }
        if(Right_Down_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(rightline[i]-rightline[i+1])<=3&&//�ǵ����ֵ���Ը���
           abs(rightline[i+1]-rightline[i+2])<=3&&
           abs(rightline[i+2]-rightline[i+3])<=3&&
            rightline[i]<MT9V03X_W-1&&//�ұ߽�㲻��ΪMT9V03X_W-1
              ((rightline[i]-rightline[i-2])<=-3)&&
              ((rightline[i]-rightline[i-3])<=-5)&&
              ((rightline[i]-rightline[i-4])<=-5))
        {
            Right_Down_Find=i+2;
            if(Right_Down_Find==start+2)
            {
                Right_Down_Find=0;//�������ʼ�У�˵��û���ҵ�
            }
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0)//�����ҵ����˳�
        {
            break;
        }
    }

}
 
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ������������յ㣬��ʮ��ʹ��,����������
  @param     �����ķ�Χ��㣬�յ�
  @return    �޸�����ȫ�ֱ���
             Left_Up_Find=0;
             Right_Up_Find=0;
  Sample     Find_Up_Point(int16 start,int16 end)
  @note      ������֮��鿴��Ӧ�ı�����ע�⣬û�ҵ�ʱ��Ӧ��������0
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
    //start<end��������
    if(end>=MT9V03X_H-5)
        end=MT9V03X_H-5;
    if(end<=5)//��ʱ����зǳ�����ҲҪ�������ֵ㣬��ֹ����Խ��
        end=5;
    if(start<=5)//����5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start=5;
    if(start<search_stop+5)
    {
        start =search_stop+5;
    }

    for(i=start;i<=end;i++)
    { 
        if(Left_Up_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(leftline[i]-leftline[i-1])<=3&&
           abs(leftline[i-1]-leftline[i-2])<=3&&
           abs(leftline[i-2]-leftline[i-3])<=3&&
           leftline[i-2]-leftline[i]>-2&&
            leftline[i-3]-leftline[i]>-3&&
              ((leftline[i]-leftline[i+2])>=3)&&
              ((leftline[i]-leftline[i+3])>=5)&&
              ((leftline[i]-leftline[i+4])>=10))
        {

            Left_Up_Find=i-2;//��ȡ��������
            if(Left_Up_Find==start-2)
            {
                Left_Up_Find=0;//�������ʼ�У�˵��û���ҵ�
            }
        }
        if(Right_Up_Find==0&&//ֻ�ҵ�һ�����������ĵ�
           abs(rightline[i]-rightline[i-1])<=3&&//��������λ�ò��
           abs(rightline[i-1]-rightline[i-2])<=3&&
           abs(rightline[i-2]-rightline[i-3])<=3&&
           rightline[i-2]-rightline[i]<2&&
           rightline[i-3]-rightline[i]<3&&
              ((rightline[i]-rightline[i+2]<=-3))&&
              ((rightline[i]-rightline[i+3])<=-5)&&
              ((rightline[i]-rightline[i+4])<=-10))
        {
            Right_Up_Find=i-2;//��ȡ�������� 
            if(Right_Up_Find==start-2)
            {
                Right_Up_Find=0;//�������ʼ�У�˵��û���ҵ�
            }
        }
        if(Left_Up_Find!=0&&Right_Up_Find!=0)//���������ҵ��ͳ�ȥ
        {
            break;
        }
    }
 
     
}

/*
------------------------------------------------------------------------------------------------------------------
�������     ����ƽ���˲���
����˵��     ��
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     
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
�������     ����ƽ���˲���
����˵��     ��
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     
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

//Բ���ж�
//1.��Բ��

/*-------------------------------------------------------------------------------------------------------------------
  @brief     ���½ǵ���
  @param     ��ʼ�У���ֹ��
  @return    ���ؽǵ����ڵ��������Ҳ�������0
  Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      �ǵ�����ֵ�ɸ���ʵ��ֵ����
-------------------------------------------------------------------------------------------------------------------*/
int16 Find_Left_Down_Point(int16 start,int16 end)//�����½ǵ㣬����ֵ�ǽǵ����ڵ�����
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
        start=MT9V03X_H-2;//����5������5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
    }
    if(end<4)
    {
        end=4;//��ʱ����зǳ�����ҲҪ�������ֵ㣬��ֹ����Խ��    
    }
    if(leftline[start]<=0&&leftline[start-1]<=0&&leftline[start-2]<=0&&leftline[start-3]<=0)//�����ʼ��Ͷ����ߣ�û�йյ��жϵ�����
    {
        return 0;
    }
    for(int i=start;i>=end;i--)//���������ҽǵ� 
    {
        if(((leftline[i]-leftline[i-2])>=8||leftline[i-2]==0)&&
           ((leftline[i]-leftline[i-3])>=15||leftline[i-3]==0)&&
           ((leftline[i]-leftline[i-4])>=15||leftline[i-4]==0))
        {
            left_down_line=i+1;//��ȡ��������
            break;
        }
    }
    return left_down_line;
}
/*-------------------------------------------------------------------------------------------------------------------
  @brief     ���½ǵ��⣨Բ����
  @param     ��ʼ�У���ֹ��
  @return    ���ؽǵ����ڵ��������Ҳ�������0
  Sample     right_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      ������ж϶�ͼ��ĸɾ��̶�Ҫ��ϸߣ��ǵ�����ֵ�ɸ���ʵ��ֵ����
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
        start=MT9V03X_H-3;//����5������5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
    }
    if(end<5)
    {
        end=5;//��ʱ����зǳ�����ҲҪ�������ֵ㣬��ֹ����Խ��    
    }
    if(rightline[start]>=MT9V03X_W-1&&rightline[start-1]>=MT9V03X_W-1&&rightline[start-2]>=MT9V03X_W-1&&rightline[start-3]>=MT9V03X_W-1)//�����ʼ��Ͷ����ߣ�û�йյ��жϵ�����
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
            right_down_line=i+1;//��ȡ��������
            break;
        }
    }
    return right_down_line;
}

int16 Find_Right_Up_Point(int16 start,int16 end)//���ĸ��ǵ㣬����ֵ�ǽǵ����ڵ�����
{
    int16 i,t;
    int16 right_up_line=0;
    if(start>end)
    {
        t=start;
        start=end;
        end=t;
    }    
    if(start<=3)//��ʱ����зǳ�����ҲҪ�������ֵ㣬��ֹ����Խ��
    {start=3;}
    if(end>=MT9V03X_H-1-4)
    {end=MT9V03X_H-1-4;}
    if(rightline[start]==MT9V03X_W-1&&rightline[start+1]==MT9V03X_W-1&&rightline[start+2]==MT9V03X_W-1)
    {
        return 0;//�����ʼ��Ͷ����ߣ�û�йյ��жϵ�����
    }
    for(i=start;i<=end;i++)
    {
        if(rightline[i]==MT9V03X_W-1)//����ұ߽����MT9V03X_W-1��˵��û���ҵ�
        {
            break;  //ֱ������
        }
        if(rightline[i]>=rightline[i-1]&&
            rightline[i-1]>=rightline[i-2]&&
           ((rightline[i]-rightline[i+2])<=-8)&&
           ((rightline[i]-rightline[i+3])<=-15||rightline[i+3]==MT9V03X_W-1)&&
           ((rightline[i]-rightline[i+4])<=-15||rightline[i+4]==MT9V03X_W-1))
        {
            right_up_line=i;//��ȡ��������
            break;
        }
    }
    return right_up_line;
}

int16 Find_Left_Up_Point(int16 start,int16 end)//���ĸ��ǵ㣬����ֵ�ǽǵ����ڵ�����
{
    int16 i,t;
    int16 left_up_line=0;
    if(start>end)
    {
        t=start; 
        start=end;
        end=t;
    }    
    if(start<=1)//��ʱ����зǳ�����ҲҪ�������� �㣬��ֹ����Խ��
    {start=1;}
    if(end>=MT9V03X_H-1-4)
    {end=MT9V03X_H-1-4;}
    if(leftline[start]==0&&leftline[start+1]==0&&leftline[start+2]==0)
    {
        return 0;//�����ʼ��Ͷ����ߣ�û�йյ��жϵ�����
    }
    for(i=start;i<=end;i++)//���������ҽǵ�
    {
        if(((leftline[i]-leftline[i+2])>=8)&&
           ((leftline[i]-leftline[i+3])>=15||leftline[i+3]==0)&&
           ((leftline[i]-leftline[i+4])>=15||leftline[i+4]==0))
        {
            left_up_line=i;//��ȡ��������
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
        start=5; //��ֹ����Խ��
    }
    if(end>MT9V03X_H-6)
    {
        end=MT9V03X_H-6; //��ֹ����Խ��
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
             return result; //�ҵ���һ�����������ĵ�ͷ���
        }
        
    }
    return result; //���û���ҵ�������0
}
/*
------------------------------------------------------------------------------------------------------------------
�������     ȷ�������ԣ����ڴ����Ҳ��ߡ�
����˵��     uint8�����յ�
���ز���     �е�λ��
ʹ��ʾ��     
��ע��Ϣ     
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
    if(start>=MT9V03X_H-2)//����Խ�籣��
        start=MT9V03X_H-2;
    if(end<=1)
    {
        end=1;
    }
    for(i=start;i>=end;i--)
    {
        if(abs(rightline[i]-rightline[i-1])>=5)//��������ֵ��5���ɸ���
       {
            continuity_change_flag=i;                                         //��i����������

            break;
       }

    }

    return continuity_change_flag;
}
/*
------------------------------------------------------------------------------------------------------------------
�������     ȷ����start��end�����ԣ����ڴ�������ߡ�
����˵��     uint8�����յ�
���ز���     �е�λ��
ʹ��ʾ��     
��ע��Ϣ     
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
    if(start>=MT9V03X_H-2)//����Խ�籣��
        start=MT9V03X_H-2;
    if(end<=1)
    {
        end=1;
    }

    for(i=start;i>=end;i--)
    {
        if(abs(leftline[i]-leftline[i-1])>=5)//��������ֵ��5���ɸ���
       {

            continuity_change_flag=i;                                         //��i����������

            break;
       }

    }
    return continuity_change_flag;
}
//�����Ա仯s
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
    if(start>=MT9V03X_H-6)//����Խ�籣��
        start=MT9V03X_H-6;
    if(end<=6)//������ܸ�
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

    // ȷ��start >= end�����������
    if (start < end) {
        uint8 t = start;
        start = end;
        end = t;
    }

    // ����Խ�籣�������ҵ����ԶԳƣ�
    if (start >= MT9V03X_H - 6) {
        start = MT9V03X_H - 6;
    }
    if (end <= 6) {  // �󵥵����豣��5��ƫ����
        end = 6;
    }

    // ����������Ӹߵ��ͣ�
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
���¶��ǲ���
���¶��ǲ���
���¶��ǲ���
���¶��ǲ���
���¶��ǲ���
-------------------------------------------------------------------------------------------------------------------
*/


/*
------------------------------------------------------------------------------------------------------------------
�������     ������
����˵��     ���x��y�յ�y��������dx
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     ������dx����x1-x2/y1-y2
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
        if(temp<0) temp=0; //��ֹԽ��
        if(temp>MT9V03X_W-1) temp=MT9V03X_W-1; //��ֹԽ��
        leftfollowline[i] = temp;

    }
}

/*
------------------------------------------------------------------------------------------------------------------
�������     ������
����˵��     ���x��y�յ�y��������dx
���ز���     ��  
ʹ��ʾ��     
��ע��Ϣ     ������dx����x1-x2/y1-y2
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
        if(temp<0) temp=0; //��ֹԽ��
        if(temp>MT9V03X_W-1) temp=MT9V03X_W-1; //��ֹԽ��    
        rightfollowline[i] = temp;
    }
}
/*
------------------------------------------------------------------------------------------------------------------
�������     �����������
����˵��     ���xy���յ�xy
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     
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
�������     �����������
����˵��     ���xy���յ�xy
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     �ܳ���0������ţ��
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
�������     ���϶��²�����
����˵��     ���
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     
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
�������     ���϶��²�����
����˵��     ���
���ز���     ��
ʹ��ʾ��      
��ע��Ϣ     
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
�������     ���¶��ϲ�����
����˵��     ���
���ز���     ��
ʹ��ʾ��     
��ע��Ϣ     
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
�������     ���¶��ϲ�����
����˵��     ���
���ز���     ��
ʹ��ʾ��      
��ע��Ϣ     
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

uint8 traceL[MT9V03X_H]=        //��߽�켣
{0,74,73,74,73,72,72,71,70,70,69,69,68,67,67,66,65,65,64,64,63,62,62,62,61,60,59,58,58,57,56,56,55,55,54,53,53,52,51,51,50,50,49,48,48,47,47,46,46,45,44,43,43,42,41,40,40,39,39,38,37,37,36,35,35,34,33,33,32,31,31,30,29,29,28,28,27,26,26,25,24,24,23,22,22,21,21,20,19,19};
uint8 traceR[MT9V03X_H]=        //�ұ߽�켣
{179,106,106,108,108,109,109,110,111,112,113,113,114,115,115,116,117,117,118,119,119,120,121,121,122,123,124,124,125,126,126,127,128,128,129,130,131,131,132,133,134,134,135,136,137,137,138,139,139,140,141,141,142,143,144,144,145,146,147,147,148,149,149,150,151,152,152,153,154,155,155,156,157,157,158,159,159,160,161,162,162,163,164,165,165,166,167,167,168,169};

uint8 trace_middle[MT9V03X_H]=         //�������
{179, 32, 33, 34, 35, 37, 37, 39, 41, 42, 44, 44, 46, 48, 48, 50, 52, 52, 54, 55, 56, 58, 59, 59, 61, 63, 65, 66, 67, 69, 70, 71, 73, 73, 75, 77, 78, 79, 82, 83, 84, 84, 86, 88, 89, 90, 92, 93, 93, 95, 97, 98, 99, 102, 103, 104, 105, 107, 108, 109, 111, 112, 113, 115, 116, 118, 119, 120, 122, 124, 125, 126, 128, 128, 130, 131, 132, 134, 135, 137, 138, 139, 141, 143, 143, 145, 146, 147, 149, 150};
/*------------------------------------------------------------------------------------------------------------------
�������     �ұ߽�켣 
����˵��     ��
���ز���     ��
ʹ��ʾ��    
��ע��Ϣ     �ұ߽�켣����߽�켣��������
-------------------------------------------------------------------------------------------------------------------
*/
void trace_right_bu(int16 start,int16 end)
{
    if (start<0||end<0||start>=MT9V03X_H||end>=MT9V03X_H) //��ֹԽ��
    {
        return;
    }
    if (start>end) //����������յ㣬����
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
        rightfollowline[i]=temp; //�ұ߽�켣
        
    }
}

/*------------------------------------------------------------------------------------------------------------------
�������     ��߽�켣
����˵��     ��
���ز���     ��
ʹ��ʾ��
��ע��Ϣ     ��߽�켣���ұ߽�켣��ȥ�������
-------------------------------------------------------------------------------------------------------------------
*/
void trace_left_bu(int16 start,int16 end)
{
    if (start<0||end<0||start>=MT9V03X_H||end>=MT9V03X_H) //��ֹԽ��
    {
        return;
    }
    if (start>end) //����������յ㣬����
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
        leftfollowline[i]=temp; //��߽�켣
    }
}

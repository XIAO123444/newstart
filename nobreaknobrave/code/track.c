
#include "track.h"
#include "photo_chuli.h"
#include "buzzer.h"
#include "speed.h"

// �ⲿ��������
uint8 cross_flag=0;


extern int32 forwardsight3;
extern int16 centerline[MT9V03X_H];      // ���������飨ͼ��߶�ά�ȣ�
extern int16 leftline[MT9V03X_H];       // ��߽������� 
extern int16 rightline[MT9V03X_H];      // �ұ߽�������
extern int16 rightfollowline[MT9V03X_H]; // �ұ߽������
extern int16 leftfollowline[MT9V03X_H];  // ��߽������
extern uint8 pix_per_meter;             // ����/�ױ���ϵ��
int16 centerline2[MT9V03X_H];           // ���μ����������

extern int16 boundry_start_left; //��߽���ʼ��
extern int16 boundry_start_right; //�ұ߽���ʼ��

extern int16 leftlostpoint[2];   //�����������ߵ�0Ϊ��������1Ϊ���ߵ�
extern int16 rightlostpoint[2];  //�Ҷ����������ߵ�0Ϊ��������1Ϊ���ߵ�
extern int16 bothlostpoint[2];   //ͬʱ�����������ߵ�0Ϊ��������1Ϊ����

extern int16 search_stop; // ��ֹ��
extern int16 search_stop1; // ��ֹ��1
// �߽������
extern int16 Right_Down_Find;  // ���±߽���к�
extern int16 Left_Down_Find;   // ���±߽���к�
extern int16 Right_Up_Find;    // ���ϱ߽���к�
extern int16 Left_Up_Find;     // ���ϱ߽���к�
int16 lastrightupfind=0; // �ϴ����ϵ�
int16 lastrightdownfind=0; // �ϴ����µ�
int16 lastleftupfind=0; // �ϴ����ϵ�
int16 lastleftdownfind=0; // �ϴ����µ�
extern int16 right_budandiao;       // �Ҳ�������
extern int16 left_budandiao;        // �󲻵�����
int16 v_point;


extern int16 right_down_guai[2];   // ���¹յ�
extern int16 right_up_guai[2];     // ���Ϲյ� 
extern int16 left_down_guai[2];    // ���¹յ�
extern int16 left_up_guai[2];      // ���Ϲյ�


extern int16 left_start_point;  //�����
extern int16 right_start_point; //�����

int16 continuity_pointLeft[2]={0,0}; // ��������[0]��ĳ�У�[1]��ĳ��
int16 continuity_pointRight[2]={0,0}; // �Ҳ������� [0]��ĳ�У�[1]��ĳ��


//��Ȩ����
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

// Բ����־
float right_dxbudandiao;            // �Ҳ�������б��

extern uint8 leftline_num;         //���ߵ�����
extern uint8 rightline_num;        //���ߵ�����

extern int32 forwardsight2;
extern int32 forwardsight_stragety;

uint8 crossconfirm=0;

int16 output_middle2(void) {
    int16 result;
    if(search_stop<forwardsight_stragety)            //�����ֹ��Զ�� ǰ�Ӿ���
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
    //�������
    for(i=MT9V03X_H-1;i>=MT9V03X_H-search_stop-1;i--)//����������
    {
        err+=(MT9V03X_W/2-((leftfollowline[i]+rightfollowline[i])>>1))*Weight[i];//����1λ����Ч��2
        weight_count+=Weight[i];
    }
    err=err/weight_count;
    return err;//ע��˴������������������С����ע����������


} 

int16 output_middle4(void) 
{
    int16 result; 
    if(search_stop1>MT9V03X_H-1-3)
    {
        return MT9V03X_W/2;         //��ֹ����Խ��
    }
    
    if(search_stop1<forwardsight3)            //�����ֹ��Զ�� ǰ�Ӿ���
    {
        result=(centerline2[forwardsight3]+centerline2[forwardsight3+1]+centerline2[forwardsight3+2])/3;        // ȡǰ�Ӿ����ƽ��ֵ 
        return result;                      
        
    }

    result=(centerline2[search_stop1]+centerline2[search_stop1+1]+centerline2[search_stop1+2])/3;          // ȡ��ֹ���ƽ��ֵ                  
    return result; // ������ֹ���ƽ��ֵ
} 

enum mark {
    straight,    // ֱ����ʻ
    crossroad,   // ʮ��·��
    crossroadL, // ��б��ʮ��
    crossroadR, // ��б��ʮ��
    round_1,
    round_2,   // �뻷��ֱ��a
    round_3,   // Բ����б�ߣ�δʹ�ã�
    round_4,   // �뻷��ʻ
    round_5,   // ��յ㲹б��
    round_6,    // ������ֱ��
    round_7    // ������б��
};
enum mark carstatus_now = straight;  // ��ǰ����״̬

bool output_addspeedflag(void)
{
    if (search_stop1<forwardsight2&&abs(centerline2[forwardsight2]-MT9V03X_W/2)<10&&carstatus_now!=round_1&&carstatus_now!=round_2&&carstatus_now!=round_3&&carstatus_now!=round_4&&carstatus_now!=round_5&&carstatus_now!=round_6&&carstatus_now!=round_7) // ǰ�Ӿ���С����ֹ���������߽ӽ�����
     // ˵��ǰ����·ƽ̹�����Լ���
     // ע�⣺�����15��һ������ֵ�����Ը���ʵ���������
    { 
        return true; // �����������
    } else
     {
        return false; // �������������
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
    // �������Ҹ����� 
    memcpy(leftfollowline, leftline, sizeof(leftline));
    memcpy(rightfollowline, rightline, sizeof(rightline));


    centerline2_change();
    v_point=find_vpoint(5,MT9V03X_W-5); // ����v��
    continuity_pointLeft[0]=continuity_left(MT9V03X_H-1,search_stop+2); // ���������ж�
    continuity_pointRight[0]=continuity_right(MT9V03X_H-1,search_stop+2); // ���������ж�
    continuity_pointLeft[1]=leftline[continuity_pointLeft[0]]; // �������Ե���
    continuity_pointRight[1]=rightline[continuity_pointRight[0]]; // �������Ե���
    Find_Up_Point(MT9V03X_H-1, search_stop); // �����ϰ�α߽��
    Find_Down_Point(MT9V03X_H-1, search_stop); //�����°�α߽��
    if(Left_Down_Find <= Left_Up_Find) {Left_Down_Find = 0;}
    if(Right_Down_Find <= Right_Up_Find){ Right_Down_Find = 0;}
    left_budandiao=montonicity_left(MT9V03X_H-1,search_stop+6); // �󲻵�����
    right_budandiao=montonicity_right(MT9V03X_H-1,search_stop+6); // �Ҳ�������

////    /*---------- ֱ��״̬��� ----------*/
    if(carstatus_now == straight) 
    {

        //ʮ���ж�
        // if(search_stop<13)
        // {
        //     if(continuity_pointLeft[0] != 0 && continuity_pointRight[0] != 0 && Right_Up_Find != 0 && Left_Up_Find != 0&&(Right_Up_Find>search_stop-2&&Left_Up_Find>search_stop-2))//���������ҵ� ���Ҳ��������ҵ��������Ϲյ��ҵ������Ϲյ��ҵ�����ʱΪ����ʮ��
        //     {
        //         carstatus_now = crossroad; // ����ʮ��·��״̬
        //         return;
        //     }
        //     if(continuity_pointLeft[0] != 0&& Left_Up_Find != 0&&Left_Up_Find>search_stop-2&&rightlostpoint[0]>50&&abs(left_budandiao-Left_Up_Find)>5)//���������ҵ� ���Ҳ�������δ�ҵ��������Ϲյ��ҵ�����ʱΪ��б��ʮ�ֺ����������ߵ�
        //     {
        //         carstatus_now = crossroadL; // ����ʮ��·��״̬
        //         return; 
        //     }

        //     if( continuity_pointRight[0] != 0 && Right_Up_Find != 0&&Right_Up_Find>search_stop-2&&leftlostpoint[0]>50)//��������δ�ҵ� �Ҳ��������ҵ��������Ϲյ��ҵ�����ʱΪ��б��ʮ��
        //     {
        //         carstatus_now = crossroadR; // ����ʮ��·��״̬
        //         return; 
        //     }
        // }
        // //Բ���ж�
        // if(right_budandiao&&Right_Down_Find>35&&rightlostpoint[0]>10&&rightlostpoint[0]<80&&search_stop<3&&leftlostpoint[0]<10&&Left_Up_Find==0)
        // //�Ҳ���������ڣ����¹յ���ڣ��Ҷ��ߵ�������10-40֮�䣬���ߵ�С��10����������ֹ��С��10
        // {
        //     carstatus_now=round_1; // �����뻷��ֱ��״̬
        //     BUZZ_START();
        //     return;
        // }

        // //ips200_show_string(0,300,"straig");
    }

//    /*---------- ʮ��·��״̬���� ----------*/
  if(carstatus_now == crossroad) {
       int start_down_point=5;
       int16 temp1_L=0;//��¼��һ����յ�
       int16 temp1_R=0;//��¼��һ���ҹյ� 

      //        // ȷ���°�α߽�㣨ȡ�����µ�


      /* �߽�����ϲ��� */
      if(Left_Down_Find != 0 && Right_Down_Find != 0) {
          // ���1�������µ����Ч �� ˫�߽�ֱ�����
          add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
                     Right_Up_Find-2, rightline[Right_Up_Find-2]);        // �ұ߽����
          add_Lline_k(leftline[Left_Down_Find], Left_Down_Find,   
                     Left_Up_Find-2, leftline[Left_Up_Find-2]);           // ��߽����
        //   //ips200_show_string(0,300,"cross1");
        //    printf("cross1");
      }
      else if(Left_Down_Find == 0 && Right_Down_Find != 0) {
          // ���2�������µ���Ч �� �ұ߽����+��߽��ӳ�
          add_Rline_k(rightline[Right_Down_Find], Right_Down_Find,        // �ұ߽����
                     Right_Up_Find, rightline[Right_Up_Find]);
          lenthen_Left_bondarise(Left_Up_Find);                       //
        //   //ips200_show_string(0,300,"cross2");
      }
      else if(Left_Down_Find != 0 && Right_Down_Find == 0) {
          // ���3�������µ���Ч �� ��߽����+�ұ߽��ӳ�
          lenthen_Right_bondarise(Right_Up_Find);
          add_Lline_k(leftline[Left_Down_Find], Left_Down_Find, 
                     Left_Up_Find, leftline[Left_Up_Find]);
        //    printf("cross3");
            //ips200_show_string(0,300,"cross3");
      }
      else {
          // ���4������Ч�µ� �� ˫�߽��ӳ�
          lenthen_Right_bondarise(Right_Up_Find);
          lenthen_Left_bondarise(Left_Up_Find);
        //    printf("cross4");
        ////ips200_show_string(0,300,"cross4");
      }

      // �쳣����ͻ���ʧЧʱ�ָ�ԭʼ�߽�
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


    if(Left_Up_Find&&Right_Up_Find)                                     //������ϵ�����ϵ㶼��Ч
    {
        carstatus_now = crossroad;                                      // ����ʮ��·��״̬
        return;
    }
    if(Left_Up_Find!=0&&Left_Down_Find!=0)                              //����ҵ������ϵ�����µ�
    {
        add_Lline_k(leftline[Left_Down_Find], Left_Down_Find,   
        Left_Up_Find, leftline[Left_Up_Find]);           
        centerline2_change();
    }
    if(Left_Up_Find!=0&&Left_Down_Find==0)                              //���ֻ�ҵ������ϵ�
    {
        lenthen_Left_bondarise(Left_Up_Find);                           //�ӳ���߽�
        centerline2_change();

    }
    if(Left_Up_Find==0)
    {
        carstatus_now = straight;                                      // ������ϵ�δ�ҵ�������ֱ��״̬
    }
    
  }
  if(carstatus_now == crossroadR)  {
        //ips200_show_string(0,300,"crossR");


    if(Left_Up_Find&&Right_Up_Find)                                     //������ϵ�����ϵ㶼��Ч
    {
        carstatus_now = crossroad;                                      // ����ʮ��·��״̬
        return;
    }
    if(Right_Up_Find!=0&&Right_Down_Find!=0)                            //����ҵ������ϵ�����µ�
    {
        add_Rline_k(rightline[Right_Down_Find], Right_Down_Find, 
        Right_Up_Find, rightline[Right_Up_Find]);
        centerline2_change();
    }
    if(Right_Up_Find!=0&&Right_Down_Find==0)                            //���ֻ�ҵ������ϵ�
    {
        lenthen_Right_bondarise(Right_Up_Find);                         //�ӳ��ұ߽�
        centerline2_change();

    }
    if(Right_Up_Find==0)
    {
        carstatus_now = straight;                                      // ������ϵ�δ�ҵ�������ֱ��״̬
    }
  }

   /*---------- Բ��Ԥʶ��״̬���� ----------*/
   if(carstatus_now == round_1)
    {
        Right_Up_Find=Find_Right_Up_Point(MT9V03X_H-1, search_stop); // �������Ϲյ�,ʹ�ø��µĺ���,
        Right_Down_Find=Find_Right_Down_Point(MT9V03X_H-2, search_stop); // �������¹յ�,ʹ�ø��µĺ���
        //ʹ�ø��µĸ���ȷ�ĺ��������¹յ�

        //ips200_show_string(0,300,"round1");
        if(Right_Up_Find>10&&Right_Up_Find<35&&Right_Down_Find==0)//���µ�û�ҵ�
        {
            lastrightupfind=Right_Up_Find; //��¼�� �����ϵ�
            BUZZ_START();
            carstatus_now = round_2;
            return; 
        }
        else    
        {   
            trace_right_bu(1,MT9V03X_H-1); //�ҵ����²�����
            centerline2_change();

        }
          if(Right_Down_Find>35&&right_budandiao)
        { 
            trace_right_bu(1,MT9V03X_H-1 ); //�ҵ����²�����
            //ע:����Ϊ�˿��ǵ�������ֱ������ô����
            centerline2_change();
        } 

   }
   if(carstatus_now == round_2)
   { 
        //ips200_show_string(0,300,"round2");
        Right_Up_Find   =Find_Right_Up_Point(MT9V03X_H-1, search_stop); // �������Ϲյ�,ʹ�ø��µĺ���,
        Right_Down_Find =Find_Right_Down_Point(MT9V03X_H-2, search_stop); // �������¹յ�,ʹ�ø��µĺ���
        if(Right_Up_Find)//�ҵ��ϵ�
        {  
            search_stop1=Right_Up_Find;
            add_Lline_k(rightline[Right_Up_Find]+10,Right_Up_Find,Right_Up_Find+30,leftline[Right_Up_Find+30]+10); //���ϵ㲹ֱ��
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
        Left_Down_Find=Find_Left_Down_Point(MT9V03X_H-1, search_stop-3); // �������¹յ�,ʹ�ø��µĺ���,
        Left_Up_Find=Find_Left_Up_Point(MT9V03X_H-2, search_stop); // �������Ϲյ�,ʹ�ø��µĺ���,
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
        left_budandiao=montonicity_left(MT9V03X_H-1,search_stop+6); // �󲻵��������ټ�6
        if(left_budandiao)
        {
             BUZZ_START();
            carstatus_now=round_5; // ������յ㲹б��״̬
            return;

        }

    }
    if (carstatus_now==round_5) 
    {
        //ips200_show_string(0,300,"round5");
        left_budandiao=montonicity_left(MT9V03X_H-1,search_stop+6); // �󲻵��������ټ�6
        // Right_Up_Find=Find_Right_Up_Point(MT9V03X_H-1, search_stop); // �������Ϲյ�,ʹ�ø��µĺ���
        // lenthen_Left_bondarise_bottom(left_budandiao); // �ӳ���߽絽�ײ�
        draw_Lline_k(leftline[left_budandiao],left_budandiao,search_stop,-1); // �󲻵����㲹ֱ��
        draw_Rline_k(MT9V03X_W-1,search_stop,rightlostpoint[1],0); // �Ҳ������㲹ֱ��
        if(left_budandiao==0) // ����󲻵�����Ϊ0
        {
            carstatus_now=round_6; // ���������ֱ��״̬
            BUZZ_START();
        }
        centerline2_change();
    }
    if(carstatus_now==round_6)
    {
        //ips200_show_string(0,300,"round6");

        Find_Right_Up_Point(MT9V03X_H-1, search_stop); // �������Ϲյ�,ʹ�ø��µĺ���,
        draw_Lline_k(0,MT9V03X_H-1,search_stop,-2);// ��߽粹ֱ��

        centerline2_change(); 

        if(Right_Up_Find||search_stop<20)
        {
            BUZZ_START();
            carstatus_now=round_7; // ����ֱ��״̬
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

    
   
    // //ips200_show_int(200,260,search_stop1,3); // ��ʾ������ֹ��1
    // //ips200_show_int(50,280,search_stop,3);      // ��ʾ��ֹ��
    // //ips200_show_string(80,220,"l_con");
    // //ips200_show_int(120,220,continuity_pointLeft[0],3); // ��ʾ��������
    // //ips200_show_string(150,220,"r_con");
    // //ips200_show_int(200,220,continuity_pointRight[0],3); // ��ʾ�Ҳ�������
    // //ips200_show_string(0,220,"r_bdd");         //�Ҳ�������
    // //ips200_show_int(50,220,right_budandiao,3); // ��ʾ�Ҳ�������
    // //ips200_show_string(0,220,"r_bdd");         //�Ҳ�������
    // //ips200_show_string(80,240,"l_bdd");         //��ֹ��
    // //ips200_show_int(120,240,left_budandiao,3); // ��ʾv��
    // //ips200_show_string(0,280,"s_stop");         //��ֹ��
    // //ips200_show_string(80,280,"l_up");          //���Ϲյ�
    // //ips200_show_int(120,280,Left_Up_Find,3);    
    // //ips200_show_string(70,300,"r_up");          //���Ϲյ�
    // //ips200_show_int(120,300,Right_Up_Find,3);
    // //ips200_show_string(160,260,"sto1");       //������ֹ��1
    // //ips200_show_string(150,280,"L_down");       //���¹յ�
    // //ips200_show_int(200,280,Left_Down_Find,3);
    // //ips200_show_string(150,300,"R_down");       //���¹յ�
    // //ips200_show_int(200,300,Right_Down_Find,3);
    // //ips200_show_string(0,260,"L_lost");       //���ߵ�
    // //ips200_show_int(50,260,leftlostpoint[0],3); // ��ʾ���ߵ�
    // //ips200_show_string(80,260,"R_lost");       //�Ҷ�
    // //ips200_show_int(130,260,rightlostpoint[0],3); // ��ʾ�Ҷ��ߵ�

    
}



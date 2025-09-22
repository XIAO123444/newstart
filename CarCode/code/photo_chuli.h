#ifndef PHOTO_CHULI_H__
#define PHOTO_CHULI_H__

#include "zf_common_headfile.h"
void difsum_left(uint8 y,uint8 x);
void difsum_right(uint8 y,uint8 x);
void image_boundary_process(void);

void set_b_imagine(int threshold);
int my_adapt_threshold(uint8 *image, uint16 col, uint16 row);
void image_boundary_process2(void);

bool stop_flag(void);


int16 continuity_left(uint8 start,uint8 end);
int16 continuity_right(uint8 start,uint8 end);
int16 montonicity_right(uint8 start,uint8 end);
int16 montonicity_left(uint8 start,uint8 end);

//Ê®×Ö
void Find_Down_Point(int16 start,int16 end);
void Find_Up_Point(int16 start,int16 end);

int16 Find_Left_Up_Point(int16 start,int16 end);
int16 Find_Right_Up_Point(int16 start,int16 end);
int16 Find_Left_Down_Point(int16 start,int16 end);
int16 Find_Right_Down_Point(int16 start,int16 end);

void draw_Lline_k(int16 startx, int16 starty, int16 endy, float dx);
void draw_Rline_k(int16 startx, int16 starty, int16 endy, float dx);

void add_Rline_k(int16 startx, int16 starty, int16 endy,int16 endx);
void add_Lline_k(int16 startx, int16 starty, int16 endy,int16 endx);

void lenthen_Left_bondarise(int16 start);
void lenthen_Right_bondarise(int16 start);

void lenthen_Right_bondarise_bottom(int16 start);
void lenthen_Left_bondarise_bottom(int16 start);


void banmaxian_check(void);
void black_protect_check(void);

int16 output_middle(void);
int16   find_vpoint(int16 start,int16 end);


void trace_right_bu(int16 start,int16 end);
void trace_left_bu(int16 start,int16 end);
void image_process(void);
void image_process2(void);

void photo_image_process_all(void);//????????
void photo_displayimage(void);      //????

void protect();
#endif

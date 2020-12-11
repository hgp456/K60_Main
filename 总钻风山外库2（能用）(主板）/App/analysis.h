#ifndef ANALYSIS_H_
#define ANALYSIS_H_

#include "constant.h"
#include "common.h"
#include "include.h"
#include "LCD_menu.h"
#include "math.h"
#include "analysis.h"
#include "PID.h"
#include "motor.h"
#include "init.h"
#include "interrupt.h"

#define UP      PTE10
#define DOWN    PTE3
#define RIGHT   PTE4
#define LEFT    PTE2
#define MIDDLE  PTD15

#define up      PTE10_IN
#define down    PTE3_IN
#define right   PTE4_IN
#define left    PTE2_IN
#define middle  PTD15_IN


#define NOPHOTO_EDGE  4   //无图像帧数大于此数转圈

extern uint8 image_threshold;
extern uint8 fakeflag;
extern int light;
extern uint16 last_image_size;
extern uint8 row_cut;

#define TURN_ROW_EDGE    55//45
#define LINE_ROW_EDGE    55
#define TWICE_ROW_EDGE   70
#define LONG_DISTANCE_EDGE 35//远近距离分界

extern uint16 count;

 typedef struct
{
    uint16  val;            
    uint8  row;                                
    uint8  col;         
    
} Point;            //点信息

extern int32 max_length;
extern int zhihui_flag;  

extern uint16 col_longest_num;
extern int last_col_longest;



//图像采集变量
extern uint8 Pix_Data[ROW][COL];
extern uint8 Pix_Data_BW[ROW][COL];
extern uint8 Pix_Data_last[ROW][COL];
extern int Is_SendPhoto;
extern int V_Cnt;
extern int hangcount;
extern uint8 Sample_Flag;

//图像处理变量
extern uint16 col_longest_min;     
extern uint16 col_longest_max;
extern uint16 col_longest_num;
extern uint8 NT;            //屏幕无白点标志

extern uint16 image_size;
extern uint16 col_position;
extern float row_position;
extern int col_longest;

extern char circleflag;
extern char motorflag;//0 减速   1加速
extern char pre_motorflag;
extern char servo_mode;

extern Point Brightest_Point[6] ;


extern uint32 circle_count;   //一个转弯过程有多少帧
extern uint32 line_count;     //一个直线过程有多少帧
extern uint32 line_count_count;  //连续多少个直线过程帧数都少于某一值（用于判断双闪或远灯）
extern uint8  slow_flag;

 //灯顺序处理                    1   2   3   4   5   6   7   8   9  10
extern uint8 SD_set[13];
extern uint8 road_num;
extern uint8 road_num_change_flag; 

extern uint8 countflag_while_circle;
extern uint8 speedflag_while_circle;


extern void motor_circle_judgement();
extern void Photo2Servo();
extern void image_test();
extern void Photoanly();
extern void getphoto();
extern void LocatingEye_OriginalImage_FixedThreshold_OledPrint();

extern uint16  uzi;//看灯的最大行
extern uint16 last_col_position;
 //超声波
extern uint8 udflag;
extern uint8 udcflag;
extern uint8 gpioudflag;
//一闪一闪
extern uint8 fakeflag;
//test
extern uint8 test_col_flag;
extern void  new_photo_deal_1(BYTE *pImgData, int width, int height);

extern int cnt;
extern	int max,sec;
extern	int lmax;
extern	int lsec;
extern	int label[20000];    //索引数组，plabel直接指向的标记索引号区间
extern	int labelcnt[20000]; //标注数组，处理过标记冲突的索引号，其数组最终大小即为连通图个数
extern	int lcnt[20000];	 //计数数组，统计每个连通图的大小
extern	int allow[11];           //运行输出最大连通域个数
   
extern	int *plabel;
extern	int num,numt;
extern	int l;
extern	int *pimage[ROW][COL];
extern	int imageout[ROW][COL];


extern   int last1_col_position;
extern   int last2_col_position;
extern  int last3_col_position;
extern  int last4_col_position;
extern  int last5_col_position;
extern  int last6_col_position;
extern void barrier_cam();
#endif 
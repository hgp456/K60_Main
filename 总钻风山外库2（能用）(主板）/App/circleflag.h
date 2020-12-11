#ifndef CIRCLEFLAG_H_
#define CIRCLEFLAG_H_

extern void mode_choose();
extern int Circle_Right_Duty[4]; //向右转
extern int Circle_Left_Duty[4]; //向左转
extern int Line_Left_Duty[4]; // 直走靠左走
extern int Line_Right_Duty[4]; //直走靠右走
extern int Left_Sidesway[4]; // 向左横移
extern int Right_Sidesway[4]; //向右横移
extern int Line_Back_Duty[4];//倒车
extern int modeflag;
extern uint8 start_col_flag;
extern float row_position_store[4];
#define COL_LEFT 90
#define COL_MID 160
#define COL_RIGHT 98
extern uint8 COL_SET;
extern float row_position_judge;  
extern uint16 col_position_judge;
extern int last_col_longest_judge;
extern uint8 NT_judge;
extern uint8 fakeflag_judge;
extern int barrier_flag;
extern uint8 stopflag;
extern uint8 col_start_flag;
extern int line_flag;
extern uint8 testflag;
extern int baiweiflag;
extern void shuangshe_use();
extern uint8 free_flag;
extern uint8 control_mode;
#endif

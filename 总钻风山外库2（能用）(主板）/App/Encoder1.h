#ifndef _ENCODER1_H_
#define _ENCODER1_H_

#include "include.h"
#include "common.h" 

extern void count_get_test();
extern void count_get();

extern int16 count_buff[2];
extern int16 count_LB1;


extern int16 count_A;
extern int16 count_B;
extern int16 count_C;
extern int16 count_D;

extern int16 count_LF;
extern int16 count_LB;
extern int16 count_RF;
extern int16 count_RB;

extern uint8 B_row_position;
extern uint8 B_col_position;
extern uint8 B_fake_flag;
extern uint8 B_NT;
extern uint8 B_last_col_longest;

extern int16 Encoder[5];
extern void receive_count();
extern int16 speed_check;
extern uint8 qie_left_flag;
extern int barrier_dis_mid;
extern uint8 barrier_congban_flag;
#endif
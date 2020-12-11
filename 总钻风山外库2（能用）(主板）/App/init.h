#ifndef _INIT_H_
#define _INIT_H_

extern void init_all();
extern void Motor_init();
extern void Enconder_init();  

extern uint8 VAR[3];//存于RAM中的数组  
extern uint8 threshold;
extern uint8 singlecarflag ;
extern uint8 sd_right_sp;
extern uint8 sd_right_long_sp;
extern uint16 CHASU_HIGHDUTY  ;
extern uint16 CHASU_LOWDUTY ;
extern uint16 HIGH_SPEED;
extern uint8 row_edge;
extern uint8 openmv_finish_flag;
extern uint8 speed_array[12];  
extern float row_edge_array[12];
extern int16  chasu_high_array[11];
extern int16  chasu_low_array[11];
extern int16  chasu_highduty_array[11];
extern int16  chasu_lowduty_array[11];
extern uint8 duzhuanmode_flag;
extern int speed_flag;
extern float row_delta[12];
extern uint8 zhuanwan_flag;
extern uint8 first_flag;
extern uint8 countflag_while_ud;
extern uint8 first_judge1;
extern uint8 first_judge2;
  extern float Error2nrf;
    extern float new_row_edge[16];
extern  uint8 col_mid_back;
#endif
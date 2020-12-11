/*

 Send.h
 

*/
#include "common.h"


#ifndef Send_H_
#define Send_H_

#define BYTE0(dwTemp)           (*(char *)(&dwTemp))
#define BYTE1(dwTemp)           (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)           (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)           (*((char *)(&dwTemp) + 3))

extern void SendBox(int16 admin[3]);
extern void recieve_check();
extern void nrf_send();
extern uint32 use_time;
extern int16 BOX[9];    //匿名科创发送数组
extern uint8 buff[DATA_PACKET]; 
extern uint8 receive_flag;
extern uint16 row_min;
extern uint16 row_max;
extern int last_col_longest;
extern void send_lindar_start();
extern void receive_lindar();
extern void tof_anas();
#endif
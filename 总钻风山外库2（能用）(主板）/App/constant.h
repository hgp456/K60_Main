/*
 constant.h
 Created on 2016.3.12
 By Yuxin Zheng
*/


#ifndef CONSTANT_H_
#define CONSTANT_H_

/**************
新加变量
***************/

#define Wheel_A 'A'
#define Wheel_B 'B'
#define Wheel_C 'C'
#define Wheel_D 'D'

#define FORWARD_SPEED 15
#define BACK_SPEED 0
#define CIRCLE_SPEED_A 15
#define CIRCLE_SPEED_B 15
#define CIRCLE_SPEED_C 15
#define CIRCLE_SPEED_D 15
#define CIRCLE_BACK_SPEED_A 45
#define CIRCLE_BACK_SPEED_B 45
#define CIRCLE_BACK_SPEED_C 45
#define CIRCLE_BACK_SPEED_D 45
#define BYTE unsigned char





//////////////////////////////////////////////////////////////
//读取另一块的信号
#define ZHUANWAN_FLAG     PTD11  //CONNECT2
#define UDSTART           PTD12   //连接GETUD  CONNECT1
//给另一块输出
#define SPEED_OUTPUT0     PTD14  //CONNECT3
#define SPEED_OUTPUT1     PTD13    //CONNECT4

#define CONNECT5   PTE12    //CONNECT5
//#define UARTTX     PTE8    
//#define UARTRX     PTE9

#define SECTOR_NUM  (FLASH_SECTOR_NUM-1)     
//////   超声波
////#define TRIG     PTC14
////#define ECHG     PTA12     //FTM1_CH0

//写RAM的地址


#define MOTOR_HZ  10*1000
#define MOTOR_FTM   FTM0
#define NEW_MOTOR_FTM FTM3

#define MOTOR1_PWM    FTM_CH3     //左
#define MOTOR2_PWM  FTM_CH4
#define MOTOR3_PWM  FTM_CH6       //右
#define MOTOR4_PWM  FTM_CH7        

#define NEW_MOTOR1_PWM   FTM_CH0
#define NEW_MOTOR2_PWM    FTM_CH2




#define HIGH_DUTY 0
#define LOW_DUTY  1000


#define CHASU_SLOW_HIGHDUTY 750
#define CHASU_SLOW_LOWDUTY  950


//倒车参数
#define BACK_DUTY 800
#define BACK_TIME 200
 
#define DAOCHE_CIRCLE_HIGHDUTY 400
#define DAOCHE_CIRCLE_LOWDUTY  650
//      电机pid参数
#define HIGH_SPEED1 25//稳
#define HIGH_SPEED2 45
#define HIGH_SPEED3 65
#define HIGH_SPEED4 85

#define RE2 50.0
#define RE4 50.0
#define RE6 50.0
#define RE8 50.0

//      舵机pwm参数   

#define S3010_FTM   FTM3
#define S3010_CH    FTM_CH0
#define S3010_HZ    (100)
#define S3010_MAX   162
#define S3010_MIN   130
#define S3010_MID   145
//     舵机PID参数
#define SD_LEFT_SP               92//123   //108
#define SD_RIGHT_SP              73//62//50
#define SD_RIGHT_LONG_SP         77

#define SD_KP                          0.15//0.18   //0.25//0.32*// 0.4  //1.1
#define SD_KD                          0.065//0.055//0.135//0.17//0.15  //1   //0.25  //0.1
#define SD_LASTERROR                   0
#define SD_PREERROR                    0

//     摄像头参数 

#define SIZE    (ROW * COL)  //图像占用空间大小


#endif 
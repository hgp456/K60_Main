#ifndef _PID_H_
#define _PID_H_

#include "motor.h"
#include "common.h"
#define minimum 0
#define maximum 1000
extern int16 Aim_Speed[5];
extern int16 Encoder[5];
extern uint8 Kp,Ki,Kd;
extern uint8 KP,KD;
extern uint8 DKPN ,DKDN;
extern uint8 pianliflag;
extern uint8 gaosu_flag;
typedef struct
{
   float KP;
    float KD;
    float mid_err;
    float dir_out;
    float last_err;
}DIR_PID;

typedef struct
{
	double Kp;
	double Ki;
	double Kd;

	double err;
	double ierr;
	double derr;

	double err_last;
	
	int Pout;
	int Iout;
	int Dout;

	double Iout_MAX;

	int OUT;

	double  c1 ;  //Œ¢∑÷œ»––
	double  c2 ;
	double  c3 ;
	double  temp ;
        
        double err_last_1;
        double err_last_2;


}PID;
        
extern void Set_Motor(float PID_Motor_0, float PID_Motor_1, float PID_Motor_2, float PID_Motor_3);
extern int Get_MxMi(float num, float max, float min);
extern void PID_Init(void);
extern void motor_circle_judgement();
extern void pid();
extern void Receive_PID();
extern void pid_set();
extern double Vx1;
extern double Vy1;
extern float Vz1;
extern int Dir_Speed;
extern PID PID_Motor[5];        
extern void dir_pid();
extern DIR_PID Dir_pid;
extern uint8 col_mid1;
extern int col_test;
extern uint8 turn_left_flag;
extern uint8 turn_right_flag;
extern uint8 bupian_count;
#endif

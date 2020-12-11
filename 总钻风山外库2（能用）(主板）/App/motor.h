#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "include.h"
#include "common.h"
extern void Motor_Init(void);
extern void Set_Motor(float MotorA, float MotorB, float MotorC, float MotorD);

extern void Motor_Move_Api(int mode,float pwm);

extern void Move_Forward(void);  //向前运动
extern void Move_Left(void);     //向左运动
extern void Move_Back(void);     //向后运动
extern void Move_Right(void);    //向右运动

extern void Turn_Left(void);      //饶yaw轴左旋转
extern void Turn_Right(void);     //饶yaw轴右旋转

extern void Stop(void);

extern void MotorA_FWD(void);     //正转，逆时针转动
extern void MotorA_REV(void);     //反转，顺时针转动
extern void MotorA_STOP(void);    //刹停，采用两路IN都为高电平输入来刹车
extern void MotorA_UNSTOP(void);  //解除刹车状态，运动状态仍与刹车前一致


extern void MotorB_FWD(void);
extern void MotorB_REV(void);
extern void MotorB_STOP(void);
extern void MotorB_UNSTOP(void);

extern void MotorC_FWD(void);
extern void MotorC_REV(void);
extern void MotorC_STOP(void);
extern void MotorC_UNSTOP(void);

extern void MotorD_FWD(void);
extern void MotorD_REV(void);
extern void MotorD_STOP(void);
extern void MotorD_UNSTOP(void);
extern void Motor_test();

extern void  Motor_FWD(int high,int low,char wheel);
extern void Motor_FORWARD();
extern void Motor_Circle_Right();
extern void Motor_Circle_Left();

extern void  McNamm_wheel_control();
extern void McNamm_wheel_control_test(float Vx1,float Vy1,float Vz1);
extern float Vx;
extern float Vy;
extern float Vz;

extern float A_Aim_speed;
extern float B_Aim_speed;
extern float C_Aim_speed;
extern float D_Aim_speed;
extern int Vx_qie;

extern const int Forward;
extern const int Back;
extern const int Left;
extern const int Right;
extern const int Move_left;
extern const int Move_right;
#define a_PARAMETER          (0.09f)               
#define b_PARAMETER          (0.09f) 
#endif

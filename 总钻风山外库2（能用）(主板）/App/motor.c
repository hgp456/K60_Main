	#include  "common.h"
#include  "stdio.h"
#include "MOTOR.h"
#include "constant.h"
#include "include.h"
#include "PID.h"
#include "circleflag.h"
#include "interrupt.h"
#include "Encoder1.h"
//定义两轮PWM值；
int MotorA1;
int MotorA2;
int MotorB1;
int MotorB2;
int MotorC1;
int MotorC2;
int MotorD1;
int MotorD2;
float Vx = 0;
float Vy = 0;
float Vz = 0;
int Vx_qie=0;
float A_Aim_speed;
float B_Aim_speed;
float C_Aim_speed;
float D_Aim_speed;
//void zhuanwan_flag = 0;
/*#define a_PARAMETER          (0.09f)               
#define b_PARAMETER          (0.09f) */
//void zhuanwan_flag = 0;



/*电机初始化放在整体初始化中*/


uint8 MotorA_Status = 0;
uint8 MotorB_Status = 0;
uint8 MotorC_Status = 0;
uint8 MotorD_Status = 0;



/*
 *麦轮运动控制API
 2020/12/11
  功能：完成麦轮全向移动的几种模式
   变量mode：选择相应的模式 比如直走 后退 左转 右转 横移左 横移右  可以在motor.h中找到这些变量
   变量pwm,选择速度 选择参数为0-10000 
   举例 Motor_Move_Api(forward,5000);
  */

//
const int Forward    =  0;
const int Back       = 1;
const int Left       = 2;
const int Right      = 3;
const int Move_left  = 4;
const int Move_right = 5;
void Motor_Move_Api(int mode,float pwm)
{
  if(mode==Forward)
  {
    Set_Motor(pwm,pwm,pwm,pwm);
  }
  else if(mode == Back)
  {
     Set_Motor(-pwm,-pwm,-pwm,-pwm);
  }
  else if(mode == Left)
  {
     Set_Motor(-pwm,pwm,pwm,-pwm);
  }
  else if(mode == Right)
  {
     Set_Motor(pwm,-pwm,-pwm,pwm);
  }
  else if(mode ==Move_left)
  {
     Set_Motor(-pwm,pwm,-pwm,pwm);
  }
  else if(mode ==Move_right)
  {
     Set_Motor(pwm,-pwm,pwm,-pwm);
  }
  else
  {
    //参数不对，检查大小写
  }
}








/********************
 电机:wheel
high>low 正 
low>high 反 
郭世琛
2019/3/9
*******************/


void Motor_test()
{
  //Motor_Init();
  while(1)
  {
  MotorA_FWD();
//  DELAY_MS(500);
//  MotorA_STOP();
//  DELAY_MS(500);
  MotorB_FWD();
//  DELAY_MS(500);
 // MotorB_STOP();
//  DELAY_MS(500);
  MotorC_FWD();
  //DELAY_MS(500);
 // MotorC_STOP();
 // DELAY_MS(500);
  MotorD_FWD();
 // DELAY_MS(500);
  //MotorD_STOP();
 // DELAY_MS(500);
  }
}
/*麦轮控制*/
void McNamm_wheel_control_test(float Vx1,float Vy1,float Vz1)
{
   // A_Aim_speed   = 3.0*(+Vx1+Vy1-Vz1*(a_PARAMETER+b_PARAMETER));
    //B_Aim_speed   = 3.0*(-Vx1+Vy1+Vz1*(a_PARAMETER+b_PARAMETER));
   // C_Aim_speed   = 3.0*(-Vx1+Vy1-Vz1*(a_PARAMETER+b_PARAMETER));
    //D_Aim_speed   = 3.0*(+Vx1+Vy1+Vz1*(a_PARAMETER+b_PARAMETER));
    Aim_Speed[0]=0;
    Aim_Speed[1]=B_Aim_speed;
    Aim_Speed[2]=0;
    Aim_Speed[3]=0;
   // pid();
}
void McNamm_wheel_control()
{
  
    if(modeflag  ==0 || modeflag ==3 )
    {
     Vz = (float)Dir_pid.dir_out ;
    }
    // Vx = (float)(Vx_turn + Vx_qie);


    A_Aim_speed   = 3.0*(+Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER));
    B_Aim_speed   = 3.0*(-Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER));
    C_Aim_speed   = 3.0*(-Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER));
    D_Aim_speed   = 3.0*(+Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER));
    
   if(control_mode == 0)
   {
    Aim_Speed[0]=A_Aim_speed;
    Aim_Speed[1]=B_Aim_speed;
    Aim_Speed[2]=C_Aim_speed;
    Aim_Speed[3]=D_Aim_speed;
    
    Encoder[0] = count_A;  //赋值
    Encoder[1] = count_B;
    Encoder[2] = count_C;
    Encoder[3] = count_D;
   }
   else
   {
        Encoder[0] = count_A;  //赋值
        Encoder[1] = count_B;
        Encoder[2] = count_C;
        Encoder[3] = count_D;
       Aim_Speed[0]  =-D_Aim_speed;  //A轮   给C轮的速度并取反
       Aim_Speed[1]  =-C_Aim_speed;  //B轮  给D轮的速度并取反
       Aim_Speed[2]  =-B_Aim_speed;  //C轮   给A轮的速度并取反
       Aim_Speed[3]  =-D_Aim_speed;  //D轮   给B轮的速度并取反
   }

  
   pid(); 
}

void Motor_FWD(int x,int y,char wheel)
{
    int high;
    int low;
    
    high=100-x;
    low =100-y;
	switch(wheel)
	{
		case 'A':
                        
                        ftm_pwm_duty(FTM0, MOTOR3_PWM, high);  //A
			ftm_pwm_duty(FTM0, MOTOR4_PWM, low);
		case 'B':

                        
                        ftm_pwm_duty(FTM3, NEW_MOTOR1_PWM, high);   //B
	                ftm_pwm_duty(FTM3, NEW_MOTOR2_PWM, low);
		case 'C':
                        
                        ftm_pwm_duty(FTM0, MOTOR2_PWM, high);
                        ftm_pwm_duty(FTM0, MOTOR1_PWM, low);
	          
		case 'D':
                        ftm_pwm_duty(FTM3, MOTOR4_PWM, high);
                        ftm_pwm_duty(FTM3, MOTOR3_PWM, low); //D
	     
	}
}


void Motor_FORWARD() //直走
     {
       Motor_FWD(FORWARD_SPEED,BACK_SPEED,Wheel_A); //
       Motor_FWD(FORWARD_SPEED,BACK_SPEED,Wheel_B);
       Motor_FWD(FORWARD_SPEED,BACK_SPEED,Wheel_C);
       Motor_FWD(FORWARD_SPEED,BACK_SPEED,Wheel_D);
     }

void Motor_Circle_Right()   //向右转
{
       Motor_FWD(CIRCLE_SPEED_A,BACK_SPEED,Wheel_A); //
       Motor_FWD(CIRCLE_SPEED_B,CIRCLE_BACK_SPEED_B,Wheel_B);
       Motor_FWD(CIRCLE_SPEED_C,BACK_SPEED,Wheel_C);
       Motor_FWD(CIRCLE_SPEED_D,BACK_SPEED,Wheel_D);
}

void Motor_Circle_Left()   //向右转
{
       Motor_FWD(CIRCLE_SPEED_A,BACK_SPEED,Wheel_A); //
       Motor_FWD(CIRCLE_SPEED_B,BACK_SPEED,Wheel_B);
       Motor_FWD(CIRCLE_SPEED_C,CIRCLE_BACK_SPEED_C,Wheel_C);
       Motor_FWD(CIRCLE_SPEED_D,BACK_SPEED,Wheel_D);
}
/****************************
            6个基础动作（跳舞动作）
*****************************/
void Move_Forward(void)
{
	MotorA_FWD();
	MotorB_FWD();
	MotorC_FWD();
	MotorD_FWD();
}

void Move_Left(void)
{
	MotorA_REV();
	MotorB_FWD();
	MotorC_FWD();
	MotorD_REV();
}

void Move_Back(void)
{
	MotorA_REV();
	MotorB_REV();
	MotorC_REV();
	MotorD_REV();
}

void Move_Right(void)
{
	MotorA_FWD();
	MotorB_REV();
	MotorC_REV();
	MotorD_FWD();
}

void Turn_Left(void)
{
	MotorA_REV();
	MotorB_FWD();
	MotorC_REV();
	MotorD_FWD();
}

void Turn_Right(void)
{
	MotorA_FWD();
	MotorB_REV();
	MotorC_FWD();
	MotorD_FWD();
}

void Stop(void)
{
	MotorA_STOP();
	MotorB_STOP();
	MotorC_STOP();
	MotorD_STOP();
}

/*补充每个运动形态 再之后移植时可以将K66状态下应有的写入*/
//每一个都是一个置零 之后调整
void MotorC_REV(void) //正转 //A轮的AB与其他轮相反 需要注意
{
	MotorC1 = 8000;
	MotorC2 = 10000;
	ftm_pwm_duty(FTM0, MOTOR1_PWM, MotorC1);
	ftm_pwm_duty(FTM0, MOTOR2_PWM, MotorC2);
	MotorC_Status = 0;
}

void MotorC_FWD(void)  //反转
{
	MotorC1 = 10000;
	MotorC2 = 8000;
	ftm_pwm_duty(FTM0, MOTOR1_PWM, MotorC1);
	ftm_pwm_duty(FTM0, MOTOR2_PWM, MotorC2);

	MotorC_Status = 1;

}

void MotorC_STOP(void)
{
	MotorC1 = 10000;
	MotorC2 = 10000;
	ftm_pwm_duty(FTM0, MOTOR1_PWM, MotorC2);
	ftm_pwm_duty(FTM0, MOTOR2_PWM, MotorC1);
}

void MotorC_UNSTOP(void)  //复原
{
	if (MotorC_Status == 0)
	{
		MotorC_FWD();
	}

	else if (MotorC_Status == 1)
	{
		MotorC_REV();
	}
}

void MotorA_FWD(void)
{
	MotorA1 = 8000;
	MotorA2 = 10000;
	ftm_pwm_duty(FTM0,MOTOR3_PWM, MotorA1); 
	ftm_pwm_duty(FTM0, MOTOR4_PWM, MotorA2);

	MotorA_Status = 0;
}

void MotorA_REV(void)
{
	MotorA1 = 10000;
	MotorA2 = 8000;
	ftm_pwm_duty(FTM0, MOTOR3_PWM, MotorA1);
	ftm_pwm_duty(FTM0, MOTOR4_PWM, MotorA2);
	MotorA_Status = 1;

}

void MotorA_STOP(void)
{
	MotorA1 = 10000;
	MotorA2 = 10000;
	ftm_pwm_duty(FTM0, MOTOR3_PWM, MotorA1);
	ftm_pwm_duty(FTM0, MOTOR4_PWM, MotorA2);

}

void MotorA_UNSTOP(void)
{
	if (MotorA_Status == 0)
	{
		MotorA_FWD();
	}
	else if (MotorA_Status == 1)
	{
		MotorA_REV();
	}
}

void MotorB_FWD(void)
{
	MotorB1 = 8000;
	MotorB2 = 10000;
	ftm_pwm_duty(FTM3, NEW_MOTOR1_PWM, MotorB1);
	ftm_pwm_duty(FTM3, NEW_MOTOR2_PWM, MotorB2);

	MotorB_Status = 0;
}

void MotorB_REV(void)
{
	MotorB1 = 10000;
	MotorB2 = 8000;
	ftm_pwm_duty(FTM3, NEW_MOTOR1_PWM, MotorB1);
	ftm_pwm_duty(FTM3, NEW_MOTOR2_PWM, MotorB2);
	MotorB_Status =1;
}

void MotorB_STOP(void)
{
	MotorB1 = 10000;
	MotorB2 = 10000;
	ftm_pwm_duty(FTM3, NEW_MOTOR1_PWM, MotorB1);
	ftm_pwm_duty(FTM3, NEW_MOTOR2_PWM, MotorB2);
}

void MotorB_UNSTOP(void)
{
	if (MotorB_Status == 0)
	{
		MotorB_FWD();
	}

	else if (MotorB_Status == 1)
	{
		MotorB_REV();
	}
}

void MotorD_REV(void)
{
	MotorD1 = 8000;
	MotorD2 = 10000;
	ftm_pwm_duty(FTM3, MOTOR3_PWM, MotorD1); // NEW_MOTOR3_PWM
	ftm_pwm_duty(FTM3, MOTOR4_PWM, MotorD2);// NEW_MOTOR4_PWM
	MotorD_Status = 0;
}

void MotorD_FWD(void)
{
	MotorD1 = 10000;
	MotorD2 = 8000;
	ftm_pwm_duty(FTM3, MOTOR3_PWM, MotorD1); // NEW_MOTOR3_PWM
	ftm_pwm_duty(FTM3, MOTOR4_PWM, MotorD2); // NEW_MOTOR4_PWM

	MotorD_Status = 1;
}

void MotorD_STOP(void)
{
	MotorD1 = 10000;
	MotorD2 = 10000;
	ftm_pwm_duty(FTM3, MOTOR3_PWM, MotorD1); // NEW_MOTOR3_PWM
	ftm_pwm_duty(FTM3, MOTOR4_PWM, MotorD2); // NEW_MOTOR4_PWM
}

void MotorD_UNSTOP(void)
{
	if (MotorD_Status == 0)
	{
		MotorD_FWD();
	}
	else if (MotorD_Status == 1)
	{
		MotorD_REV();
	}
}

//转弯  三个轮转一个不转 尝试进行组合
/*void circle_motor(int zhuawan_flag)
{
	switch(zhuanwan_flag)
		case 0:
			MotorA_FWD();
			MotorB_FWD();
			MotorC_FWD();
			MotorD_FWD();
		case 1: 
			MotorA_STOP();
			MotorB_FWD();
			MotorC_FWD();
			MotorD_FWD();
		case 2：
			MotorA_FWD();
			MotorB_STOP();
			MotorC_FWD();
			MotorD_FWD();
		case 3:
			MotorA_FWD();
			MotorB_FWD();
			MotorC_STOP();
			MotorD_FWD();
		case 4：
			MotorA_FWD();
			MotorB_FWD();
			MotorC_FWD();
			MotorD_STOP();							
}*/
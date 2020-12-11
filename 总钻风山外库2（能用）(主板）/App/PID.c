#include "PID.h"
#include "constant.h"
#include "math.h"
#include "Encoder1.h"
#include "common.h"
#include "include.h"
#include "Send.h"
#include "circleflag.h"
#include "analysis.h"
double   PID_Threshold = 60; //积分分离
int16 Aim_Speed[5]={0};
int16 Last_Encoder[5];
int16 Encoder[5];
PID  PID_Motor[5];
int Dir_Speed;
unsigned char nmr = 0;
double  K = 0.8;
int  Ki_switch = 0;
double stepIn = (maximum - minimum)*0.1 + minimum;  //步进值 仍需计算 未调整
int kFactor = 0;//用于判断步进值的加减
double gama;  //微分先行 滤波系数
uint8 COL_SET = 0;
DIR_PID Dir_pid;
uint8 start_col_flag = 0;
  int DKI_SWITCH = 0;
/* 目前只用了增量式 */
  
/*typedef struct
{
	float Kp;
	float Ki;
	float Kd;

	float err;
	float ierr;
	float derr;

	float err_last;

	float Pout;
	float Iout;
	float Dout;

	float Iout_MAX;

	float OUT;

	float  Gama;  //微分先行
	float  c1;
	float  c2;
	float  c3;
	float  temp;


}PID;*/

void PID_Init()
{
	Dir_pid.KP = (float)KP;
	Dir_pid.KD = (float)KD;
	PID_Motor[0].Kp = (float)Kp;
	PID_Motor[0].Ki = (float)Ki;
	PID_Motor[0].Kd = (float)Kd;
  for (nmr = 0 ;nmr<4; nmr++)
  {
    PID_Motor[nmr].err = 0.0;
    PID_Motor[nmr].err_last_1 = 0.0;
    PID_Motor[nmr].err_last_2 = 0.0;
    
	PID_Motor[nmr].Kp = PID_Motor[0].Kp;
	PID_Motor[nmr].Ki = PID_Motor[0].Ki;
	PID_Motor[nmr].Kd = PID_Motor[0].Kd;
  }

   

       
   
}
  

void pid()
{
        pid_set();
	for (nmr = 0; nmr < 4; nmr++)
	{
		PID_Motor[nmr].err =Aim_Speed[nmr] - Encoder[nmr];  //


	/*	if ( fabs(PID_Motor[nmr].err) > PID_Threshold)   //积分分离  这个值需要实际确定
		{
			Ki_switch = 0;
		}
		else
		{
			Ki_switch = 1;
		}*/

		//PID_Motor[nmr].err = PID_Motor[nmr].err - PID_Motor[nmr].err_last_1;
		PID_Motor[nmr].ierr = PID_Motor[nmr].err; 
		PID_Motor[nmr].derr = PID_Motor[nmr].err - 2*PID_Motor[nmr].err_last_1 + PID_Motor[nmr].err_last_2;  //换上增量式 但是现在不知道往年程序中为什么除以7.5

		//PID_Motor[nmr].temp = gama * PID_Motor[nmr].Kd + PID_Motor[nmr].Kp;
		//PID_Motor[nmr].c3 = PID_Motor[nmr].Kd / PID_Motor[nmr].temp;
		//PID_Motor[nmr].c2 = (PID_Motor[nmr].Kd + PID_Motor[nmr].Kp) / PID_Motor[nmr].temp; //（kd+kp）/temp
		//PID_Motor[nmr].c1 = gama * PID_Motor[nmr].c3;
		
		PID_Motor[nmr].Pout = PID_Motor[nmr].Kp * (PID_Motor[nmr].err- PID_Motor[nmr].err_last_1);
		//输出I
		//PID_Motor[nmr].Iout = PID_Motor[nmr].Ki * PID_Motor[nmr].ierr*Ki_switch; //积分分离
               PID_Motor[nmr].Iout = PID_Motor[nmr].Ki * PID_Motor[nmr].ierr;
		//PID_Motor[nmr].Iout = Get_MxMi(PID_Motor[nmr].Iout, 200, -200);// //限波 这个值也需要改
		//输出D
		//PID_Motor[nmr].Dout = PID_Motor[nmr].c1 * PID_Motor[nmr].Dout + PID_Motor[nmr].c2 * Encoder[nmr] + PID_Motor[nmr].c3 * Last_Encoder[nmr]; //微分先行设定值
		PID_Motor[nmr].Dout = PID_Motor[nmr].Kd *PID_Motor[nmr].derr; //这个为原本的Dout 
		//更新误差
		PID_Motor[nmr].err_last_1 = PID_Motor[nmr].err;
		PID_Motor[nmr].err_last_2 = PID_Motor[nmr].err_last_1;
		//输出
	    
                 PID_Motor[nmr].OUT =Get_MxMi((((PID_Motor[nmr].Pout + PID_Motor[nmr].Iout + PID_Motor[nmr].Dout)/3.0)*100), 9000,-9000); //限波 这个值也需要改
               
		Last_Encoder[nmr] = Encoder[nmr];//微分先行
                         
	}
        //Set_Motor(A_OUT,B_OUT,C_OUT,D_OUT);
         Set_Motor(PID_Motor[0].OUT,PID_Motor[1].OUT,PID_Motor[2].OUT,PID_Motor[3].OUT);//更改PID参数值
}

void Set_Motor(float PID_Motor_0, float PID_Motor_1, float PID_Motor_2,float PID_Motor_3) //或者直接写在里面
{
  /*double OUPUT_0,OUPUT_1,OUPUT_2,OUPUT_3;
  OUPUT_0 = PID_Motor_0 -Dir_pid.dir_out;
  OUPUT_1 = PID_Motor_1 +Dir_pid.dir_out;
  OUPUT_2 = PID_Motor_2 -Dir_pid.dir_out;
  OUPUT_3 = PID_Motor_3 +Dir_pid.dir_out;*/
  if(PID_Motor_1>=0.0)
  {
    ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,(int)(10000.0-(PID_Motor_1))); 
    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,(int)10000.0);
  }
  else
  {
   ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,(int)10000.0); 
    ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,(int)(10000.0-(-PID_Motor_1)));
  }
  if(PID_Motor_2>=0.0)
  {
    ftm_pwm_duty(NEW_MOTOR_FTM, NEW_MOTOR1_PWM,(int)(10000.0-(PID_Motor_2))); 
    ftm_pwm_duty(NEW_MOTOR_FTM, NEW_MOTOR2_PWM,(int)10000.0);
  }
  else
  {
   ftm_pwm_duty(NEW_MOTOR_FTM, NEW_MOTOR1_PWM,(int)10000.0); 
    ftm_pwm_duty(NEW_MOTOR_FTM, NEW_MOTOR2_PWM,(int)(10000.0-(-PID_Motor_2)));
  }
    if(PID_Motor_3>=0.0)
  {
    ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,(int)10000.0); 
    ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,(int)(10000.0-(PID_Motor_3)));
  }
  else
  {
   ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,(int)(10000.0-(-PID_Motor_3))); 
    ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,(int)10000);
  }
     if(PID_Motor_0>=0.0)
  {
    ftm_pwm_duty(NEW_MOTOR_FTM, MOTOR3_PWM,(int)10000.0); 
    ftm_pwm_duty(NEW_MOTOR_FTM, MOTOR4_PWM,(int)(10000.0-(PID_Motor_0)));
  }
  else
  {
   ftm_pwm_duty(NEW_MOTOR_FTM, MOTOR3_PWM,(int)(10000.0-(-PID_Motor_0))); 
    ftm_pwm_duty(NEW_MOTOR_FTM, MOTOR4_PWM,(int)10000);
  }
   
  
}






int Get_MxMi(float num,float max,float min)
{
	if(num > max)
		return max;
	else if(num < min)
		return min;
	else
		return num;
}

void pid_set()  //一组参数
{
  
  
    PID_Motor[1].Kp =  PID_Motor[0].Kp;
    PID_Motor[2].Kp =  PID_Motor[0].Kp;
    PID_Motor[3].Kp =  PID_Motor[0].Kp;  //先用一组参数
    
    PID_Motor[1].Ki = PID_Motor[0].Ki;
    PID_Motor[2].Ki = PID_Motor[0].Ki;
    PID_Motor[3].Ki = PID_Motor[0].Ki;
    
    PID_Motor[1].Kd = PID_Motor[0].Kd;
    PID_Motor[2].Kd = PID_Motor[0].Kd;
    PID_Motor[3].Kd = PID_Motor[0].Kd;
    
    gama =  0; //微分先行 滤波系数
    PID_Threshold = 0;  //积分分离

    //McNamm_wheel_control(); 
      /*if(mode_change_flag ==1)
       {
         Aim_Speed_Change();
       }*/
   // motion_check();
    
    
}

//微分先行参考 已进入主程序
/*定义结构体和公用体*/
/*
typedef struct

{

	float setpoint;       //设定值

	float proportiongain;     //比例系数

	float integralgain;      //积分系数

	float derivativegain;    //微分系数

	float lasterror;     //前一拍偏差

	float result;     //输出值

	float integral;   //积分值

	float derivative;      //微分项

	float lastPv;     //前一拍的测量值

	float gama;      //微分先行滤波系数

}PID;

void PIDRegulation(PID *vPID, float processValue)

{

  float thisError;


    
  thisError=vPID->Aim_Speed[nmr]-Encoder[nmr];  //更改后的误差 我认为Process的value 为测量值
  temp= gama   * PID_Motor[nmr].Kd  + PID_Motor[nmr].Kp;

  c3= PID_Motor[nmr].Kd/temp; // kd /temp

  c2=(PID_Motor[nmr].Kd+ PID_Motor[nmr].Kp)/temp; //（kd+kp）/temp

  c1= gama*c3;//gama*kd/temp

  PID_Motor[nmr].Dout =c1* PID_Motor[nmr].Dout +c2* processValue+c3* vPID-> Last_Encoder[nmr];

 

vPID->result=vPID->PID_Motor[nmr].Kp*thisError+vPID->integralgain*vPID->integral+vPID-> derivative;

  vPID->lasterror=thisError;


}
*/
uint8 pianliflag;
uint8 col_mid1;
int col_test;
uint8 turn_left_flag;
uint8 turn_right_flag;
uint8 col_start_flag=1;
uint8 gaosu_flag = 0;
uint8 bupian_count = 0;  
float xishu  = 0;
void dir_pid()
{
  if(control_mode == 0 )
  {
    if(col_position_judge > col_mid1)
    {
      Dir_pid.mid_err = ((col_mid1-col_position_judge)*94)/(188-col_mid1);
    }
    else
    {
      Dir_pid.mid_err = ((col_mid1-col_position_judge)*94)/col_mid1;
    }
  }
  else if (control_mode ==1)
  {
     if(col_position_judge > Ki)
    {
      Dir_pid.mid_err = ((Ki-col_position_judge)*94)/(188-Ki);
    }
    else
    {
      Dir_pid.mid_err = ((Ki-col_position_judge)*94)/Ki;
    }
  }
   Dir_pid.KP = (float)KP;
    Dir_pid.KD = (float)KD;

  /*else
  {
    Dir_pid.KP = 255.0+(float)DKPN;
    Dir_pid.KD = (float)Kd;
    gaosu_flag =255;
  }*/
    //col_test     =  94  - col_position;
  if(  (Dir_pid.mid_err != 0 ) || (pianliflag ==1 && (last_col_longest >10 ||last_col_longest_send >10) ))
  {
    if(col_start_flag ==1)
   {
   if( Dir_pid.mid_err<0)
   {
    test_col_flag = 1;//右边
   }
  else
  {
    test_col_flag = 0;//左边
  }
     col_start_flag =0;
    }
   if(test_col_flag ==1)
  {
    // Dir_pid.mid_err = col_test - col_mid1;
     turn_left_flag =  1;
     //turn_right_flag = 0;
  }
   else
   {
    // Dir_pid.mid_err = col_test + col_mid1;
     turn_left_flag =  0;
   //  turn_right_flag = 1;
   }
   if(Dir_pid.mid_err < 10 && Dir_pid.mid_err >-10 )
  {

    pianliflag = 0;
    bupian_count ++;

  }
  if(Dir_pid.mid_err >20 && Dir_pid.mid_err < -20)
    
  {
    pianliflag = 1;
    bupian_count = 0;
  }

   if(fabs(Dir_pid.mid_err) < 30)
   {
     DKI_SWITCH = 1;
   }
   else
   {
     DKI_SWITCH = 0;
   }

    Dir_pid.dir_out=Get_MxMi((Dir_pid.KP*Dir_pid.mid_err /*+ DKI_SWITCH*DKPN * Dir_pid.mid_err  */ + Dir_pid.KD*(Dir_pid.mid_err -Dir_pid.last_err))/100,150,-150);
     Dir_pid.last_err = Dir_pid.mid_err;
 
  }
}
#include "analysis.h"
#include "PID.h"
#include "include.h"
#include "common.h"
#include "circleflag.h"
#include "Encoder1.h"
#include  "Send.h"
//     设定值（Aim_Speed）                      A B C D   这个地方设定的时候可以做大十倍 然后再最后设定时候除以十；
int Circle_Right_Duty[4] = {50,50,50,50}; //向右转
int Circle_Left_Duty[4] = {50,50,50,50}; //向左转
int Line_Left_Duty[4] = {50,50,50,50}; // 直走靠左走
int Line_Right_Duty[4] = {50,50,50,50}; //直走靠右走
int Left_Sidesway[4] = {50,50,50,50}; // 向左横移
int Right_Sidesway[4] = {50,50,50,50}; //向右横移
int Line_Back_Duty[4] = {50,50,50,50} ;//倒车

int modeflag = 0; // 0 直线 1右转弯 2左转弯 3 4 5 6 7
int line_flag = 0;
double Vx1,Vy1;
float Vz1 = 0;
unsigned char nmr1=0;
float row_position_judge = 0;
uint16 col_position_judge = 0;
uint8 NT_judge = 0;
uint8 fakeflag_judge = 0;
int last_col_longest_judge = 0;
int barrier_flag =1;
uint8 big_turn_edge =  60;
int speed_flag =0;
int jiansuflag = 0;
uint8 testflag = 0;
int  baiweiflag = 0;
uint8 speed_count= 0;
uint8 speed_up_flag = 0;
//暂未考虑闭环
/*void motor_judgement()
{
	if (row_position >= LONG_DISTANCE_EDGE && row_position != 120)  //设为加速mode
	{
	  //冲冲冲
	}
}*/


	void mode_choose()  //分析完后 满足啥条件就进入啥 然后进行PID修改
	{
            if(control_mode == 0)
            { 
              last_col_longest_judge = last_col_longest;
              NT_judge = NT;
              fakeflag_judge = fakeflag;
              col_position_judge = col_position;
            }
            else
            {
              last_col_longest_judge = last_col_longest_send;
              NT_judge = NT_send;
              fakeflag_judge = fakeflag_send;
              col_position_judge = col_position_send;
            }
                if ((last_col_longest_judge >row_edge)||NT_judge >= NOPHOTO_EDGE )//|| last_col_longest>20)// || ((row_position >= row_edge) && (row_position != 120))) //无图像或者满足条件即转向
		{
                   
                   if(NT_judge >= big_turn_edge)
                   {
                     NT_judge = big_turn_edge;
         
                   }
                         if( last_col_longest_judge >row_edge)
                       {
                          modeflag =2;
                       }                 
                       else if(NT_judge > 3 )
                       {
                         modeflag =4;
                        }
                           
                   free_flag = 0;
                        line_flag = 0;
                     col_start_flag = 1;
                      
                
		}
      
		else                              //这里有可能是ROW_POSITION 大于小于某值 就怎么怎么样；
		{
                        
                       
                        line_flag ++;
                       if(pianliflag == 0)
                       {
                         speed_flag ++;                    
                       }
                       else
                       {
                         speed_flag = 0;
                       }

                       if(speed_check > 95)
                       {
                         if(last_col_longest_judge >speed_down-2)
                              {
                                  modeflag = 3;
                                  
                                     speed_flag  = 0 ;
                              }
                          else
                          {
                            modeflag = 0;
                          }
                       }
                       else
                       {
                           if(last_col_longest_judge >speed_down)
                              {
                                  modeflag = 3;
                                  
                                     speed_flag  = 0 ;
                              }
                          else
                          {
                            modeflag = 0;
                          }
                       }
                      

		}
		

        }
void motor_circle_judgement()
{
	if (modeflag == 0)
	{
       
		// McNamm_wheel_control(-0.5,20,0);
               /* Vy1=20;
                Vx1=0;
                Vz1=0;*/
           
          if(barrierflag ==1)
          {
             Vy =-20;
             
           } 
          else if(pianliflag ==1)
           {
             Vy = 10;
           }
           else
           {
            if(bupian_count > 20)
              
             Vy = DKDN;
            
            else
             
             Vy = Kd;
           }
          }
             
                 //McNamm_wheel_control(Vx1,Vy1,Vz1);
	
       
	else if (modeflag == 1) //右转弯
	{
          if(mianbiflag ==0)
          {
            Vy = 30;
           Vz = -40;
          }
          else
           {
             Vy = -20;
             Vx =0;
           }
        } 
	else if (modeflag == 2 ) //左转弯 
	{
       if(jiansuflag ==1)
         {
          if (speed_check < 20)
          {
             Vx =23;        
            testflag = 1;
          }
           else if(speed_check<25)
          {           
             Vx =23;
            testflag = 2;
          }
          else if(speed_check<30)
          {    
             Vx =28; 
            testflag = 3;
          }
          else if(speed_check <35)
          {
             Vx =33;
            testflag = 4;
          }
           else if(speed_check<40)
          {

             Vx =45;
            testflag = 5;
          }
          else if(speed_check <200)
          {

             Vx =45;
            testflag = 21;
          }
               
         else
         {     
           Vy = 30;
            jiansuflag = 0;
         }
          jiansuflag = 0; 
        }
       else
         {
         
           Vy = 30;
          jiansuflag = 1;
         }
           //Vx  = 0;
       
           Vz = 0;
           bupian_count  = 0;
	}

	else if (modeflag == 3 )//左偏移 
	{
          Vy = 35;
  
        jiansuflag = 1;
        }
        
        else if(modeflag ==4)
        {
          //加在这里 灭灯后的减速过程
      /*    if (speed_check > 50)
         {
             Vy = -Kd;
              
          }
          else
          {
             Vy = 0;
          }*/

         if(NT_judge!=big_turn_edge)
         {
          Vy = -10;
          Vz = -DKPN;
         }
         else
         {
           Vy = 20;
           Vz = -80;
         }
         Vx= 0;
        // }
          
        }
}
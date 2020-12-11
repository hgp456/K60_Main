#include "analysis.h"
#include "circleflag.h"
#include "common.h"
#include "include.h"
#include "constant.h"
#include "LCD_menu.h"
#include "math.h"
#include "PID.h"
#include "motor.h"
#include "init.h"
#include "interrupt.h"
#include  "stdlib.h"
#include "Encoder1.h"
 uint8 cameraA_flag=0;
 uint8 cameraB_flag=0;
 uint8 mode_change_flag = 0;
 uint8 Aim_speed_change_flag = 0;
 
uint8 free_flag  = 0; // 0表示现在是空闲的 可以被抢占 
uint8 control_mode = 0 ; //0表示往前，1表示往后
void shuangshe_use() // 双摄抢占 每次当空闲flag =0时两个完成数据接收后都会进入该判断  
                 // 第一个确认有灯后就使用这个 直到 超过一定范围
{
 
   if ((col_position != 0) &&(NT<NOPHOTO_EDGE))
   {
     control_mode = 0;
/*     uart_rx_irq_dis(UART0);
     uart_rx_irq_en(UART1);
     uart_rx_irq_en(UART5);*/
   }
   else if((col_position_send != 0)&&(NT_send < NOPHOTO_EDGE))
   {
      control_mode = 1;
        dir_pid();
        /* uart_rx_irq_en(UART0);
          uart_rx_irq_dis(UART1);
           uart_rx_irq_dis(UART5);*/
   }
   
    mode_choose();
   //free_flag = 1;

}
void Aim_Speed_Change()
{
       int16 temp[2];
        temp[0] = Aim_Speed[0];
        temp[1] = Aim_Speed[1];
       Aim_Speed[0]  =-Aim_Speed[2];  //A轮   给C轮的速度并取反
       Aim_Speed[1]  =-Aim_Speed[3];  //B轮  给D轮的速度并取反
       Aim_Speed[2]  =-temp[0];  //C轮   给A轮的速度并取反
       Aim_Speed[3]  =-temp[1];  //D轮   给B轮的速度并取反
}
void camera_choose()
{
if (fakeflag <=3 && NT <NOPHOTO_EDGE)
{
  cameraA_flag = 1;
}
else
{
  cameraA_flag = 0;
}
if (B_fake_flag<=3 && B_NT<NOPHOTO_EDGE)
{
  cameraB_flag = 1;
}
else 
{
  cameraB_flag = 0;
}
if (cameraA_flag ==1 && cameraB_flag == 0)
{
   Aim_speed_change_flag =0;
}
if (cameraA_flag ==0 && cameraB_flag == 1)
{ 
   Aim_speed_change_flag =1;
}

          if(Aim_speed_change_flag==1)
          {
            //row_position_judge = B_row_position;
            col_position_judge = B_col_position;
            last_col_longest_judge = B_last_col_longest;
            NT_judge = B_NT;
            fakeflag_judge = B_fake_flag;
          }
          else
          {
          //  row_position_judge = row_position;
            col_position_judge = col_position;
            last_col_longest_judge  = last_col_longest;
            fakeflag_judge = fakeflag;
            NT_judge = NT;
          }
}




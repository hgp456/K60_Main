#include "common.h"
#include "include.h"
#include "constant.h"
#include "interrupt.h"
#include "PID.h"
#include "LCD_menu.h"
#include "analysis.h"
#include "VCAN_OV7725_Eagle.h"
#include "init.h"
#include "circleflag.h"
#include "Send.h"
uint8 start_flag = 0;

//测速用
/*
void main()
{

 init_all();
  EnableInterrupts;
   nrf_init();
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);
    enable_irq(PORTE_IRQn);
    //MT9V032_camera_init();
   for(;;)
   {
    if(openmv_finish_flag)
    {
      openmv_finish_flag = 0;
       if(openmv_col_position!=0)
          {
             start_flag=1;
          }
         dir_pid();
         mode_choose();
          nrf_send();
    }
   }
}*/

void main()
{
  init_all();
  EnableInterrupts;
  //uart_putchar(UART2,threshold);
     MT9V032_camera_init();
    nrf_init();
    set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);
    enable_irq(PORTE_IRQn);     
   while(1)
    {
    Motor_Move_Api(Forward,3000);
    DELAY_MS(2000);
    }
   /*for(;;)   //打开此处
   {
       if(mt9v032_finish_flag)
        {
         
          mt9v032_finish_flag=0;
 
        Photoanly();
         shuangshe_use();
          dir_pid(); 
         if(col_position!=0 || col_position_send != 0)
          {
             start_flag=1;
          }
         //camera_choose();
        //if(stopflag==1)
          {
          //  Set_Motor(-920.0,-3080.0,-920.0,-3080.0);
          //  DELAY_MS(750) ;
          //   Set_Motor(3000,3000,3000,3000);
          //  DELAY_MS(500) ;
          //  stopflag = 0 ;
          //  NT = 0;
          //}       
         

        }
    }*/     //打开此处
}
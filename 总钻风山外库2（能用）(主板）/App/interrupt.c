#include "common.h"
#include "include.h"
#include "PID.h"
#include "Send.h"
#include "analysis.h"
#include "motor.h"
#include "init.h"
#include "Encoder1.h"
#include "circleflag.h"
#include "math.h"
#include "interrupt.h"
uint8 imgbuff[CAMERA_SIZE]; 
uint16 dis=0,ang=0;
int16 stopcount;
uint8 data_count; 
uint8 tof_rec[7];
int16 barrier_dis = 2000;
int16 barrier_dis_right = 2000;
int16 barrier_dis_left =2000;
int16 barrierdis[11]={2000};
 uint8 TOF_left_byteReceived;
 uint8 TOF_right_byteReceived;
 uint8 TOFbyteReceived;
 int16 barrieravg;
 int16 last_barrieravg;
 uint8 barrierflag = 0;
   uint8 openmv_fakeflag;
  uint8 openmv_NT;
  uint8 openmv_last_col_longest;
  uint8 openmv_col_position;
  uint8 openmv_finish_flag;
  uint8 openmvbyteReceive; 
  uint8 openmv_count= 0;
  int A_OUT,B_OUT,C_OUT,D_OUT;
  int barrieravg_pre;
  int barrieravg_back;
   uint8 back_camera = 0;
  uint8 barrier_left_flag = 0;
  uint8 barrier_right_flag = 0;
  float Vx_turn = 0;
  int barrier_dis_mid = 2000;
  int barrier_mid1_flag =0;
  int barrier_left1_flag =0;
  int barrier_right1_flag =0;
  int mianbiflag = 0;
  int tof_count = 0;
  uint8  last_col_longest_send   = 40;
  uint8 col_position_send =  0;
  uint8   fakeflag_send   =  4;
  uint8    NT_send =  4;
   uint8 barrier_Received;
   uint8 barrier_back_right;
   uint8 barrier_back_left;
   uint8 back_barrier_state;
   uint8 stopflag =0;
  void barrier_anas()
{
  if(barrier_dis_left <1000 )
   {
    
     barrier_left1_flag = 1;
   }
   else
   {
      barrier_left1_flag = 0;
   }
   if(barrier_dis_right <1000)
    {
    
      barrier_right1_flag = 1;
    }
   else
   {
  
     barrier_right1_flag = 0;
   }
if(control_mode == 0 )
{
  if(modeflag == 0)
  {
 
    if( barrier_left1_flag ==1 )
    {
      Vx =60;
      barrierflag = 1;
    }
    else if(barrier_right1_flag ==1)
    {
      Vx = -60;
      barrierflag =1;
    }
    else
    {
      Vx =0 ;
      barrierflag = 0; 
    }
  }
  
  else
  {
     barrierflag = 0;
     mianbiflag =0;
    // tof_count = 0;
  }
}
  else
  {
     if(modeflag == 0)
  {
 
    if( back_barrier_state ==0x01 )
    {
      Vx =60;
      barrierflag = 1;
    }
    else if(back_barrier_state == 0x02)
    {
      Vx = -60;
      barrierflag =1;
    }
    else
    {
      Vx =0 ;
      barrierflag = 0; 
    }
  }
  
  else
  {
     barrierflag = 0;
     mianbiflag =0;
    // tof_count = 0;
  }
  }
}
  /*if(barrier_left_flag ==1 && barrier_right_flag ==0)
  {
       Vx = 35;
  }
  else if ( barrier_right_flag ==1 && barrier_left_flag ==0)
  {
        Vx  = -35;
  }
  else if( barrier_right_flag ==1 && barrier_left_flag ==1)
  {
  }*/
  /*else
  { 
     Vx =  0 ;
     barrierflag =0;
  }
 
  }
   else
  {
     barrierflag =0;
  }*/
  void UART0_IRQhandler()
  {
         static uint8_t rec_barrier_back_temp[10]={0};
 	static uint8_t rec_barrier_back_state = 0;
	static uint8_t rec_barrier_back_len =4,_data_barrier_back_cnt = 0;
	 uint8 barrier_back_byteReceived;
	uart_getchar(UART0,&barrier_back_byteReceived);
        back_barrier_state = barrier_back_byteReceived;
        barrier_anas();
	
  }
 void receive_barrier()   //这里是摄像头
 {
        static uint8_t rec_barrier_temp[10]={0};
 	static uint8_t rec_barrier_state = 0;
	static uint8_t rec_barrier_len =4,_data_barrier_cnt = 0;
	 uint8 barrier_byteReceived;
	uart_getchar(UART4,&barrier_byteReceived);
   
	if(barrier_byteReceived == 0xfd && rec_barrier_state==0)                              
	{
		rec_barrier_state = 1;
		rec_barrier_temp[0] = barrier_byteReceived;
	}			
	else if(barrier_byteReceived == 0xfb && rec_barrier_state==1)													
	{
		rec_barrier_state=2;
		rec_barrier_temp[1] = barrier_byteReceived;
	}
	else if(rec_barrier_state == 2 && rec_barrier_len>0)
	{
		rec_barrier_len--;
		rec_barrier_temp[2+_data_barrier_cnt++] = barrier_byteReceived;
		if(rec_barrier_len==0)
			rec_barrier_state = 3;
	}
	else if(rec_barrier_state == 3)
	{
                /*rec_len_ted --;
                rec_temp[6+_data_cnt_ted] = byteReceived;
                if(rec_len_ted==0)
                         rec_state = 4;*/
		if(barrier_byteReceived == 0xfc)  
                {

			//barrier_dis_mid = ((int)(rec_barrier_temp[2]<<8)&0xff00)|rec_barrier_temp[3];         
			//barrier_anas();
                          last_col_longest_send   =  rec_barrier_temp[2];
                          col_position_send =      rec_barrier_temp[3];
                            fakeflag_send   = rec_barrier_temp[4];
                              NT_send =  rec_barrier_temp[5];
                             shuangshe_use(); //抢占
			rec_barrier_state = 0;
			rec_barrier_len = 4;                                                
			_data_barrier_cnt = 0;
			return;
		}
		else    
		{
			_data_barrier_cnt = 0;
			rec_barrier_len = 4;																							 
			rec_barrier_state = 0;
			return;
		}

	}
        
	else
	{
		rec_barrier_state = 0;
		rec_barrier_len = 4;                                                 
		_data_barrier_cnt = 0;
	}
 }
void PIT1_IRQHandler()  //定时控制编码器闭环
{
	PIT_Flag_Clear(PIT1);
       count_get(); 
      if(start_flag ==1 )
    {
      if(stopflag ==0)
     {
	 
        motor_circle_judgement();
        //barrier_anas();
          McNamm_wheel_control();
           if(abs(count_A) <15 || abs(count_B)<15 || abs(count_C)<15 || abs(count_D)<15)
          {
            stopcount++;
            if(stopcount >500)
            {
              stopflag = 1; 
              stopcount = 0;
            }
          }
          else
          {
            stopflag = 0;
            stopcount = 0;
          }
     }
    }
}
  //}
        //}
      //含PID（）;      



void UART2_IRQhandler()//从板
{  
                led(LED0,LED_ON);
                receive_count();
                //barrier_anas();
}

void UART4_IRQhandler()//从板
{  
        receive_barrier();
         
}

  

void UART1_IRQhandler()//预留口
{
        static uint8_t rec_TOF_left[7]={0};
	static uint8_t rec_TOF_left_state = 0;
	static uint8_t  data_TOF_left_cnt = 0;                  
	uart_getchar(UART1,&TOF_left_byteReceived);
	if(TOF_left_byteReceived == 0x0D &&rec_TOF_left_state==0)                              
	{
		rec_TOF_left_state = 1;
		rec_TOF_left[0] = TOF_left_byteReceived;
	}			
	else if(TOF_left_byteReceived == 0x0A && rec_TOF_left_state==1)													
	{
		rec_TOF_left_state=2;
		rec_TOF_left[1] = TOF_left_byteReceived;
	}
	else if(rec_TOF_left_state == 2 )
	{
          if(TOF_left_byteReceived!=0x6D)
          {
		rec_TOF_left[2+data_TOF_left_cnt++] = TOF_left_byteReceived;
          }
          else
          {
            rec_TOF_left_state =3;
          }
                
		
	}
	else if(rec_TOF_left_state == 3)
	{
               
		if(TOF_left_byteReceived == 0x6D)  																			
		{
                            
                        if(data_TOF_left_cnt==1)
                        {
                          rec_TOF_left[2]=rec_TOF_left[2]-0x30;
                          barrier_dis_left = rec_TOF_left[2] ;
                         // barrier_anas();
                        }
                        else if(data_TOF_left_cnt==2)
                        {
                          rec_TOF_left[2]=rec_TOF_left[2]-0x30;
                          rec_TOF_left[3]=rec_TOF_left[3]-0x30;
                           barrier_dis_left = rec_TOF_left[2] * 10 + rec_TOF_left[3]*1;
                          // barrier_anas();
                        }
                        else if (data_TOF_left_cnt==3)
                        {
                          rec_TOF_left[2]=rec_TOF_left[2]-0x30;
                          rec_TOF_left[3]=rec_TOF_left[3]-0x30;
                          rec_TOF_left[4]=rec_TOF_left[4]-0x30;
                           barrier_dis_left = rec_TOF_left[2] * 100 + rec_TOF_left[3]*10 + rec_TOF_left[4]*1;
                           //barrier_anas();
                        }
                        else if(data_TOF_left_cnt==4)
                        {
                           rec_TOF_left[2]=rec_TOF_left[2]-0x30;
                          rec_TOF_left[3]=rec_TOF_left[3]-0x30;
                          rec_TOF_left[4]=rec_TOF_left[4]-0x30;
                          rec_TOF_left[5]=rec_TOF_left[5]-0x30;
                           barrier_dis_left = rec_TOF_left[2] * 1000 + rec_TOF_left[3]*100 + rec_TOF_left[4]*10+rec_TOF_left[5]; 
                         //  barrier_anas();
                        }
                        barrier_anas();
			rec_TOF_left_state = 0;                                          
			data_TOF_left_cnt = 0;
			return;
                        
		}
		else    
		{
			data_TOF_left_cnt = 0;																						 
			rec_TOF_left_state = 0;
			return;
		}

	}
	else
	{
		rec_TOF_left_state = 0;                                                 
		data_TOF_left_cnt = 0;
        }
  
}
void UART5_IRQhandler()//OPENMV接收 可用
{  
          static uint8_t rec_TOF_right[7]={0};
	static uint8_t rec_TOF_right_state = 0;
	static uint8_t  data_TOF_right_cnt = 0;                  
	uart_getchar(UART5,&TOF_right_byteReceived);
	if(TOF_right_byteReceived == 0x0D &&rec_TOF_right_state==0)                              
	{
		rec_TOF_right_state = 1;
		rec_TOF_right[0] = TOF_right_byteReceived;
	}			
	else if(TOF_right_byteReceived == 0x0A && rec_TOF_right_state==1)													
	{
		rec_TOF_right_state=2;
		rec_TOF_right[1] = TOF_right_byteReceived;
	}
	else if(rec_TOF_right_state == 2 )
	{
          if(TOF_right_byteReceived!=0x6D)
          {
		rec_TOF_right[2+data_TOF_right_cnt++] = TOF_right_byteReceived;
          }
          else
          {
            rec_TOF_right_state =3;
          }
                
		
	}
	else if(rec_TOF_right_state == 3)
	{
               
		if(TOF_right_byteReceived == 0x6D)  																			
		{
                            
                        if(data_TOF_right_cnt==1)
                        {
                          rec_TOF_right[2]=rec_TOF_right[2]-0x30;
                          barrier_dis_right = rec_TOF_right[2] ;
                        //  barrier_anas();
                        }
                        else if(data_TOF_right_cnt==2)
                        {
                          rec_TOF_right[2]=rec_TOF_right[2]-0x30;
                          rec_TOF_right[3]=rec_TOF_right[3]-0x30;
                           barrier_dis_right = rec_TOF_right[2] * 10 + rec_TOF_right[3]*1;
                           //barrier_anas();
                        }
                        else if (data_TOF_right_cnt==3)
                        {
                          rec_TOF_right[2]=rec_TOF_right[2]-0x30;
                          rec_TOF_right[3]=rec_TOF_right[3]-0x30;
                          rec_TOF_right[4]=rec_TOF_right[4]-0x30;
                           barrier_dis_right = rec_TOF_right[2] * 100 + rec_TOF_right[3]*10 + rec_TOF_right[4]*1;
                           //barrier_anas();
                        }
                        else if(data_TOF_right_cnt==4)
                        {
                           rec_TOF_right[2]=rec_TOF_right[2]-0x30;
                          rec_TOF_right[3]=rec_TOF_right[3]-0x30;
                          rec_TOF_right[4]=rec_TOF_right[4]-0x30;
                          rec_TOF_right[5]=rec_TOF_right[5]-0x30;
                           barrier_dis_right = rec_TOF_right[2] * 1000 + rec_TOF_right[3]*100 + rec_TOF_right[4]*10+rec_TOF_right[5]; 
                          // barrier_anas();
                        }
                        barrier_anas();
			rec_TOF_right_state = 0;                                          
			data_TOF_right_cnt = 0;
			return;
		}
		else    
		{
			data_TOF_right_cnt = 0;																						 
			rec_TOF_right_state = 0;
			return;
		}

	}
	else
	{
		rec_TOF_right_state = 0;                                                 
		data_TOF_right_cnt = 0;
        }

}

void PORTE_IRQHandler()
{
	uint8 n;  //引脚号
	uint32 flag;

	flag = PORTE_ISFR;
	PORTE_ISFR = ~0;  //清中断标志位

	n = 27;
	if (flag& (1 << n))
	{
		nrf_handler();
	}
}


void PORTA_IRQHandler() //摄像头
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
      VSYNC();
      
    }
}

void DMA0_IRQHandler() //摄像头
{
    DMA_IRQ_CLEAN(DMA_CH0);
    row_finished();
}


void PIT0_IRQHandler()
{
  PIT_Flag_Clear(PIT0);
  A_OUT = PID_Motor[0].Pout - Dir_pid.dir_out ;
  B_OUT = PID_Motor[1].Pout + Dir_pid.dir_out ;
  C_OUT = PID_Motor[2].Pout - Dir_pid.dir_out ;
  D_OUT = PID_Motor[3].Pout + Dir_pid.dir_out ;
  Set_Motor(A_OUT,B_OUT,C_OUT,D_OUT);
}



void USART3_IRQHandler(void)  //摄像头
{
    UARTn_e uratn = UART3;
    if(UART_S1_REG(UARTN[uratn]) & UART_S1_RDRF_MASK)   //接收数据寄存器满
    {
        mt9v032_cof_uart_interrupt();                    //串口接收中断
    }
}


void DMA10_IRQHandler(void)
{
  DMA_IRQ_CLEAN(DMA_CH10);
   openmv_anas();
     led_turn(LED1);
  
}

void DMA_Error_IRQHandler(void)
{
  uint32 error = DMA_ES_REG(DMA_BASE_PTR);
  led_turn(LED1);
}

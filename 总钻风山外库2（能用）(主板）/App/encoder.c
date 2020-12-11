
#include "common.h"
#include "include.h"
#include "Encoder1.h"
#include "circleflag.h"
#include "init.h"
#include "interrupt.h"
#include "math.h"
int16 count_A;
int16 count_B;
int16 count_C;
int16 count_D;


int16 count_LF;
int16 count_LB;
int16 count_RF;
int16 count_RB;

uint8 B_row_position;
uint8 B_col_position;
uint8 B_fake_flag;
uint8 B_NT;
uint8 B_last_col_longest;
uint8 qie_left_flag;
uint8 A_flag;
int16 speed_check;
uint8 barrier_congban_flag;
void count_get()
{
        count_D = -ftm_quad_get(FTM2);
	count_B = -ftm_quad_get(FTM1);
	count_C = -count_LF;
        count_A = -count_LB;
        
     // if(modeflag==0 || modeflag ==3 ||modeflag ==4 )
     //{
        
        speed_check = abs((count_A +count_B + count_C + count_D)/4);
        if((count_A + count_C) > (count_B+count_D) )
        {
          qie_left_flag = 0;
        }
        else
        {
          qie_left_flag = 0;
        }
 
     // }
        ftm_quad_clean(FTM1);
        ftm_quad_clean(FTM2);
      



}

void receive_count()  //接收从板数据
{
        
	static uint8_t rec_temp[10]={0};
	static uint8_t rec_state = 0;
	static uint8_t rec_len =4,_data_cnt = 0;
	 uint8 byteReceived;
	uart_getchar(UART2,&byteReceived);
	if(byteReceived == 0xfe && rec_state==0)                              
	{
		rec_state = 1;
		rec_temp[0] = byteReceived;
	}			
	else if(byteReceived == 0xfa && rec_state==1)													
	{
		rec_state=2;
		rec_temp[1] = byteReceived;
	}
	else if(rec_state == 2 && rec_len>0)
	{
		rec_len--;
		rec_temp[2+_data_cnt++] = byteReceived;
		if(rec_len==0)
			rec_state = 3;
	}
	else if(rec_state == 3)
	{
                /*rec_len_ted --;
                rec_temp[6+_data_cnt_ted] = byteReceived;
                if(rec_len_ted==0)
                         rec_state = 4;*/
		if(byteReceived == 0xff)  
                {

			count_LF = ((int)(rec_temp[2]<<8)&0xff00)|rec_temp[3];         
			count_LB = ((int)(rec_temp[4]<<8)&0xff00)|rec_temp[5];

			rec_state = 0;
			rec_len = 4;                                                
			_data_cnt = 0;
			return;
		}
		else    
		{
			_data_cnt = 0;
			rec_len = 4;																							 
			rec_state = 0;
			return;
		}

	}
        
	else
	{
		rec_state = 0;
		rec_len = 4;                                                 
		_data_cnt = 0;
	}
}

/*void count_get_test() // 摘录自他校 不知具体效果 因为另俩编码器在从板上 可能需要写两部分程序 
                      // 这样一个程序 如果涉及反转 应该怎么办？？？？？  可能存在的意义不是很重要
	                  // 也可以 有两或三个值为负的时候 清空上层数据 重新记录 
{
	count_A = (int16)ftm_quad_get(FTM1);
	ftm_quad_clean(FTM1);
	count_B = (int16)ftm_quad_get(FTM2);
	ftm_quad_clean(FTM2);

        count_LF1[0] = count_A;
	count_LF1[1] = count_LF1[0];
	count_LF1[2] = count_LF1[1];
	count_LF1[3] = count_LF1[2];
	count_LF1[4] = count_LF1[3];
	count_LF1[5] = count_LF1[4];
	count_LF1[6] = count_LF1[5];
	count_LF1[7] = count_LF1[6];
	count_LF1[8] = count_LF1[7];
	count_LF1[9] = count_LF1[8];

	count_LF = (count_LF1[0] + count_LF1[1] + count_LF1[2] + count_LF1[3] + count_LF1[4] + count_LF1[5] + count_LF1[6] + count_LF1[7] + count_LF1[8] + count_LF1[9]) / 10;
	Encoder[0] = count_LF;

        count_LB1[0] = count_B;
	count_LB1[1] = count_LB1[0];
	count_LB1[2] = count_LB1[1];
	count_LB1[3] = count_LB1[2];
	count_LB1[4] = count_LB1[3];
	count_LB1[5] = count_LB1[4];
	count_LB1[6] = count_LB1[5];
	count_LB1[7] = count_LB1[6];
	count_LB1[8] = count_LB1[7];
	count_LB1[9] = count_LB1[8];

	count_LB = (count_LB1[0] + count_LB1[1] + count_LB1[2] + count_LB1[3] + count_LB1[4] + count_LB1[5] + count_LB1[6] + count_LB1[7] + count_LB1[8] + count_LB1[9]) / 10;
	Encoder[1] = count_LB;
}*/
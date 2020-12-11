#include "include.h"
#include "common.h"
#include "analysis.h"
#include "circleflag.h"
#include "Send.h"
#include "constant.h"
#include "LCD_menu.h"
#include "math.h"
#include "PID.h"
#include "motor.h"
#include "init.h"
#include "interrupt.h"
#include  "stdlib.h"
      uint8 threshold=20;
      uint8  sd_right_sp=74;
     uint8  sd_right_long_sp=78;
     uint16 HIGH_SPEED;
     uint8 singlecarflag=0;
     uint8 duzhuanmode_flag=1;
     uint8 zhuanwan_flag=0;
     uint8 first_flag=0;
     uint8 countflag_while_ud=0;
     uint8 first_judge1=0;
     uint8 first_judge2=0;
     float new_row_edge[16];
     
     uint8 Kp = 90,Ki = 0,Kd;
     uint8 KP =90,KD=0;
     uint8 DKPN = 90,DKDN =0;
     uint8 col_mid_back;
void get_switch()
{
    uint8 temp[8]; 
      
         //从flash读取
       //row_edge=flash_read(SECTOR_NUM,0,uint8);
        threshold=flash_read(SECTOR_NUM,0,uint8);   
        //row_edge=threshold;
        
        row_edge=flash_read(SECTOR_NUM,8*1,uint8); 
        //sd_right_long_sp=SD_RIGHT_LONG_SP;//覆盖
        Kp=flash_read(SECTOR_NUM,8*2,uint8);
        Ki=flash_read(SECTOR_NUM,8*3,uint8);
        //sd_right_sp=SD_RIGHT_SP;          //覆盖           
       // temp[0]=flash_read(SECTOR_NUM,8*3,uint8);       
        col_mid1=flash_read(SECTOR_NUM,8*4,uint8); 
        //temp[2]=flash_read(SECTOR_NUM,8*5,uint8);  
        KP=flash_read(SECTOR_NUM,8*5,uint8); 
     //  temp[3]=flash_read(SECTOR_NUM,8*6,uint8);
        KD=flash_read(SECTOR_NUM,8*6,uint8); 
        row_cut=flash_read(SECTOR_NUM,8*7,uint8);       
        DKPN=flash_read(SECTOR_NUM,8*8,uint8); 
        DKDN=flash_read(SECTOR_NUM,8*9,uint8);     
         speed_down =flash_read(SECTOR_NUM,8*10,uint8);
        Kd=flash_read(SECTOR_NUM,8*11,uint8);
        
        row_edge_array[2]=temp[0]+(float)temp[1]/10;
       // HIGH_SPEED=(uint16)row_edge_array[2];
        row_edge_array[4]=temp[2]+(float)temp[3]/10;
        row_edge_array[6]=temp[4]+(float)temp[5]/10;
        row_edge_array[8]=temp[6]+(float)temp[7]/10;
        new_row_edge[2]+=row_edge_array[2]-50.0;   //35_50速度 距离加减
        new_row_edge[3]+=row_edge_array[2]-50.0;
        new_row_edge[4]+=row_edge_array[4]-50.0;
        new_row_edge[5]+=row_edge_array[4]-50.0;
        new_row_edge[6]+=row_edge_array[6]-50.0;  //55-70速度  距离加减
        new_row_edge[7]+=row_edge_array[6]-50.0;
        new_row_edge[8]+=row_edge_array[8]-50.0;
        new_row_edge[9]+=row_edge_array[8]-50.0;
          
        
       
}

void uart2_init()
{
       uart_init(UART2,115200);
       set_vector_handler(UART2_RX_TX_VECTORn,UART2_IRQhandler);  // 设置中断服务函数到中断向量表里 ，假如 VCAN_PORT == UART3 ，那么 UART0_RX_TX_VECTORn + VCAN_PORT*2 = UART3_RX_TX_VECTORn
        uart_rx_irq_en(UART2);
         enable_irq(UART2_RX_TX_IRQn);  //key？

      
}
void uart4_init()
{
       uart_init(UART4,115200);
       set_vector_handler(UART4_RX_TX_VECTORn,UART4_IRQhandler);  // 设置中断服务函数到中断向量表里 ，假如 VCAN_PORT == UART3 ，那么 UART0_RX_TX_VECTORn + VCAN_PORT*2 = UART3_RX_TX_VECTORn
        uart_rx_irq_en(UART4);
         enable_irq(UART4_RX_TX_IRQn);  //key？

      
}

void uart0_init()
{
       uart_init(UART0,115200);
       set_vector_handler(UART0_RX_TX_VECTORn,UART0_IRQhandler);  // 设置中断服务函数到中断向量表里 ，假如 VCAN_PORT == UART3 ，那么 UART0_RX_TX_VECTORn + VCAN_PORT*2 = UART3_RX_TX_VECTORn
        uart_rx_irq_en(UART0);
         enable_irq(UART0_RX_TX_IRQn);  //key？

      
}

void uart1_init()
{
       uart_init(UART1,9600);
       set_vector_handler(UART1_RX_TX_VECTORn,UART1_IRQhandler);  // 设置中断服务函数到中断向量表里 ，假如 VCAN_PORT == UART3 ，那么 UART0_RX_TX_VECTORn + VCAN_PORT*2 = UART3_RX_TX_VECTORn
        uart_rx_irq_en(UART1);
         enable_irq(UART1_RX_TX_IRQn);  //key？

      
}


void uart5_init()
{

       uart_init(UART5,9600);
       set_vector_handler(UART5_RX_TX_VECTORn,UART5_IRQhandler);  // 设置中断服务函数到中断向量表里 ，假如 VCAN_PORT == UART3 ，那么 UART0_RX_TX_VECTORn + VCAN_PORT*2 = UART3_RX_TX_VECTORn
        uart_rx_irq_en(UART5);
         enable_irq(UART5_RX_TX_IRQn);  //key？

      
}

void Motor_Init()
{
	ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM, MOTOR_HZ, 10000); 
	ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM, MOTOR_HZ, 10000);   
	ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM, MOTOR_HZ, 10000);  
	ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM, MOTOR_HZ, 10000);
	//原作者此处第二个用了BACK_DUTY/10.0 猜想与PID调节有关 此处先完成。
      
        
        ftm_pwm_init(NEW_MOTOR_FTM, NEW_MOTOR1_PWM,MOTOR_HZ,10000);  //B
        ftm_pwm_init(NEW_MOTOR_FTM, NEW_MOTOR2_PWM,MOTOR_HZ,10000); 
        ftm_pwm_init(NEW_MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,10000);  //D
        ftm_pwm_init(NEW_MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,10000); 
        

}

void Enconder_init()   //TPM似乎不够 可能需要外界线将其导入到另外一边
{
	//ftm_quad_init(FTM1, FTM_PS_1, FTM1_QDPHA_PIN, FTM1_QDPHB_PIN);
	//ftm_quad_init(FTM2, FTM_PS_1, FTM2_QDPHA_PIN, FTM2_QDPHB_PIN);
        ftm_quad_init(FTM1);
        ftm_quad_init(FTM2);
        
}

 void switch_init()
{
  
    
      port_init (UP,ALT1 | PULLUP );
      port_init (DOWN,ALT1 | PULLUP );
      port_init (MIDDLE,ALT1 | PULLUP );
      port_init (RIGHT,ALT1 | PULLUP );
      port_init (LEFT,ALT1 | PULLUP );     
      
}
     


void init_all()
{
  
   uint8 temp[8]; 
      DisableInterrupts;
       
         uart0_init();
      uart1_init();
        uart2_init();
       uart4_init();
      uart5_init();
        Motor_Init();
	Enconder_init();
        switch_init();
        get_switch();
        
       //NRF
         
          
          //LCD
         LCD_init();
        menu_display();
         
         //摄像头
    //  camera_init(imgbuff);
     // set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);   
     // set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      
      
  
        //uart_init(UART2,115200);
   set_vector_handler(UART3_RX_TX_VECTORn , USART3_IRQHandler);
   set_vector_handler(DMA0_VECTORn,DMA0_IRQHandler);
   set_vector_handler(PORTA_VECTORn,PORTA_IRQHandler);
   
     // DMA_init();
       //中断优先级
    /*NVIC_SetPriorityGrouping(4);
    // NVIC_SetPriority ((IRQn_Type)PORTA_VECTORn,1);
    // NVIC_SetPriority ((IRQn_Type)DMA0_VECTORn,2);
     //NVIC_SetPriority ((IRQn_Type)UART3_RX_TX_VECTORn,3);
     //NVIC_SetPriority ((IRQn_Type)PIT0_VECTORn,6); 
     NVIC_SetPriority ((IRQn_Type)PIT1_VECTORn,3);
     NVIC_SetPriority ((IRQn_Type)UART2_RX_TX_VECTORn,2);
     NVIC_SetPriority ((IRQn_Type)DMA10_VECTORn,0); 
      NVIC_SetPriority ((IRQn_Type)DMA_Error_VECTORn,1); 
    // NVIC_SetPriority ((IRQn_Type)UART1_RX_TX_VECTORn,0);
      NVIC_SetPriority ((IRQn_Type)UART4_RX_TX_VECTORn,0);
      NVIC_SetPriority ((IRQn_Type)PORTE_VECTORn,5);*/
     
     NVIC_SetPriorityGrouping(4);
     NVIC_SetPriority ((IRQn_Type)PORTA_VECTORn,1);
     NVIC_SetPriority ((IRQn_Type)DMA0_VECTORn,0);
     NVIC_SetPriority ((IRQn_Type)UART3_RX_TX_VECTORn,2);
     NVIC_SetPriority ((IRQn_Type)UART4_RX_TX_VECTORn,6); 
     NVIC_SetPriority ((IRQn_Type)PIT1_VECTORn,4);
     NVIC_SetPriority ((IRQn_Type)UART2_RX_TX_VECTORn,3);
      NVIC_SetPriority ((IRQn_Type)UART5_RX_TX_VECTORn,7);
      NVIC_SetPriority ((IRQn_Type)UART1_RX_TX_VECTORn,8);
     NVIC_SetPriority ((IRQn_Type)UART0_RX_TX_VECTORn,5);
     //pid参数调整标志LED
      led_init(LED0); 
      led_init(LED1);
      led_init(LED2); 
      
            //初始化闪烁判断是否断电重启
      led_init(LED2); 
     // led(LED2,LED_ON);
      
     // set_prameter();
      //uart_getchar(UART2,&col_mid_back);
   while(middle)              
   {
     DELAY_MS(10);
     if (middle)
     {
        set_prameter();
     
     }
   }
   
 
   flash_init();//
   DELAY_MS(10);
     if(1)         //ERASE   PTE6
     {
     flash_erase_sector(SECTOR_NUM);//
     }
  
    temp[0]=(uint8)(row_edge_array[2]); 
    temp[1]=(uint8)((uint16)(row_edge_array[2]*10)%10); 
    temp[2]=(uint8)(row_edge_array[4]); 
    temp[3]=(uint8)((uint16)(row_edge_array[4]*10)%10); 
    temp[4]=(uint8)(row_edge_array[6]); 
    temp[5]=(uint8)((uint16)(row_edge_array[6]*10)%10); 
    temp[6]=(uint8)(row_edge_array[8]); 
    temp[7]=(uint8)((uint16)(row_edge_array[8]*10)%10); 
       
    
   while(!flash_write_buf(SECTOR_NUM,0,sizeof(threshold),&threshold));
   while(!flash_write_buf(SECTOR_NUM,8*1,sizeof(row_edge),&row_edge));
   while(!flash_write_buf(SECTOR_NUM,8*2,sizeof(Kp),&Kp));
   while(!flash_write_buf(SECTOR_NUM,8*3,sizeof(Ki),&Ki));
   while(!flash_write_buf(SECTOR_NUM,8*4,sizeof(col_mid1),&col_mid1));
   while(!flash_write_buf(SECTOR_NUM,8*5,sizeof(KP),&KP));
   while(!flash_write_buf(SECTOR_NUM,8*6,sizeof(KD),&KD));
   while(!flash_write_buf(SECTOR_NUM,8*7,sizeof(row_cut),&row_cut));
   while(!flash_write_buf(SECTOR_NUM,8*8,sizeof(DKPN),&DKPN));
   while(!flash_write_buf(SECTOR_NUM,8*9,sizeof(DKDN),&DKDN));
   while(!flash_write_buf(SECTOR_NUM,8*10,sizeof(speed_down),&speed_down));
   while(!flash_write_buf(SECTOR_NUM,8*11,sizeof(Kd),&Kd));  

      
   PID_Init();
    pit_init_ms(PIT1, 5);      //周期为5ms                          
    set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //设置PIT1的中断服务函数为 PIT1_IRQHandler
    enable_irq (PIT1_IRQn);     //使能PIT1中断
   //pit_init_ms(PIT0, 2);      //周期为5ms                          
  //  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT1的中断服务函数为 PIT1_IRQHandler
   // enable_irq (PIT0_IRQn);
       //UART 开中断并使能 这个地方需要计算时间 划重点
   

    
     


          
          
}



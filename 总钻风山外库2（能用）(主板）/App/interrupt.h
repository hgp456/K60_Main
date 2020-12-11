#ifndef INTERRUPT_H_
#define INTERRUPT_H_


extern void PIT1_IRQHandler();
extern void PORTE_IRQHandler();
extern void PORTD_IRQHandler();
extern void PORTA_IRQHandler();
extern void DMA0_IRQHandler();
extern void FTM1_INPUT_IRQHandler();
extern uint8 daocheflag;
extern uint8 imgbuff[CAMERA_SIZE];
extern uint8 duzhuan_light_flag;
extern void UART2_IRQhandler();
extern void UART4_IRQhandler();
extern void USART3_IRQHandler();
extern void UART5_IRQhandler();
extern void UART1_IRQhandler();
extern void PIT0_IRQHandler();
extern uint8 start_flag;
extern uint16 dis;
extern uint16 ang;
extern int16 barrier_dis;
extern uint8 TOFbyteReceived;
extern int16 barrieravg;
extern int16 last_barrieravg;
extern uint8 barrierflag;
extern  uint8 openmv_fakeflag;
extern  uint8 openmv_NT;
extern  uint8 openmv_last_col_longest;
extern  uint8 openmv_col_position;
extern  uint8 openmv_finish_flag;
extern void DMA_Error_IRQHandler(void);
extern void DMA10_IRQHandler(void);
extern void DMA_init();
extern int A_OUT,B_OUT,C_OUT,D_OUT;
extern void openmv_anas();
extern int barrieravg_pre;
extern int barrieravg_back;
extern int16 barrierdis[11];
extern    uint8 back_camera;
extern  float Vx_turn;
extern int16 barrier_dis_right;
extern int16 barrier_dis_left;
extern  int mianbiflag ;
extern  int tof_count;
extern   int barrier_mid1_flag;
extern  uint8  last_col_longest_send  ;
extern  uint8 col_position_send;
extern  uint8   fakeflag_send ;
extern   uint8    NT_send  ;
extern void UART0_IRQhandler();
extern uint8 back_barrier_state;
#endif
/********顺暗*************/
#ifndef __IIC_H_
#define __IIC_H_

#include "include.h"


#define SCL_Out         PTXn_T(PTE1,DDR) = 1      //配置输出作为SCL_Out
#define SDA_Out         PTXn_T(PTE0,DDR) = 1      //配置作为输出作为SDA_Out
#define SDA_In          PTXn_T(PTE0,IN)      //配置作为输入作为SDA_In
 
#define SCL_High        PTXn_T(PTE1,OUT) = 1      //配置输出高电平
#define SCL_Low         PTXn_T(PTE1,OUT) = 0     //配置输出低电平
#define SDA_High        PTXn_T(PTE0,OUT) = 1      //配置输出高电平
#define SDA_Low         PTXn_T(PTE0,OUT) = 0      //配置输出低电平
#define SDA_Data        PTXn_T(PTE0,IN)         //读取引脚上的引脚状态



void SCCB_Init(void);
void SCCB_Wait(void);
void SCCB_Stop(void);
void SCCB_Star(void);
uint8 SCCB_SendByte(uint8 Data);
void SCB_RegWrite(uint8 Device,uint8 Address,uint16 Data);
#endif




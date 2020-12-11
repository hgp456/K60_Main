#include "include.h"


/*************************************************************************
* 北京龙邱智能科技 K60开发板           
*
*  函数名称：void SCCB_Init(void)
*  功能说明：配置SCCB所用引脚为GPIO功能，暂时不配置数据方向
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB1_Init(void)
{
    gpio_init  (PTE0, GPO, 0); //初始化SCL
    gpio_init  (PTE1, GPO, 0); //初始化SDA
  
}

/*************************************************************************
* 北京龙邱智能科技 K60开发板           
*
*  函数名称：void SCCB_Wait(void)
*  功能说明：SCCB等待演示
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Wait(void)
{
  uint8 i=0;
  for(i=0;i<200;i++)
  { 
    asm ("nop");
  }  
}

/*************************************************************************
* 北京龙邱智能科技 K60开发板           
*
*  函数名称：void SCCB_Star(void)
*  功能说明：启动函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Star(void)
{
  SCL_Out;
  SDA_Out;
  SCCB_Wait();
  SDA_High;
  SCL_High;asm ("nop");asm ("nop");  
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_Low;asm ("nop");asm ("nop");  
  
}
/*************************************************************************
* 北京龙邱智能科技 K60开发板           
*
*  函数名称：void SCCB_Stop(void)
*  功能说明：停止函数
*  参数说明：无
*  函数返回：无
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCCB_Stop(void)
{
  SCL_Out;
  SDA_Out;
  SCCB_Wait();
  SDA_Low;
  SCCB_Wait();
  SCL_High;asm ("nop");asm ("nop");  
  SCCB_Wait();
  SDA_High;
  SCCB_Wait();
  
}
/*************************************************************************
* 北京龙邱智能科技 K60开发板           
*
*  函数名称：uint8 SCCB_SendByte(uint8 Data)
*  功能说明：SCCB发送字节函数
*  参数说明：要发送的字节
*  函数返回：应答信号
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
uint8 SCCB_SendByte(uint8 Data)
{
  uint8 i;
  uint8 Ack;
  SDA_Out;
  for( i=0; i<8; i++)
  {
    if(Data & 0x80) SDA_High;
    else            SDA_Low;    
    Data <<= 1;
    SCCB_Wait();
    SCL_High;asm ("nop");asm ("nop");      
    SCCB_Wait();
    SCL_Low;asm ("nop");asm ("nop");  
    SCCB_Wait();
  }
  SDA_High;
  SDA_In;
  SCCB_Wait();
  
  SCL_High;asm ("nop");asm ("nop");  
  SCCB_Wait();
  Ack = SDA_Data;
  SCL_Low;asm ("nop");asm ("nop");  
  SCCB_Wait();
  return Ack;
}

/*************************************************************************
* 北京龙邱智能科技 K60开发板           
*
*  函数名称：void SCB_RegWrite(uint8 Device,uint8 Address,uint16 Data)
*  功能说明：向设备写数据
*  参数说明：要发送的字节
*  函数返回：应答信号
*  修改时间：2017年12月5日
*  备    注：
*************************************************************************/
void SCB_RegWrite(uint8 Device,uint8 Address,uint16 Data)
{
  uint8 i;
  uint8 Ack;
  
  for( i=0; i<3; i++)
  {
    SCCB_Star();
    Ack = SCCB_SendByte(Device);
    if( Ack == 1 )
    {
      continue;
    }
    
    Ack = SCCB_SendByte(Address);
    if( Ack == 1 )
    {
      continue;
    }
    
    Ack = SCCB_SendByte((uint8)(Data>>8));
    Ack = SCCB_SendByte((uint8)Data);
    if( Ack == 1 )
    {
      continue;
    }
    
    SCCB_Stop();
    if( Ack == 0 ) break;
  }
}

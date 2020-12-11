#ifndef _SEEKFREE_OELD_H
#define _SEEKFREE_OELD_H

#include "include.h"

#define byte unsigned char  //自己加的
#define word unsigned int   //自己加的

//----宏定义OLED引脚----	 
#define  OLED_SCL_PIN	PTB1
#define  OLED_SDA_PIN	PTB3
#define  OLED_RST_PIN	PTB5
#define  OLED_DC_PIN	PTB7
//#define  OLED_CS_PIN	E4


void Dly_ms(uint16 ms);
void OLED_Init(void);
void OLED_Fill(uint8 dat);
void OLED_WrDat(uint8 data);
void OLED_Wr6Dat(uint8 data);
void OLED_Draw_LQLogo(void);
void OLED_Draw_LibLogo(void);
void OLED_PutPixel(uint8 x,uint8 y);
void OLED_ClrPixel(uint8 x,uint8 y);
void OLED_Set_Pos(uint8 x, uint8 y);
void LCD_CCD_SHOW(uint8 a,uint8 * CCD);
void blprint(uint8 x,uint8 y,uint8 ax);  
void OLED_BL(uint8 x, uint8 y, uint16 z);
void OLED_Print(byte x, byte y, byte ch[]);
void OLED_P14x16Str(byte x,byte y,byte ch[]);
void OLED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
void OLED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
void OLED_HEXACSII(uint16 hex,uint8* Print);
void OLED_Print_Num(uint8 x, uint8 y, uint16 num);
void OLED_Print_Num1(uint8 x, uint8 y, int16 num);
void OLED_Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]);
void dis_bmp(uint16 high, uint16 width, uint8 *p,uint8 value);



/********************************************************************/

#endif

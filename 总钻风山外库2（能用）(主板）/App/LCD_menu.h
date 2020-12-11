/*

 Send.c
 Created on 2016.3.12
 By Yuxin Zheng

*/

#ifndef LCD_MENU_H_
#define LCD_MENU_H_

#include "constant.h"

 extern Site_t site;
 extern Site_t site2;
 extern Size_t imgsize;             //图像大小
 extern Size_t size;                   //显示区域图像大
 
 extern uint8 VAR[3];


 extern void AMY_LCD_GRAY(Site_t site, Size_t size, uint8 img[][COL]);
 extern void AMY_LCD_BW(Site_t site, Size_t size, uint8 img[][COL]);
 extern void menu_display();
 extern void set_prameter();
 extern uint8 speed_down;
#endif
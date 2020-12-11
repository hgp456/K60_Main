/*

 Send.c
 Created on 2016.3.12
 By Yuxin Zheng

*/
#include "common.h"
#include "include.h"
#include "LCD_menu.h"
#include "constant.h"
#include "analysis.h"
#include "init.h"       
#include "PID.h"
  Site_t site = {0,0};
    Site_t site1 = {0,0};
    Site_t site2 = {0,70};
    Size_t imgsize  = {ROW, COL};             //图像大小
    Size_t size;                   //显示区域图像大
      //size.H =120;
      //size.W =160;
uint8 VAR[3]={20,50,70};
/*按键调参*/

   uint8 speed_down=0;
     
     //                                               65 40
      //                                      <20 20-30 30-40 40-50 50-60 60-70 70-80 80-90 90-100 >100
     //                                 0     1     2     3     4      5     6     7     8     9     10   11 //11用于滞回判断中rowarray[countflag+1]
    // float  row_edge_array[12]=   {   86.5 ,85.0 ,72.0 , 63.5 ,53.0 ,49.5 ,44.0, 42.5, 40.5, 30.0 ,29.0 ,29.0};
     uint8 speed_array[12]    =    {     0 ,  15  , 25 ,   35  ,  45  , 55  , 65  , 75  , 85  , 95   ,105  , 115 };  
     float  row_edge_array[12]=   {    86.5 ,85.5 ,RE2 , 68.2 , RE4  , 53.0 ,RE6 , 47.0, RE8 , 42.0 ,42.0,42.0};
     int16 chasu_high_array[11]=   {    0,   0,     0,    0,     30,    30,   30,   30,   35,   75,    75 };
     int16  chasu_low_array[11]=   {     0,   0,     0 ,   0,    10,    10,   10,   10,   10,   30,   30 };
     int16 chasu_highduty_array[11]={    750,  750,  750,  750,  750,  750,  430,  450,  350,  350,  400 };
     int16  chasu_lowduty_array[11]={    950,  950,  950,  950,  970,  950,  930,  900,  900,  900,  800 };
     float row_delta[12]=           {   0,    0,    0,     0,    5,   6.5,  7.5,   7.5  ,  7.5,  7.5 ,  0,   0};
  


void menu_display()
{
  
  Site_t site1; 
  
  site1.x = 80;
  site1.y = 10;
  
  LCD_str(site1, "TH", BLUE, RED);
  
  site1.x = 110;
  site1.y = 10;
  
  LCD_num(site1, threshold, BLUE, RED);
  
  site1.x = 110;
  site1.y = 30;
  
  LCD_num(site1, row_edge, BLUE, RED);
  
  site1.x = 80;
  site1.y = 30;
  
  LCD_str(site1, "REG", BLUE, RED);
  
  site1.x = 110;
  site1.y = 50;
  
  LCD_num(site1, KP, BLUE, RED);
  
  site1.x = 80;
  site1.y = 50;
  
  LCD_str(site1, "DKP", BLUE, RED);
  
   
    site1.x = 70;
  site1.y = 70;
 
  LCD_str(site1, "ZW", BLUE, RED);
  
    site1.x =110 ;
  site1.y = 70;
  
  LCD_num(site1, DKPN, BLUE, RED);
  
      site1.x = 70;
  site1.y = 90;
  
  LCD_str(site1, "Vy", BLUE, RED);
  
    site1.x = 110;
  site1.y = 90;
  
  LCD_num(site1, DKDN, BLUE, RED);
  
  
  
      site1.x = 70;
  site1.y = 110;
  LCD_str(site1, "NS", BLUE, RED);
  
   site1.x = 110;
  site1.y = 110;
  LCD_num(site1, Kd, BLUE, RED);
  
  site1.x = 40;
  site1.y = 10;
  
  LCD_num(site1, Ki, BLUE, RED);
  
  site1.x = 10;
  site1.y = 10;
  
  LCD_str(site1, "CB", BLUE, RED);
  
  
  
  
  
  site1.x = 40;
  site1.y = 30;
  
  LCD_num(site1, col_mid1, BLUE, RED);
  
  site1.x = 10;
  site1.y = 30;
  
  LCD_str(site1, "CM", BLUE, RED);
  
  
  
  
  
  site1.x = 40;
  site1.y = 50;
  
  LCD_num(site1, KD, BLUE, RED);
  
  site1.x = 10;
  site1.y = 50;
  
  LCD_str(site1, "DKD", BLUE, RED);
  
  
  
  
  site1.x = 40;
  site1.y = 70;
  
  LCD_num(site1, Kp, BLUE, RED);
  
  site1.x = 10;
  site1.y = 70;
  
  LCD_str(site1, "Kp", BLUE, RED);
  
    site1.x = 40;
  site1.y = 90;
  
  LCD_num(site1, row_cut, BLUE, RED);
  
  site1.x = 10;
  site1.y = 90;
  
  LCD_str(site1, "RC", BLUE, RED);
    
   
 /**/
    site1.x = 40;
  site1.y = 110;
  
  LCD_num(site1, speed_down, BLUE, RED);
  
  site1.x = 10;
  site1.y = 110;
  
  LCD_str(site1, "JS", BLUE, RED);
  

  
  
}

void set_prameter()
{

  static uint8 flag=1;
  
  if(down==0)              
   {
     DELAY_MS(100);
     while(down==0);
     
     flag++;
     if ( flag == 13)
       flag = 1;
 
   }
    if(up==0)              
   {
     DELAY_MS(100);
     while(up==0);
     
     flag--;
     if ( flag == 0)
       flag = 13;
   }
  
   if(right==0)              
   {
      DELAY_MS(100);
      while(right==0);
      
      switch (flag)
      {
        case 1:  menu_display();
                 threshold++;
                 VAR[0]=threshold;
             //   while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                 
                 site1.x = 110;
                 site1.y = 10;     
                 LCD_num(site1, threshold, BLUE, RED);
                 break;
                   
        case 2:  menu_display();
                 row_edge++;
                // VAR[1]= row_edge;
             //  while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                 site1.x = 110;
                 site1.y = 30;     
                 LCD_num(site1, row_edge,BLUE, RED);
                 break;
                   
        case 3: menu_display();
                 KP++;
                 VAR[2]= Kp;
             //  while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                 site1.x = 110;
                 site1.y = 50;     
                 LCD_num(site1, KP,BLUE, RED);
                 break;   
        case 4: menu_display();
                DKPN++;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 110;
                site1.y = 70;     
                LCD_num(site1,DKPN,BLUE, RED);
                break;  
        case 5: menu_display();
                DKDN++;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 110;
                site1.y = 90;     
                LCD_num(site1,DKDN,BLUE, RED);
                break;   
        
        case 6: menu_display();
                Kd++;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 110;
                site1.y = 110;     
                LCD_num(site1,Kd,BLUE, RED);
                break;   
                
        case 7: menu_display();
                Ki++;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 10;     
                LCD_num(site1, Ki,BLUE, RED);
                break;   
                 
        case 8: menu_display();
                col_mid1++;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 30;     
                LCD_num(site1,col_mid1,BLUE, RED);
                break;                  
                 
        case 9: menu_display();
                KD++;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 50;     
                LCD_num(site1,KD,BLUE, RED);
                break; 
        
        case 10: menu_display();
                Kp++;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 70;     
                LCD_num(site1,Kp,BLUE, RED);
                break;   
                
        case 11: menu_display();
                row_cut++;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 90;     
                LCD_num(site1,speed_down,BLUE, RED);
                break;
       case 12: menu_display();
                speed_down++;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 110;     
                LCD_num(site1,speed_down,BLUE, RED);
                break;         
         
             
     
        
      }
   }
  
  if(left==0)              
   {
      DELAY_MS(100);
      while(left==0);
      
      switch (flag)
      {
        case 1: menu_display();
                 threshold--;
                 VAR[0]=threshold;
            //   while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                 site1.x = 110;
                 site1.y = 10;     
                 LCD_num(site1,threshold,BLUE, RED);
                 break;
                   
        case 2: menu_display();
                 row_edge--;
              //VAR[1]=row_edge;
           //    while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                 site1.x = 110;
                 site1.y = 30;     
                 LCD_num(site1, row_edge,BLUE, RED);
                 break;
                   
        case 3: menu_display();
                 KP--;
               VAR[2]=Kp;
          //   while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)   
                 site1.x = 110;
                 site1.y = 50;     
                 LCD_num(site1, KP,BLUE, RED);
                 break;    
                 
      case 4: menu_display();
                DKPN--;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 110;
                site1.y = 70;     
                LCD_num(site1,DKPN,BLUE, RED);
                break;  
                
       case 5: menu_display();
                DKDN--;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 110;
                site1.y = 90;     
                LCD_num(site1,DKDN,BLUE, RED);
                break;  
                
      case 6: menu_display();
                Kd--;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 110;
                site1.y = 110;     
                LCD_num(site1,Kd,BLUE, RED);
                break;   
                        
        case 7: menu_display();
                Ki--;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 10;     
                LCD_num(site1,Ki,BLUE, RED);
                break;   
                 
        case 8: menu_display();
                col_mid1--;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 30;     
                LCD_num(site1,col_mid1,BLUE, RED);
                break;                  
                 
        case 9: menu_display();
                KD--;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 50;     
                LCD_num(site1,KD,BLUE, RED);
                break; 
        
        case 10: menu_display();
                Kp--;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 70;     
                LCD_num(site1,Kp,BLUE, RED);
                break;    
                
       case 11: menu_display();
                row_cut--;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 90;     
                LCD_num(site1,row_cut,BLUE, RED);
                break;  
                
       case 12: menu_display();
                speed_down--;
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 40;
                site1.y = 110;     
                LCD_num(site1,speed_down,BLUE, RED);
                break;   
     
                 
                 
      }     
   }
  
   switch (flag)
      {
        case 1: menu_display();
                 site1.x = 110;
                 site1.y = 10;     
                 LCD_num(site1,threshold,BLUE, WHITE);
                 break;
                   
        case 2: menu_display();
                 site1.x = 110;
                 site1.y = 30;     
                 LCD_num(site1, row_edge,BLUE,  WHITE);
                 break;
                   
        case 3: menu_display();
                 site1.x = 110;
                 site1.y = 50;     
                 LCD_num(site1, KP,BLUE,  WHITE);
                 break;    
        case 4: menu_display();
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 110;
                site1.y = 70;     
                LCD_num(site1,DKPN,BLUE, RED);
                break;  
        case 5: menu_display();
             // while(!flash_write_buf(SECTOR_NUM,0,sizeof(VAR),VAR));        //扇区127,偏移地址为0,写入数据源地址:buff,数目sizeof(buff)
                site1.x = 110;
                site1.y = 90;     
                LCD_num(site1,DKDN,BLUE, RED);
                break; 
         case 6: menu_display();
                site1.x = 110;
                site1.y = 110;     
                LCD_num(site1,Kd,BLUE, RED);
                break;   
                                 
        case 7: menu_display();
                site1.x = 40;
                site1.y = 10;     
                LCD_num(site1, Ki,BLUE,  WHITE);
                break;   
                 
        case 8: menu_display();
                site1.x = 40;
                site1.y = 30;     
                LCD_num(site1,col_mid1,BLUE,  WHITE);
                break;                  
                 
        case 9: menu_display();
                site1.x = 40;
                site1.y = 50;     
                LCD_num(site1,KD,BLUE, WHITE);
                break; 
        
        case 10: menu_display();
                site1.x = 40;
                site1.y = 70;     
                LCD_num(site1,Kp,BLUE,  WHITE);
                break;  
                
                 
        case 11: menu_display();
                site1.x = 40;
                site1.y = 90;     
                LCD_num(site1,row_cut,BLUE,  WHITE);
                break;   
        case 12: menu_display();

                site1.x = 40;
                site1.y = 110;     
                LCD_num(site1,speed_down,BLUE, RED);
                break;           
       
      }

}




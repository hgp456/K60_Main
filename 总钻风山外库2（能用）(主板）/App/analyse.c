#include "analysis.h"
#include "circleflag.h"
#include "common.h"
#include "include.h"
#include "constant.h"
#include "LCD_menu.h"
#include "math.h"
#include "PID.h"
#include "motor.h"
#include "init.h"
#include "interrupt.h"
#include  "stdlib.h"

#define THRESHOLD_JOINNUM 5000
#define SIZE (ROW * COL)
int32 max_length = 0;
int zhihui_flag = 0;
uint16 col_longest_num;

uint8 row_cut=5;
uint8 NT = 0; //屏幕无白点标志
uint16 col_longest_max = 0;
uint16 col_longest_min = ROW;

uint16 temp_image_size = 0;
uint16 temp_col_position = 0;
uint32 temp_row_position = 0;
uint16 image_size = 0;
uint16 col_position = 0;//uint8
float row_position = ROW;//uint8
int col_longest = 0;//uint8

char circleflag = 1;  // 0 直线 1 转弯 2 正常打角
char motorflag = 1; //0 减速 1 加速
char pre_motorflag = 1;
char servo_mode = 0; // 0 左转弯 1 右转弯 2 偏左直行 3 偏右直行
//char zhuanwan_flag = 

uint8 row_edge=90;//猜想：越大转的越早 因为其图像是从下往上位移
int totalnum=0;
uint16 uzi = 0;

//灯顺序处理
uint8 road_num = 1;
 
uint32 circle_count = 0;  //一个转弯过程有多少帧
uint32 line_count = 0;   //一个直线过程有多少帧 
uint32 line_count_count = 0;  // 连续多少个直线过程帧数都少于某一值（用于判断双闪或远灯）
uint8 slow_flag = 0;  //减速转圈标志

uint8 countflag_while_circle = 0;
uint8 speedflag_while_circle = 0;

uint8 gpioudflag = 0;

uint16 last_col_position = 0;
float row_position_store[4] = { 0 , 0 ,0,0};
uint8 nextflag = 0;
uint8 last_NT = 0;
uint16 last_image_size = 0;
uint8 fakeflag = 0;
int col_error = 0;
uint8 cutflag = 0;
uint8 zhihuiflag = 1;
int last_col_longest = 0;
uint8 test_col_flag = 0;
int size_1[2]={160,120};
uint8 Pix_Data[ROW][COL];
int last_totalnum = 0;
    uint16 i,j;
    uint16 col_max[ROW],col_min[ROW],col_len[ROW];
    uint16 temp_col_max[ROW],temp_col_min[ROW];
    uint16 row_max=0,row_min=ROW;      //灯的上下限
void set_sp(); // 原本用来用距离更换舵机设定值 此处不需要


    int last1_col_position=200;
    int last2_col_position=200;
  int last3_col_position=200;
  int last4_col_position=200;
  int last5_col_position=200;
  int last6_col_position=200;
  
void imageduel(unsigned char *image,int width, int height)  //连通域  用不了太慢了 优化不够
{
	int biaohao=0,maxl=0,bx,cx,dx,ex,i=0,j=0;
	int pimage[160][120]={0};
        for (i = 0; i<height;i++)
          for(j = 0; j<width;j++)
            pimage[i][j]=0;
	for(i=1;i<height;i++)//size[1]是行数 size[0]是列数
		for(j=1;j<width-1;j++)
		{
			if( (i==1) && (j==1) )
			{	
				pimage[i-1][j-1]=NULL;
				pimage[i-1][j]=NULL;
				pimage[i-1][j+1]=NULL;
				pimage[i][j-1]=NULL;
			}
			else if( (i==1) && (j!=1) && (j!=width-2) )
			{
				pimage[i-1][j+1]=NULL;
			}
			else if( (i==1) && (j==width-2) )
			{
				pimage[i-1][j+1]=NULL;
				pimage[i][j+1]=NULL;
			}
			else if( (i!=1) && (j==1))
			{
				pimage[i][j-1]=NULL;
			}
			else if( (i!=1) && (j==width-2) )
			{
				pimage[i][j+1]=NULL;
			}
			//到这儿为止图像四周的边框变黑
	for(i=1;i<height-1;i++)

	{

		for(j=1;j<width-1;j++)

		{
			if(*(image + i * width + j)==0) pimage[i][j]=NULL;//二维图像当作一维图像来处理
			else
			{

				bx=pimage[i][j-1];

				cx=pimage[i-1][j-1];

				dx=pimage[i-1][j];

				ex=pimage[i-1][j+1];

				

				if(bx==0&&cx==0&&dx==0&&ex==0)

				{

					biaohao+=1;

					maxl+=1;                 
					pimage[i][j]=biaohao;   
				}

				else 

				{

					int l1,l2,l3,l4;

					if(bx!=0)

					{

						l1=l2=bx;

						if(ex!=0)

							l2=ex;

					}

					else if(cx!=0)

					{

						l1=l2=cx;

						if(ex!=0)

							l2=ex;

					}

					else if(dx!=0) l1=l2=dx;

					else if(ex!=0) l1=l2=ex;

					

					if(l1==l2)

						pimage[i][j]=l1;

					

					else

					{

						l3=l1<l2?l1:l2;

						l4=l1>l2?l1:l2;

						pimage[i][j]=l3;

						for(int m=0;m<i;m++)
						{
							for(int n=0;n<j;n++)
							{

							    if(pimage[m][n]==l4) pimage[i][j]=l3;

							     maxl--;
							}
						}

					}	

				}

			}

		}

	}
	if(biaohao!=maxl)

	{

		//int *all;

		//all=(int*)malloc(sizeof(int)*(biaohao+1));
                int all[1000];
		for(i=0;i<biaohao+1;i++)

			all[i]=0;

		uint16 newbiaohao=1;

		for(i=0;i<height;i++)

		{
			for(j=0;j<width;j++)
			{

			if(pimage[i][j]!=0)

				if(all[pimage[i][j]]==0)

				{

					all[pimage[i][j]]=newbiaohao;

					newbiaohao+=1;

				}
			}

		}

		biaohao=newbiaohao-1;

		for(i=0;i<height;i++)
			for(j=0;j<width;j++)

			    pimage[i][j]=all[pimage[i][j]];

		

		free(all);

	}

//=================================储存第二轮标号=========
	int *number;

	number=(int*)malloc(sizeof(int)*(maxl+1));

	for(i=0;i<maxl+1;i++)

		number[i]=0;

	for(i=0;i<height;i++)

	{
		for(j=0;j<width;j++)
		{

		if(pimage[i][j]>0)

			number[pimage[i][j]]++;
		}

	}
	for(i=0;i<height;i++)

	{
		for(j=0;j<width;j++)
		{

		if(number[pimage[i][j]]<THRESHOLD_JOINNUM)

			/*

			

		//当白色连通域的面积小于THRESHOLD_JOINNUM时，就把整个连通域变为背景区域，即是0，这样就删除了小块的目标区域

			*/

		{

			number[pimage[i][j]]=0;			

			pimage[i][j]=0;

		}
		}

	}
	int totalnum=0;
	for (int af=0;af<maxl+1;af++)

		{

			if (number[af]!=0)

			{

				totalnum+=1;

				for (int ap=0;ap<height;ap++)

				{
					for(int bp=0;bp<width;bp++)
					{

					if (pimage[ap][bp]==af)

					{

						pimage[ap][bp]=totalnum;

					}

				}

			}

		}

		//m_pbiaohao=totalnum;

	}

	

	//释放动态申请的空间

	free(number);

	number=NULL;
                }
}
void  new_photo_deal_1(BYTE *pImgData, int width, int height)//连通域  用不了太慢了 优化不够
{
      totalnum = 0;
      int biaohao=0,maxl=0,bx,cx,dx,ex,i=0,j=0;

	int gx[20000];

	for(i=0;i<height*width;i++)      

		gx[i]=0;

	for(j=1;j<height-1;j++)

	{

		for(int i=1;i<width-1;i++)

		{

			if(pImgData[j*width+i]!=0)//白

			{

				bx=gx[j*width+i-1];

				cx=gx[(j-1)*width+i-1];

				dx=gx[(j-1)*width+i];

				ex=gx[(j-1)*width+i+1];

				

				if(bx==0&&cx==0&&dx==0&&ex==0)

				{

					biaohao+=1;

					maxl+=1;                 
					gx[j*width+i]=biaohao;   
				}

				else 

				{

					int l1,l2,l3,l4;

					if(bx!=0)

					{

						l1=l2=bx;

						if(ex!=0)

							l2=ex;

					}

					else if(cx!=0)

					{

						l1=l2=cx;

						if(ex!=0)

							l2=ex;

					}

					else if(dx!=0) l1=l2=dx;

					else if(ex!=0) l1=l2=ex;

					

					if(l1==l2)

						gx[j*width+i]=l1;

					

					else

					{

						l3=l1<l2?l1:l2;

						l4=l1>l2?l1:l2;

						gx[j*width+i]=l3;

						for(int k=0;k<j*width+i;k++)

							if(gx[k]==l4) gx[k]=l3;

							maxl--;

					}	

				}

			}

		}

	}

	

	if(biaohao!=maxl)

	{

		int *all;

		all=(int*)malloc(sizeof(int)*(biaohao+1));

		for(int i=0;i<biaohao+1;i++)

			all[i]=0;

		int newbiaohao=1;

		for(i=0;i<height*width;i++)

		{

			if(gx[i]!=0)

				if(all[gx[i]]==0)

				{

					all[gx[i]]=newbiaohao;

					newbiaohao+=1;

				}

		}

		biaohao=newbiaohao-1;

		for(i=0;i<height*width;i++)

			gx[i]=all[gx[i]];

		

		free(all);

	}

//=================================储存第二轮标号=========



	int *number;

	number=(int*)malloc(sizeof(int)*(maxl+1));

	for(i=0;i<maxl+1;i++)

		number[i]=0;

	for(i=0;i<height*width;i++)

	{

		if(gx[i]>0)

			number[gx[i]]++;

	}


	/*m_pmax=0;//先清0

	for (int aaa=0;aaa<maxl+1;aaa++)

	{

		if (number[aaa]>THRESHOLD_JOINNUM)

		{

			m_pmax+=1;//连通域个数
		}

	}

	if (m_pbiaohao>=100||m_pmax>=100)

	{

		m_pbufferout+=1;


	}



	//删除小连通域后 需要再次重新标号
*/
	
	for (int af=0;af<maxl+1;af++)

		{

			if (number[af]!=0)

			{

				totalnum+=1;

				for (int ap=0;ap<width*height;ap++)

				{

					if (gx[ap]==af)

					{

						gx[ap]=totalnum;

					}

				}

			}

		}
            
		//m_pbiaohao=totalnum;
        int *imagesize;
    int *center;
    int k;

    imagesize = (int*)malloc(sizeof(int)*(totalnum+1));
    center = (int*)malloc(sizeof(int)*(totalnum+1));

    for (i = 0; i < totalnum+1; i++)
		imagesize[i] = 0;
    for (i = 0; i < totalnum+1; i++)
		center[i] = 0;

    for (i = 0; i < height*width; i++)
	{
	    if (gx[i] > 0)
		    imagesize[gx[i]]++;
	}

    for(k=1;k<=totalnum;k++)
	{
        uint16 col_position=0;
	    uint16 row_position=0;
        uint16 image_size=0;
	    uint16 temp_col_position=0;
        uint32 temp_row_position=0;
	    uint16 col_max=0;
	    uint16 col_min=0;
	    uint16 row_max=0;
	    uint16 row_min=0;
	    int rowflag=0;
	
	    image_size=imagesize[k];
        for(i=0;i<=height;i++)
		{
	        for (j=0;j<=width;j++)
			{
			    if (gx[i*width + j-1] != k && gx[i*width + j] == k)
				    col_min = j; 
			    if (gx[i*width + j] == k && gx[i*width + j+1] != k)
				    col_max = j;
			}
		    temp_col_position+= col_max+col_min;
	        temp_row_position+= (col_max-col_min+1)*i;
		    if (col_max==0 && col_min==0 && rowflag==0)
			{
			    row_min =i;
			    rowflag = 1;
			}
		    else if (col_max==0 && col_min==0 && rowflag==1)
                    {
			row_max =i;
			col_min = 0;
			col_max = 0;
                    }
		}
        col_position= temp_col_position/ (( (row_max-row_min+1)*2 ));
        row_position= temp_row_position/image_size;
	    center[k] == row_position*width + col_position;
	}
	

	  last_totalnum = totalnum;

	//释放动态申请的空间

	free(number);

	number=NULL;

	free(center);
        
        center=NULL;



}


 

	
 



	uint8 image_threshold;
   int light;   
 uint16 count;
 uint8 row_cut;
void Photoanly()  //图像处理 中心思想是大津后的找中心点  之后进行控制
{
  count = 0;
   image_threshold = MyOSTU(COL,ROW,*image);
    if (image_threshold <  threshold)
     image_threshold = threshold;
    for(i=0;i<ROW;i++)
    {
      for(j=0;j<COL;j++)
      {
        if(image[i][j]<image_threshold || i < row_cut )
        {
          Pix_Data[i][j]=0;
      
        }
        else 
        {
          Pix_Data[i][j]=255;
        
        }
        
     }
    }
   
//    uint16 temp;
     
    
    temp_image_size=0;
    temp_col_position=0;
    temp_row_position=0;
    image_size=0;
    col_position=0;
    row_position=ROW; 
    col_longest=0;
       // imageduel( *Pix_Data,160,  120);
     for(i=0;i<ROW;i++) //
    {
      col_max[i]=0;
      col_min[i]=COL;
      col_len[i]=0;
      
      temp_col_max[i]=0;
      temp_col_min[i]=COL;
    }
     

     col_longest_max=0;
     col_longest_min=COL;
     col_longest_num=COL;
     
     //四周一圈变为黑 ？
     for(i=0;i<ROW;i++)
     {
       Pix_Data[i][0]=0;
       Pix_Data[i][COL-1]=0;
     }
     for(j=0;j<COL;j++)
     {
       Pix_Data[0][j]=0;
       Pix_Data[ROW-1][j]=0;
     }
  
    //new_photo_deal_1(*Pix_Data,160,120);

    
    
      
     //
   for(i=0;i<ROW;i++)
    {
       for(j=0;j<COL-1;j++) 
       {  
          if( ( Pix_Data[i][j+1]==255 ) && ( Pix_Data[i][j] == 0 ) )       
            temp_col_min[i]=j+1;
          else if( ( Pix_Data[i][j+1]==0 ) && ( Pix_Data[i][j] == 255) )
            temp_col_max[i]=j;
          else;
          
          
          if( col_max[i]==0 && col_min[i]==COL ) //？ 我认为是 col_max[i]=0 && col_min[i] ==COL) 即第一次进入循环 其他时候就只有那个值变大后才进入
          {
            col_max[i]=temp_col_max[i];
            col_min[i]=temp_col_min[i];
          }
          else if(((int)(temp_col_max[i]-temp_col_min[i])) > ((int)(col_max[i]-col_min[i])) ) //其实无影响 因为第一次一定会进入
          {
            col_max[i]=temp_col_max[i];
            col_min[i]=temp_col_min[i];
          }
          else;
       }
   
       //判断是否一行全黑
       if (!((col_min[i]==COL)&&(col_max[i]==0)))    //只要不是全黑 即没有变化 就进入循环
       {  
            col_len[i] = col_max[i]-col_min[i]+1;
       }
       
       else //没变化
         col_len[i] = 0;//全黑
     
    }
    
    
    //找最长行
    for(i=0;i<ROW;i++)
    {
       if (col_len[i]>col_longest)
       {
                col_longest_max=col_max[i];
                col_longest_min=col_min[i];
                col_longest =col_len[i];
                col_longest_num=i;
       }
    }
    
    //找整个灯上下限
    for(i=0;i<ROW-1;i++)

    { zhihuiflag--;  //葫芦灯交界处可能列宽度不稳，防止出现多个边界
     /* if(zhihuiflag==0)
      {
      zhihuiflag=1;
      }*/
      //col_error=col_max[i]+col_min[i]-col_max[i+1]-col_min[i+1]; //列偏差  用于解决葫芦灯
             //从无到有，上边界                              
       if( ( col_len[i+1] > 0 ) && ( col_len[i] == 0 ))      
          row_min=i+1;
       else if( ( col_len[i+1]==0 ) && ( col_len[i] > 0) )
       {   row_max=i;
//        if(row_max==ROW)
//          
//         break;
       }
       else;
           //上下行 列位置偏移大  确定葫芦灯row_max row_min
     /*  if((col_len[i+1]>1)&&(col_len[i]>1)&&(col_error>=2||col_error<=-2))
       {
        
        
        if(zhihuiflag==1)
        {  
       row_max=i;
       cutflag=1;
        zhihuiflag=3;
        }
             
       }
       if( row_min<=col_longest_num && row_max>=col_longest_num)  //该区域包含最长行，则该区域为灯  
       { cutflag=0;
         break;
       }
       //确定下一灯的row_min
       if(cutflag==1)
       {
        cutflag=0;
       row_min=i+1;
       }*/
       
    }
    
    
    //计算整个灯大小 
                                           //最长行的宽度理应一直变大，即使变小幅度也不能过大，用于判定闪烁灯和小太阳同屏
  /*  if( (row_min != ROW) && (row_max != 0)&&(col_longest-last_col_longest>-0.5*col_longest))
    { 
      testflag=0;
      fakeflag++;     
      NT=0;  //常有灯且无闪烁点  帧数
     // if(col_longest-last_col_longest>1)  //最长行宽度正常变大 
      //{
      //fakeflag=0;
      //}
    
      
      if(fakeflag>=4)    //常有灯无闪烁超过4帧
      {fakeflag=4;
      }
      last_NT=NT;                         //上一轮无灯次数
                                   //无灯标志置零
      
      for(i=row_min;i<=row_max;i++)
      {  
         temp_image_size+= col_max[i]-col_min[i]+1;
         temp_col_position+= col_max[i]+col_min[i];  //
         temp_row_position+= (col_max[i]-col_min[i]+1)*i; //
         
      }
      image_size=temp_image_size;
      col_position= temp_col_position/ (( (row_max-row_min+1)*2 ));
      last_col_position=col_position;
      last_image_size=image_size;
      row_position=(float)temp_row_position/(float)image_size;
  
    
    }
    else  //补偿无灯时的灯位置
    {

      NT++;                                       //无灯标志
      fakeflag=0;
      testflag++;
      if(NT>=NOPHOTO_EDGE)
      {
      NT=NOPHOTO_EDGE;
      nextflag=1;
      last_col_longest=0;
      }
      else
      {  
      image_size=last_image_size;  
      col_position=last_col_position;
      }

      }

     
      if(row_position<120)
     {
      if((row_position_store[1]==0)||nextflag==1)  //第一次进入或灭灯后
      {
      row_position_store[1]=row_position;
      row_position_store[0]=row_position;
      nextflag=0;
          
      }
      else                   //存储正常点的行数
      {
 
      row_position_store[1]=row_position_store[0];
      row_position_store[0]=row_position;
      
      }
     }
     else
     {
        if(NT<NOPHOTO_EDGE)
        {
            if((row_position_store[0]-row_position_store[1])<=1&&(row_position_store[0]-row_position_store[1])>=-1 )//静态
               row_position=row_position_store[0]; 
           else
            {
            if(testflag<2)
              row_position=row_position_store[0]+NT*(row_position_store[0]-row_position_store[1])/(last_NT+1);
            }
         }
     }
     
      if (col_longest-last_col_longest>-0.5*col_longest)   //last_col_longest过滤掉突然变小的col_longest
      {
      last_col_longest=col_longest;
      }*/
    

    //lcd显示
    /*Site_t site     = {0, 0};                           //显示图像左上角位置
    Size_t imgsize  = {CAMERA_W, CAMERA_H};             //图像大小
    Size_t size     = {80,60};                   //显示区域图像大小
     LCD_Img_gray_Z(site, size,(uint8_t*)Pix_Data ,imgsize);*/
   /* 
    //lcd_img_binary_z(site,size,imgbuff,imgsize,BLACK,WHITE);
      if( (row_min != ROW) && (row_max != 0)&&(col_longest-last_col_longest>-0.2*col_longest) )
    { 
      testflag=0;
      fakeflag++;                         //常有灯且无闪烁点  帧数

      if(fakeflag>=3)    //常有灯无闪烁超过4帧
      {fakeflag=3;
      }
      last_NT=NT;                         //上一轮无灯次数
      NT=0;                               //无灯标志置零

     for(i=row_min;i<=row_max;i++)
      {  
         temp_image_size+= col_max[i]-col_min[i]+1;
         temp_col_position+= col_max[i]+col_min[i];
         temp_row_position+= (col_max[i]-col_min[i]+1)*i;
         
      }
      image_size=temp_image_size;
      col_position= temp_col_position/ (( (row_max-row_min+1)*2 ));
      last_col_position=col_position;
      last_image_size=image_size;
      row_position=(float)temp_row_position/(float)image_size;
      if((row_position_store[1]==0)||nextflag==1)  //第一次进入或灭灯后
      {
      row_position_store[1]=row_position;
      row_position_store[0]=row_position;
      nextflag=0;
          
      }
      else                   //存储正常点的行数
      {
      row_position_store[1]=row_position_store[0];
      row_position_store[0]=row_position;
      
      }
      
    }
    else  //补偿无灯时的灯位置
    {
      if(col_longest-last_col_longest<=-0.2*col_longest)   //测试
      {
        testflag++;
      }
      NT++;                                       //无灯标志
      fakeflag=0;
      if(NT>=NOPHOTO_EDGE)
      {
      NT=NOPHOTO_EDGE;
      nextflag=1;
      last_col_longest=0;
      }
      else
      {  
      image_size=last_image_size;  
      col_position=last_col_position;
      if((row_position_store[0]-row_position_store[1])<=1&&(row_position_store[0]-row_position_store[1])>=-1)//静态
      {
       row_position=row_position_store[0]; 
      }
      else   //动态补偿
      { 
        row_position=row_position_store[0]; 
      //row_position=row_position_store[0]+NT*(row_position_store[0]-row_position_store[1])/(last_NT+1);
      }
      }
      
    
    
    }  
      if (col_longest-last_col_longest>-0.2*col_longest)   //last_col_longest过滤掉突然变小的col_longest
      {
      last_col_longest=col_longest;
      }
     Site_t site     = {0, 0};                           //显示图像左上角位置
    Size_t imgsize  = {188, 120};             //图像大小
    Size_t size     = {94,60};                   //显示区域图像大小
     LCD_Img_gray_Z(site, size,(uint8_t*)Pix_Data ,imgsize);
    
    
    //lcd_img_binary_z(site,size,imgbuff,imgsize,BLACK,WHITE);
    
    
   // LCD_Img_gray_Z(site, size,(uint8_t*)Pix_Data ,imgsize);
    //AMY_LCD_GRAY(site,size, Pix_Data);
    //判断是否全黑，全黑NT++

        max_length = col_longest_max-col_longest_min;
    
    
    row_max=row_max;    
    row_min=row_min;
    */
 if( (row_min != ROW) && (row_max != 0)&&(col_longest-last_col_longest>-0.2*col_longest))
    { 
    
      fakeflag++;     
      NT=0;  //常有灯且无闪烁点  帧数
     // if(col_longest-last_col_longest>1)  //最长行宽度正常变大 
      //{
      //fakeflag=0;
      //}
    
      
      if(fakeflag>=3)    //常有灯无闪烁超过4帧
      {fakeflag=3;
      }
      last_NT=NT;                         //上一轮无灯次数
                                   //无灯标志置零
      
      for(i=row_min;i<=row_max;i++)
      {  
         temp_image_size+= col_max[i]-col_min[i]+1;
         temp_col_position+= col_max[i]+col_min[i];  //
         temp_row_position+= (col_max[i]-col_min[i]+1)*i; //
         
      }
      image_size=temp_image_size;
      col_position= temp_col_position/ (( (row_max-row_min+1)*2 ));
      last_col_position=col_position;
      last_image_size=image_size;
      row_position=(float)temp_row_position/(float)image_size;
     if(row_position<120)
     {
      if((row_position_store[1]==0)||nextflag==1)  //第一次进入或灭灯后
      {
      row_position_store[1]=row_position;
      row_position_store[0]=row_position;
      nextflag=0;
          
      }
      else                   //存储正常点的行数
      {
 
      row_position_store[1]=row_position_store[0];
      row_position_store[0]=row_position;
      
      }
     }
    
    }
    else  //补偿无灯时的灯位置
    {
      if(col_longest-last_col_longest<=-0.05*col_longest)   //测试
      {
      
      }
      NT++;                                       //无灯标志
      fakeflag=0;
      if(NT>=NOPHOTO_EDGE)
      {
     // NT=NOPHOTO_EDGE;
      nextflag=1;
      last_col_longest=0;
      }
      else
      {  
      image_size=last_image_size;  
      //col_position=last_col_position;
      if((row_position_store[0]-row_position_store[1])<=1&&(row_position_store[0]-row_position_store[1])>=-1)//静态
      {
      

       row_position=row_position_store[0]; 
      }
      //else   //动态补偿
     // { 
     // row_position=row_position_store[0]+NT*(row_position_store[0]-row_position_store[1])/(last_NT+1);
    
     // }
      
     
   /*  if(fakeflag<=2)
      {
        row_position = row_position_store[0];
      }
      }*/

      
      }
    }
    if (fakeflag<=2 && row_position >=120)
    {
      row_position = row_position_store[0];
    }
     if (col_longest-last_col_longest>-0.2*col_longest)   //last_col_longest过滤掉突然变小的col_longest
      {
      last_col_longest=col_longest;
      }
     
   if(col_position == 0 && (NT < NOPHOTO_EDGE))
   {
     col_position = last_col_position;
   }
   

    //AMY_LCD_GRAY(site,size, Pix_Data);
    //判断是否全黑，全黑NT++ 
     Site_t site     = {0, 0};                           //显示图像左上角位置
    Size_t imgsize  = {188, 120};             //图像大小
    Size_t size     = {94,60};                   //显示区域图像大小
   //  LCD_Img_gray_Z(site, size,(uint8_t*)Pix_Data ,imgsize);
     LCD_Img_gray_Z(site, size,(uint8_t*)Pix_Data ,imgsize);
        max_length = col_longest_max-col_longest_min;
    
    
    row_max=row_max;
    row_min=row_min;
    
  if(last1_col_position==200&&col_position!=200&&col_position!=0) last1_col_position=col_position;  //将初始值last1和last2初始化 
  if(last2_col_position==200&&col_position!=200&&col_position!=0) last2_col_position=col_position;
  if(last3_col_position==200&&col_position!=200&&col_position!=0) last3_col_position=col_position;
  if(last4_col_position==200&&col_position!=200&&col_position!=0) last4_col_position=col_position;  
  //if(last5_col_position==200&&now_col_position!=200&&now_col_position!=0) last5_col_position=now_col_position;
  //if(last6_col_position==200&&now_col_position!=200&&now_col_position!=0) last6_col_position=now_col_position;
 
  last1_col_position=last2_col_position;
  last2_col_position=last3_col_position;
  last3_col_position=last4_col_position;
  last4_col_position=col_position;
 
    }


void barrier_cam()
{
 image_threshold = MyOSTU(COL,ROW,*image);
    if (image_threshold <  threshold)
     image_threshold = threshold;
    for(i=0;i<ROW;i++)
    {
      for(j=0;j<COL;j++)
      {
        
        if(image[i][j]>image_threshold && i>row_cut)
        {
          Pix_Data[i][j]=255;
      
        }
        else 
        {
          Pix_Data[i][j]=0;
        
        }
         Site_t site     = {0, 0};                           //显示图像左上角位置
    Size_t imgsize  = {188, 120};             //图像大小
    Size_t size     = {94,60};                   //显示区域图像大小
     LCD_Img_gray_Z(site, size,(uint8_t*)Pix_Data ,imgsize);
        max_length = col_longest_max-col_longest_min;
        mt9v032_finish_flag = 0;
        }
    }
}
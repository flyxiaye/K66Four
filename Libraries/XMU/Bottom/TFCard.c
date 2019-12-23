/*!
 * 	@file       ZS_SD_Card.c
 * 	@brief      XMU SmartCar -> Group -> Camera
 * 	@author     ZP
 * 	@version    New Car
 * 	@date       The Tenth in 2015
 */

#include "common.h"
#include "TFCard.h"
#include "ff.h"
#include "diskio.h"
#include "MK60_sdhc.h"
#include <GlobalVar.h>
#include "SEEKFREE_MT9V032.h"


FRESULT res;			/* 结果代码 */
FATFS fatfs;			/* 文件系统对象 */
FIL faddata[100];		/* 文件对象 */
unsigned int bw, br;
unsigned char filebuff[SD_DATA_NUM*4+2]={0};

unsigned int filebuffoffset=SD_DATA_NUM*4+2;				//一行存的数据量+2   	
unsigned char SD_Num[20];


void OLED_Write_String(unsigned char x,unsigned char y,signed char String[]);
void OLED_Write_Int   (unsigned char x,unsigned char y,signed int Number);

/*
 *  @brief:     SD卡初始化
 *  @param:     void
 *  @return:    void
 *  @note:      void  
 */
void sd_init(void)
{
	unsigned char *Temp_Char;
	Temp_Char=&SD_Num[0];
	*(Temp_Char+0)='0';
	*(Temp_Char+1)='0';
	*(Temp_Char+2)='0';
	*(Temp_Char+3)='0';
	*(Temp_Char+4)='.';
	*(Temp_Char+5)='t';
	*(Temp_Char+6)='x';
	*(Temp_Char+7)='t';
	*(Temp_Char+8)='\0';
	res = f_mount(0,&fatfs);  
//  OLED_Write_Int(0,0,res);
	for(int Temp_i = 0; Temp_i < SD_PICTURE_NUM; Temp_i++)
	{
		res = f_open(&faddata[Temp_i],(char const *)SD_Num, FA_CREATE_ALWAYS | FA_WRITE);     
												//创建一个名为data.txt的文件  需花较长的时间
		*(Temp_Char+3)=(Temp_i%10)+'0';
		*(Temp_Char+2)=(Temp_i/10)+'0';
	}
}





int sd_i;


/*
 *  @brief:     SD卡存Data数据
 *  @param:     unsigned char i,BYTE sdarray[]：要存的数据数组
 *  @return:    void
 *  @note:      void  
 */

char Getchar(int buff,int frac)
{
  int flag = 0;
  flag = (int)(buff/frac);
  switch(flag)
  {
  case 0x00:return'0';         break;
  case 0x01:return'1';         break;
  case 0x02:return'2'; 	break;
  case 0x03:return'3'; 	break;
  case 0x04:return'4'; 	break;
  case 0x05:return'5'; 	break;
  case 0x06:return'6'; 	break;
  case 0x07:return'7'; 	break;
  case 0x08:return'8'; 	break;
  case 0x09:return'9'; 	break;
  }
}
/*
 *  @brief:     SD卡存Data数据
 *  @param:     unsigned char i,BYTE sdarray[]：要存的数据数组
 *  @return:    void
 *  @note:      void  
 */
extern signed int picture_number;
void SD_storage_Gray(unsigned char i,unsigned char sdarray[],unsigned char LINE_NUM_MAX)
{
    unsigned int j=0;	
    static signed int save_picture = 1;
    sd_i = i;
    int storeint = 0;
    int k = 0;
    int frac = 100;

	for(j = 0; j < SD_DATA_NUM*4;j++)				//DATA_NUM  一行的数据量		
	{
           storeint = (int) *sdarray;
           frac = 100;
           for(k = 0;k<3;k++)
           {
             filebuff[j]= Getchar(storeint,frac);
             storeint = storeint%frac;
             frac /=10;
             j++;
             
           }
           filebuff[j]=' ';
           sdarray++;

	}   //循环结束  

	filebuff[j] = 0x0D;
	j++;
	filebuff[j] = 0x0A;		//行结束 换行
	
    /****************添加标志为结束,多个文件采集***************/
    /*if(save_picture < SD_PICTURE_NUM) 				//save_picture 为存的图像个数
    {
      
        res = f_write(&faddata[save_picture],filebuff,filebuffoffset, &bw);            
                                        //将文件缓冲区里的数据写入data.txt 需花较长时间
        if(i == LINE_NUM_MAX)
        {
            res = f_sync(&faddata[save_picture]);
            save_picture++; 
        }
    }*/
if(SD_SAVE_FILE_NUM == 1)
	{
		/****************添加标志为结束,多个文件采集***************/
		if(save_picture < SD_PICTURE_NUM) 				//save_picture 为存的图像个数
		{
		  
			res = f_write(&faddata[save_picture],filebuff,filebuffoffset, &bw);            
											//将文件缓冲区里的数据写入data.txt 需花较长时间
			if(i == LINE_NUM_MAX)
			{
				res = f_sync(&faddata[save_picture]);
				save_picture++; 
			}
		}	
	}
	else if(SD_SAVE_FILE_NUM == 0)
	{
		/****************添加标志为结束,单个文件采集***************/
		res = f_write(&faddata[0],filebuff,filebuffoffset, &bw);            
											//将文件缓冲区里的数据写入data.txt 需花较长时间
		if(i == LINE_NUM_MAX)
		{
			res = f_sync(&faddata[0]);
		}
	}
//picture_number=save_picture;
}




void getData(void)
{
  int i ,j;
  for(i =0;i<ROW;i++)
  {
    for(j = 0;j<COL;j++)
    {
       g_Img_All_120x188[i][j]= image[i][j];
      
    }
  }
}



void SD_Gather_Camera_Picture_120x188(void)
{
  	static signed int count = 0;
	unsigned int i = 0;
        getData();
	
	if(count < SD_PICTURE_NUM)
	{
		
		for(i = 0; i < 120; i++)
		{
			SD_storage_Gray(i,&g_Img_All_120x188[i][0],119);
		}
		count++;
                OLED_Write_Int(4,96,count);
	}
}

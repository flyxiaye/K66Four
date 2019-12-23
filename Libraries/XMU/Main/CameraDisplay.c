#include "common.h"
#include  "CameraDisplay.h"
#include <GlobalVar.h>
#include "SEEKFREE_MT9V032.h"


unsigned char image_double[75][188],threshold=120;
signed int middleline[64];
//
//  @brief: 将64*80个点转换为8*80个数据给液晶显示
//  @param: unsigned char img64x80[], unsigned char img8x80[]
//  return: void
//  @note:  void    
//
extern float image_threshold;
void Screen_64x80Transform8x80(unsigned char img64x80[], unsigned char img8x80[])
{
	unsigned char *p_img64x80;
	unsigned char *p_img8x80;
	unsigned char temp_bit = 0;
	
  	p_img64x80 = img64x80;
	for(signed int i = 0; i < 64; i++)
	{
	  	p_img8x80 = &img8x80[80*(i>>3)];
	  	temp_bit = g_Bit_Val_Up[i & 0x07];
		for(signed int j = 0; j < 80; j++)
		{
			if( *(p_img64x80) == 0)
			  	*(p_img8x80) &= (~temp_bit);
			else
			  	*(p_img8x80) |= (temp_bit);
			p_img64x80++;
			p_img8x80++;
		}
	}
}

//
//  @brief:		将输入的8*40的数组（8位）显示在液晶上
//  @param:		unsigned char arr[], unsigned position	//需要显示的数据的数组名和显示的起始点
//  @return:	void
//  @note:		在Dis_Camera_Picture里面调用
//
void Dis_Play_Data8x80(unsigned char arr[], unsigned position)
{
	signed int i = 0, j = 0;
	unsigned char *p_arr = arr;
	for(i = 0; i < 8; i++)
	{
		OLED_Set_Pos(position,i);
		for(j = 0; j < 80; j++)
		{
			OLED_Write_Data(*p_arr);
			p_arr++;
		}
	}	
}





//
//  @brief:  先将输入的一维数组转化成用于液晶显示的8*40个数据
//			再调用函数Dis_Play_Data8x40(unsigned char arr[], unsigned position)显示图像
//  @param:  unsigned char img[], unsigned char position	原始的一位数组和放置的起始位置
//  @return: void
//  @note:   显示需要用的128行的数据，远处的是0
//
void Dis_Camera_Picture128x320(unsigned char img[], unsigned char position)
{
  	unsigned char *p_img;
  
	unsigned char temp_img_64x80[64*80];
	unsigned char *p_temp_img_64x80 = temp_img_64x80;
	
	unsigned char temp_img_8x80[8*80];
	
	signed int i = 0, j = 0;

	for(i = 0; i < 64; i++)
	{

		  p_img = &img[80*i];
                  p_img+= 40*40;
			for(j = 0; j < 40; j++)
			{
			  	//高位
			  	if( (*(p_img) & 0xF0) == 0 &&  (*(p_img+40) & 0xF0) == 0 )
				{
					(*p_temp_img_64x80) = 1;	
				}
				else
				{
					(*p_temp_img_64x80) = 0;
				}
				p_temp_img_64x80++;
				//低位
				if( (*(p_img) & 0x0F) == 0 &&  (*(p_img+40) & 0x0F) == 0 )
				{
					(*p_temp_img_64x80) = 1;	
				}
				else
				{
					(*p_temp_img_64x80) = 0;
				}
				p_temp_img_64x80++;
				
				p_img++;
			}	
	}	
	Screen_64x80Transform8x80(temp_img_64x80,temp_img_8x80);
	Dis_Play_Data8x80(temp_img_8x80,position);
}

//显示二值化后的图像
void DispalyBinary(uint8 *p, uint16 position)
{
  int row = 61, col = 188;
  unsigned char BinaryImg[64][94];
  for (int i = 0; i < row; ++i)
  {
    for (int j = 0; j < 94; ++j)
    {
      if (50 >= *(p + 2 * j) || 50 >= *(p + 2 * j + 1))
      {
        BinaryImg[i][j] = 0;
      }
      else BinaryImg[i][j] = 255;
    }
    p += 188;
  }
  
  unsigned char DisOLEDImg[8 * 94] = { 0 };
  for (int i = 0; i < 8; i++)
  {
    OLED_Set_Pos(position,i);
    for (int j = 0; j < 94; ++j)
    {
      for (int k = 0; k < 8; ++k)
      {
        if (BinaryImg[i * 8 + k][j] != 0)
          DisOLEDImg[i * 94 + j] |= g_Bit_Val_Up[k];
      }
      OLED_Write_Data(DisOLEDImg[i * 94 + j]);
    }
  }
}
      
void DisPlayImage(void)
{
  DispalyBinary(image[UP_EAGE], 0);
}
      





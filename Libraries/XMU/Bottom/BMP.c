#include "ff.h"
#include "BMP.h"
#include "GlobalVar.h"

void OLED_Write_Int(unsigned char x,unsigned char y,signed int Number);
signed int Int_To_String(signed long Int_Num,signed char String[]);
FIL bmpfsrc, bmpfdst; 
FRESULT bmpres;
FATFS fs;			/* 文件系统对象 */

//灰度调色板数组                
uint8 Color_Palette[]=
{
 0,0,0,0,1,1,1,0,2,2,2,0,3,3,3,0,4,4,4,0,5,5,5,0,6,6,6,0,7,7,7,0,
8,8,8,0,9,9,9,0,10,10,10,0,11,11,11,0,12,12,12,0,13,13,13,0,14,14,14,0,15,15,15,0,
16,16,16,0,17,17,17,0,18,18,18,0,19,19,19,0,20,20,20,0,21,21,21,0,22,22,22,0,23,23,23,0,
24,24,24,0,25,25,25,0,26,26,26,0,27,27,27,0,28,28,28,0,29,29,29,0,30,30,30,0,31,31,31,0,
32,32,32,0,33,33,33,0,34,34,34,0,35,35,35,0,36,36,36,0,37,37,37,0,38,38,38,0,39,39,39,0,
40,40,40,0,41,41,41,0,42,42,42,0,43,43,43,0,44,44,44,0,45,45,45,0,46,46,46,0,47,47,47,0,
48,48,48,0,49,49,49,0,50,50,50,0,51,51,51,0,52,52,52,0,53,53,53,0,54,54,54,0,55,55,55,0,
56,56,56,0,57,57,57,0,58,58,58,0,59,59,59,0,60,60,60,0,61,61,61,0,62,62,62,0,63,63,63,0,
64,64,64,0,65,65,65,0,66,66,66,0,67,67,67,0,68,68,68,0,69,69,69,0,70,70,70,0,71,71,71,0,
72,72,72,0,73,73,73,0,74,74,74,0,75,75,75,0,76,76,76,0,77,77,77,0,78,78,78,0,79,79,79,0,
80,80,80,0,81,81,81,0,82,82,82,0,83,83,83,0,84,84,84,0,85,85,85,0,86,86,86,0,87,87,87,0,
88,88,88,0,89,89,89,0,90,90,90,0,91,91,91,0,92,92,92,0,93,93,93,0,94,94,94,0,95,95,95,0,
96,96,96,0,97,97,97,0,98,98,98,0,99,99,99,0,100,100,100,0,101,101,101,0,102,102,102,0,103,103,103,0,
104,104,104,0,105,105,105,0,106,106,106,0,107,107,107,0,108,108,108,0,109,109,109,0,110,110,110,0,111,111,111,0,
112,112,112,0,113,113,113,0,114,114,114,0,115,115,115,0,116,116,116,0,117,117,117,0,118,118,118,0,119,119,119,0,
120,120,120,0,121,121,121,0,122,122,122,0,123,123,123,0,124,124,124,0,125,125,125,0,126,126,126,0,127,127,127,0,
128,128,128,0,129,129,129,0,130,130,130,0,131,131,131,0,132,132,132,0,133,133,133,0,134,134,134,0,135,135,135,0,
136,136,136,0,137,137,137,0,138,138,138,0,139,139,139,0,140,140,140,0,141,141,141,0,142,142,142,0,143,143,143,0,
144,144,144,0,145,145,145,0,146,146,146,0,147,147,147,0,148,148,148,0,149,149,149,0,150,150,150,0,151,151,151,0,
152,152,152,0,153,153,153,0,154,154,154,0,155,155,155,0,156,156,156,0,157,157,157,0,158,158,158,0,159,159,159,0,
160,160,160,0,161,161,161,0,162,162,162,0,163,163,163,0,164,164,164,0,165,165,165,0,166,166,166,0,167,167,167,0,
168,168,168,0,169,169,169,0,170,170,170,0,171,171,171,0,172,172,172,0,173,173,173,0,174,174,174,0,175,175,175,0,
176,176,176,0,177,177,177,0,178,178,178,0,179,179,179,0,180,180,180,0,181,181,181,0,182,182,182,0,183,183,183,0,
184,184,184,0,185,185,185,0,186,186,186,0,187,187,187,0,188,188,188,0,189,189,189,0,190,190,190,0,191,191,191,0,
192,192,192,0,193,193,193,0,194,194,194,0,195,195,195,0,196,196,196,0,197,197,197,0,198,198,198,0,199,199,199,0,
200,200,200,0,201,201,201,0,202,202,202,0,203,203,203,0,204,204,204,0,205,205,205,0,206,206,206,0,207,207,207,0,
208,208,208,0,209,209,209,0,210,210,210,0,211,211,211,0,212,212,212,0,213,213,213,0,214,214,214,0,215,215,215,0,
216,216,216,0,217,217,217,0,218,218,218,0,219,219,219,0,220,220,220,0,221,221,221,0,222,222,222,0,223,223,223,0,
224,224,224,0,225,225,225,0,226,226,226,0,227,227,227,0,228,228,228,0,229,229,229,0,230,230,230,0,231,231,231,0,
232,232,232,0,233,233,233,0,234,234,234,0,235,235,235,0,236,236,236,0,237,237,237,0,238,238,238,0,239,239,239,0,
240,240,240,0,241,241,241,0,242,242,242,0,243,243,243,0,244,244,244,0,245,245,245,0,246,246,246,0,247,247,247,0,
248,248,248,0,249,249,249,0,250,250,250,0,251,251,251,0,252,252,252,0,253,253,253,0,254,254,254,0,255,255,255,0
};
int image_count = 0;            //截图计数

int SD_Gather_Gray_Picture120x188(void) //采图函数
{
  int temp_count = image_count;
  char name[20] = "0000.bmp";
  name[3] = temp_count % 10 + '0';
  temp_count /= 10;
  name[2] = temp_count % 10 + '0';
  temp_count /= 10;
  name[1] = temp_count % 10 + '0';
  temp_count /= 10;
  name[0] = temp_count % 10 + '0';
  if(0 == Screen_Shot(0,0,188,120,name))
  {
    OLED_Write_Int(6,8,image_count);
    image_count++;
    return 1;
  }
  else 
  {
    OLED_Write_Int(0,0,0);
    return 0;
  }
}

void SD_BMP_Init(void)          //SD卡初始化
{
  bmpres = f_mount(0,&fs);  
  OLED_Write_Int(0,0,bmpres);
//  int temp_count = 0;
//  while(1)
//  {
//    char name[20] = "0000.bmp";
//    name[3] = temp_count % 10 + '0';
//    temp_count /= 10;
//    name[2] = temp_count % 10 + '0';
//    temp_count /= 10;
//    name[1] = temp_count % 10 + '0';
//    temp_count /= 10;
//    name[0] = temp_count % 10 + '0';
//    if (f_unlink(name))
//      break;
//    temp_count++;
//  }
}
/**
 * @brief  设置截取BMP图片
 * @param  x ：截取区域的起点X坐标 
 * @param  y ：截取区域的起点Y坐标 
 * @param  Width ：区域宽度
 * @param  Height ：区域高度 
 * @retval 无
  *   该参数为以下值之一：
  *     @arg 0 :截图成功
  *     @arg -1 :截图失败
 */
int Screen_Shot( uint16_t x, uint16_t y, uint16_t Width, uint16_t Height, char * filename)
{
	/* bmp  文件头 54个字节 */
	unsigned char header[54] =
	{
		0x42, 0x4d, 0, 0, 0, 0, 
		0, 0, 0, 0, 0x36, 0x04,
		0, 0, 40,0, 0, 0, 
		0, 0, 0, 0, 0, 0, 
		0, 0, 1, 0, 8, 0, 
		0, 0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0, 
		0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 
		0, 0, 0
	};
	
	int i;
	int j;
	long file_size;     
	long width;
	long height;
	unsigned int mybw;
	unsigned char read_data;
	char kk[4]={0,0,0,0};
	
	uint8_t ucAlign;//
	
	
	/* 宽*高 +补充的字节 + 头部信息 */
        file_size = (long)Width * (long)Height + 54 + 1024;

	/* 文件大小 4个字节 */
	header[2] = (unsigned char)(file_size &0x000000ff);
	header[3] = (file_size >> 8) & 0x000000ff;
	header[4] = (file_size >> 16) & 0x000000ff;
	header[5] = (file_size >> 24) & 0x000000ff;
	
	/* 位图宽 4个字节 */
	width=Width;	
	header[18] = width & 0x000000ff;
	header[19] = (width >> 8) &0x000000ff;
	header[20] = (width >> 16) &0x000000ff;
	header[21] = (width >> 24) &0x000000ff;
	
	/* 位图高 4个字节 */
	height = Height;
	header[22] = height &0x000000ff;
	header[23] = (height >> 8) &0x000000ff;
	header[24] = (height >> 16) &0x000000ff;
	header[25] = (height >> 24) &0x000000ff;
		
	/* 新建一个文件 */
	bmpres = f_open( &bmpfsrc , (char*)filename, FA_CREATE_ALWAYS | FA_WRITE );
	
	/* 新建文件之后要先关闭再打开才能写入 */
	f_close(&bmpfsrc);
		
	bmpres = f_open( &bmpfsrc , (char*)filename,  FA_OPEN_EXISTING | FA_WRITE);

	if ( bmpres == FR_OK )
	{    
		/* 将预先定义好的bmp头部信息写进文件里面 */
		bmpres = f_write(&bmpfsrc, header,sizeof(unsigned char)*54, &mybw);		
		/* 将灰度调色板信息写进文件里面 */
                bmpres = f_write(&bmpfsrc, Color_Palette, sizeof(unsigned char) * 1024, &mybw);
		ucAlign = Width % 4;
		
		for(i=0; i<Height; i++)					
		{
			for(j=0; j<Width; j++)  
			{					
				read_data = image[y + Height - 1 - i][x + j];					
				bmpres = f_write(&bmpfsrc, &read_data, sizeof(unsigned char), &mybw);
			}
				
			if( ucAlign )				/* 如果不是4字节对齐 */
				bmpres = f_write ( & bmpfsrc, kk, sizeof(unsigned char) * ( ucAlign ), & mybw );

		}/* 截屏完毕 */

		f_close(&bmpfsrc); 
		
		return 0;
		
	}	
	else/* 截屏失败 */
		return -1;

}


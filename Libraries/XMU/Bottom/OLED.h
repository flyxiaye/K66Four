#ifndef _SEEKFREE_OELD_H
#define _SEEKFREE_OELD_H

#include "common.h"





/******************************************************/
/********************  生成接口    ********************/
/******************************************************/






#define  OLED_SCL_PIN	C10
#define  OLED_SDA_PIN	C11
#define  OLED_RST_PIN	C8
#define  OLED_DC_PIN	C9



#define Dot_Size 2                  // 小数点后数位



   


/*-----------------*OLED写一个字符串*-----------------*/      

void OLED_Write_String(unsigned char x,unsigned char y,signed char String[]);

/*-----------------*OLED写一个字符*-------------------*/      

void OLED_Write_Char  (unsigned char x,unsigned char y,signed char Asc); 
// 调用格式OLED_Write_Char(行，列，ASCII码)
// OLED_Write_Char(1,1,'a')在第一行，第一列开始显示a

/*-----------------*OLED写一个字符*-------------------*/      

void OLED_Write_Draw  (unsigned char x,unsigned char y,signed char Asc); 
// 调用格式OLED_Write_Draw(行，列，图形编号)
// OLED_Write_Draw(1,1,10)在第一行，第一列显示满电电池
                                            
/*----------------*OLED写一个整型数据*----------------*/      

void OLED_Write_Int   (unsigned char x,unsigned char y,signed int Number);

/*---------------*OLED写一个浮点型数据*---------------*/

void OLED_Write_Float (unsigned char x,unsigned char y,float Number);

    
extern unsigned char Draw[];


/******************************************************/
/*******************  ASCII码字库    ******************/
/******************************************************/
    
extern const unsigned char Font_8x16[1520];
extern const unsigned char gImage_xmdx[512];
extern const unsigned char gImage_xmdx1[512];
extern const unsigned char gImage_xmdx128[2048]; 

/******************************************************/
/********************  OLED函数    ********************/
/******************************************************/

#define X_WIDTH 128
#define Y_WIDTH 64

/*--------------------*OLED写数据*--------------------*/

void OLED_Write_Data(unsigned char Data);


/*--------------------*OLED写命令*--------------------*/

void OLED_Write_Command(unsigned char Cmd);


/*-------------------*整型转字符串*-------------------*/

void OLED_Set_Pos(unsigned char x,unsigned char y);
 

/*-------------------*整型转字符串*-------------------*/

void OLED_Fill(unsigned char Bmp_data);


/*---------------------*OLED清屏*---------------------*/

void OLED_Clear(void);

/*--------------------*OLED初始化*--------------------*/

void OLED_IO_Init(void);
void OLED_Init(void);        
 

/*-------------------*整型转字符串*-------------------*/

signed int Int_To_String(signed long Int_Num,signed char String[]);


/*------------------*浮点型转字符串*------------------*/

signed int Float_To_String(float Float_Num,signed char String[]);
// Dot_Size为小数点后边保留位数
// 返回字符串长度



/******************************************************/
/********************  生成接口    ********************/
/******************************************************/

/*-----------------*OLED写一个字符串*-----------------*/      

void OLED_Write_String(unsigned char x,unsigned char y,signed char String[]);


/*-----------------*OLED写一个字符*-------------------*/      

void OLED_Write_Char(unsigned char x,unsigned char y,signed char Asc); 


/*-----------------*OLED写一个字符*-------------------*/      

void OLED_Write_Draw(unsigned char x,unsigned char y,signed char Asc); 
      
              
                                               
/*----------------*OLED写一个整型数据*----------------*/      

void OLED_Write_Int(unsigned char x,unsigned char y,signed int Number);


/*---------------*OLED写一个浮点型数据*---------------*/

void OLED_Write_Float(unsigned char x,unsigned char y,float Number);

void Dis_Cursor(unsigned char Position,signed char Number);


/*---------------*OLED写一个16x16字*---------------*/
void Dis_Chinese16x16(unsigned char x,unsigned char y,unsigned char character_line);


#endif

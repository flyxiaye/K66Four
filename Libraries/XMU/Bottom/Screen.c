#include "headfile.h"
#include "Screen.h"
#include "SEEKFREE_18TFT.h"

#if _USE_LCD
#define SCREEN_DEPART 88
#define MAX_WORD 9
#define HEAD_MAX_WORD 15
#elif _USE_IPS
#define SCREEN_DEPART 128
#define MAX_WORD 14
#define HEAD_MAX_WORD 25
#else 
#define SCREEN_DEPART 72
#define MAX_WORD 7
#define HEAD_MAX_WORD 12
#endif

signed char Temp_List[128];
float Battery;
unsigned char Result[Cross / 8][Row];

union                                   // 转换结果
{
	unsigned char Paint_Buffer[Cross / 8][Row];
}_Paint_;

#define Paint_Buffer          _Paint_.Paint_Buffer



const unsigned char Change_To_Bit_2[8] =
{
	0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80
};



/******************************************************/
/********************  功能函数    ********************/
/******************************************************/

char Change_To_Bit[8] =
{
	0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01
};

/*-----------------*OLED写七位字符串*-----------------*/

void OLED_Write_String_7(unsigned char y, unsigned char x, signed char String[])
{
	unsigned char Temp_Char = 0, Temp_i = 0, Temp_j = 0, Temp_k = 0;
	//unsigned char Over_Flag=0;
	unsigned char String_Length = 0;

	while (String[String_Length] != '\0')
		String_Length++;

	if (String_Length > MAX_WORD)
		String_Length = MAX_WORD;

	while (Temp_j < MAX_WORD)
	{
		String_Length++;
		if (String_Length >= MAX_WORD + 1)
		{
#if _USE_LCD
			Temp_Char = *(String++);
			LCD_DispChar(x, y, Temp_Char);
#elif _USE_IPS
			Temp_Char = *(String++);
			IPS_DispChar(x, y, Temp_Char);
#else
			Temp_Char = String[Temp_k] - 32;
			OLED_Set_Pos(x, y);
			for (Temp_i = 0; Temp_i < 8; Temp_i++)
				OLED_Write_Data(Font_8x16[Temp_Char * 16 + Temp_i]);
			OLED_Set_Pos(x, y + 1);
			for (Temp_i = 0; Temp_i < 8; Temp_i++)
				OLED_Write_Data(Font_8x16[Temp_Char * 16 + Temp_i + 8]);
			Temp_k++;
#endif
		}
		else
		{
#if _USE_LCD
			LCD_DispChar(x, y, ' ');
#elif _USE_IPS
			IPS_DispChar(x, y, ' ');
#else
			OLED_Set_Pos(x, y);
			for (Temp_i = 0; Temp_i < 8; Temp_i++)
				OLED_Write_Data(Font_8x16[0]);
			OLED_Set_Pos(x, y + 1);
			for (Temp_i = 0; Temp_i < 8; Temp_i++)
				OLED_Write_Data(Font_8x16[0]);
#endif
		}
		x += 8;
		Temp_j++;
	}
}

/*---------------------*清空位置*---------------------*/

void Position_Clear(unsigned char Position)
{
	unsigned char Temp_i;
	unsigned char Temp_y, Temp_x;
	switch (Position)
	{
	case 1:Temp_x = 8, Temp_y = 2; break;
	case 2:Temp_x = SCREEN_DEPART, Temp_y = 2; break;
	case 3:Temp_x = 8, Temp_y = 4; break;
	case 4:Temp_x = SCREEN_DEPART, Temp_y = 4; break;
	case 5:Temp_x = 8, Temp_y = 6; break;
	case 6:Temp_x = SCREEN_DEPART, Temp_y = 6; break;
#if _USE_LCD
	case 7:Temp_x = 8, Temp_y = 8; break;
	case 8:Temp_x = SCREEN_DEPART, Temp_y = 8; break;
	case 9:Temp_x = 8, Temp_y = 10; break;
	case 10:Temp_x = SCREEN_DEPART, Temp_y = 10; break;
	case 11:Temp_x = 8, Temp_y = 12; break;
	case 12:Temp_x = SCREEN_DEPART, Temp_y = 12; break;
	case 13:Temp_x = 8, Temp_y = 14; break;
	case 14:Temp_x = SCREEN_DEPART, Temp_y = 14; break;
#elif _USE_IPS
	case 7:Temp_x = 8, Temp_y = 8; break;
	case 8:Temp_x = SCREEN_DEPART, Temp_y = 8; break;
	case 9:Temp_x = 8, Temp_y = 10; break;
	case 10:Temp_x = SCREEN_DEPART, Temp_y = 10; break;
	case 11:Temp_x = 8, Temp_y = 12; break;
	case 12:Temp_x = SCREEN_DEPART, Temp_y = 12; break;
	case 13:Temp_x = 8, Temp_y = 14; break;
	case 14:Temp_x = SCREEN_DEPART, Temp_y = 14; break;
#endif
	default:break;
	}
#if _USE_LCD
	if (Position & 0x01)
	{
		Lcd_SetRegion(Temp_x, Temp_y, Temp_x + SCREEN_DEPART - 8, Temp_y + 16);
		for (Temp_i = 0; Temp_i < 16 * (SCREEN_DEPART - 8); Temp_i++)
			LCD_WriteData_16Bit(BGCOLOR);
	}
	else
	{
		Lcd_SetRegion(Temp_x, Temp_y, Temp_x + 160 - SCREEN_DEPART, Temp_y + 16);
		for (Temp_i = 0; Temp_i < 16 * (160 - SCREEN_DEPART); Temp_i++)
			LCD_WriteData_16Bit(BGCOLOR);
	}
#elif _USE_IPS
	if (Position & 0x01)
	{
		ips_set_region(Temp_x, Temp_y, Temp_x + SCREEN_DEPART - 8, Temp_y + 16);
		for (Temp_i = 0; Temp_i < 16 * (160 - SCREEN_DEPART); Temp_i++)
			ips_writedata_16bit(IPS_BGCOLOR);
	}
	else
	{
		ips_set_region(Temp_x, Temp_y, Temp_x + 160 - SCREEN_DEPART, Temp_y + 16);
		for (Temp_i = 0; Temp_i < 16 * (160 - SCREEN_DEPART); Temp_i++)
			ips_writedata_16bit(IPS_BGCOLOR);
}
#else
	OLED_Set_Pos(Temp_x, Temp_y);
	for (Temp_i = 0; Temp_i < 56; Temp_i++)
		OLED_Write_Data(0x00);
	OLED_Set_Pos(Temp_x, Temp_y + 1);
	for (Temp_i = 0; Temp_i < 56; Temp_i++)
		OLED_Write_Data(0x00);
#endif
}

/*--------------------*屏幕头显示*--------------------*/

void Dis_Head(signed char String[])
{
	unsigned char Temp_Char = 0, Temp_i = 0, Temp_j = 0, Position_x = 8;
	unsigned char Over_Flag = 0;
	unsigned char String_Length = 0;
	unsigned char Begin = 0;

	while (String[String_Length] != '\0')
		String_Length++;

	if (String_Length > HEAD_MAX_WORD)
		Begin = 0;
	else
#if _USE_LCD
		Begin = (signed char)((18.0 - String_Length) / 2);
#elif _USE_IPS
		Begin = (signed char)((28.0 - String_Length) / 2);
#else
		Begin = (signed char)((14.0 - String_Length) / 2);
#endif

	while (Temp_j < HEAD_MAX_WORD)
	{
		if (Temp_j < Begin)
		{
#if _USE_LCD
			LCD_DispChar(Position_x, 0, ' ');
#elif _USE_IPS
			IPS_DispChar(Position_x, 0, ' ');
#else
			OLED_Set_Pos(Position_x, 0);
			for (Temp_i = 0; Temp_i < 8; Temp_i++)
				OLED_Write_Data(Font_8x16[0]);
			OLED_Set_Pos(Position_x, 1);
			for (Temp_i = 0; Temp_i < 8; Temp_i++)
				OLED_Write_Data(Font_8x16[0]);
#endif
		}
		else
		{
#if _USE_LCD
			Temp_Char = *(String++);
			if (Temp_Char == '\0')
				Over_Flag = 1;
			if (Over_Flag == 0)
				LCD_DispChar(Position_x, 0, Temp_Char);
			else
				LCD_DispChar(Position_x, 0, ' ');
#elif _USE_IPS
			Temp_Char = *(String++);
			if (Temp_Char == '\0')
				Over_Flag = 1;
			if (Over_Flag == 0)
				IPS_DispChar(Position_x, 0, Temp_Char);
			else
				IPS_DispChar(Position_x, 0, ' ');
#else
			Temp_Char = String[Temp_j - Begin] - 32;

			if (String[Temp_j - Begin] == '\0')
				Over_Flag = 1;

			if (Over_Flag == 0)
			{
				OLED_Set_Pos(Position_x, 0);
				for (Temp_i = 0; Temp_i < 8; Temp_i++)
					OLED_Write_Data(Font_8x16[Temp_Char * 16 + Temp_i]);
				OLED_Set_Pos(Position_x, 1);
				for (Temp_i = 0; Temp_i < 8; Temp_i++)
					OLED_Write_Data(Font_8x16[Temp_Char * 16 + Temp_i + 8]);
			}
			else
			{
				OLED_Set_Pos(Position_x, 0);
				for (Temp_i = 0; Temp_i < 8; Temp_i++)
					OLED_Write_Data(Font_8x16[0]);
				OLED_Set_Pos(Position_x, 1);
				for (Temp_i = 0; Temp_i < 8; Temp_i++)
					OLED_Write_Data(Font_8x16[0]);
			}
#endif
		}

		Position_x += 8;
		Temp_j++;
	}
			}

/*--------------------*显示字符串*--------------------*/

void Dis_String(unsigned char Position, signed char String[])
{
	switch (Position)
	{
	case 1:OLED_Write_String_7(2, 8, String); break;
	case 2:OLED_Write_String_7(2, SCREEN_DEPART, String); break;
	case 3:OLED_Write_String_7(4, 8, String); break;
	case 4:OLED_Write_String_7(4, SCREEN_DEPART, String); break;
	case 5:OLED_Write_String_7(6, 8, String); break;
	case 6:OLED_Write_String_7(6, SCREEN_DEPART, String); break;
#if _USE_LCD
	case 7:OLED_Write_String_7(8, 8, String); break;
	case 8:OLED_Write_String_7(8, SCREEN_DEPART, String); break;
	case 9:OLED_Write_String_7(10, 8, String); break;
	case 10:OLED_Write_String_7(10, SCREEN_DEPART, String); break;
	case 11:OLED_Write_String_7(12, 8, String); break;
	case 12:OLED_Write_String_7(12, SCREEN_DEPART, String); break;
	case 13:OLED_Write_String_7(14, 8, String); break;
	case 14:OLED_Write_String_7(14, SCREEN_DEPART, String); break;
#elif _USE_IPS
	case 7:OLED_Write_String_7(8, 8, String); break;
	case 8:OLED_Write_String_7(8, SCREEN_DEPART, String); break;
	case 9:OLED_Write_String_7(10, 8, String); break;
	case 10:OLED_Write_String_7(10, SCREEN_DEPART, String); break;
	case 11:OLED_Write_String_7(12, 8, String); break;
	case 12:OLED_Write_String_7(12, SCREEN_DEPART, String); break;
	case 13:OLED_Write_String_7(14, 8, String); break;
	case 14:OLED_Write_String_7(14, SCREEN_DEPART, String); break;
#endif
	default:break;
	}

}

/*--------------------*显示整型数*--------------------*/

void Dis_Int(unsigned char Position, signed int Number)
{
	signed char Temp_Buf[17];
	(void)Int_To_String(Number, Temp_Buf);

	Dis_String(Position, Temp_Buf);
}

/*--------------------*显示整型数*--------------------*/

void Dis_Int_Flg(unsigned char Position, signed int Number)
{
	signed char Temp_Buf[17];
	(void)Int_To_String(Number, Temp_Buf);
	switch (Position)
	{
	case 1:OLED_Write_String_7(2, 24, Temp_Buf); break;
	case 2:OLED_Write_String_7(2, 88, Temp_Buf); break;
	case 3:OLED_Write_String_7(4, 24, Temp_Buf); break;
	case 4:OLED_Write_String_7(4, 88, Temp_Buf); break;
	case 5:OLED_Write_String_7(6, 24, Temp_Buf); break;
	case 6:OLED_Write_String_7(6, 88, Temp_Buf); break;
	default:break;
	}
}

/*--------------------*显示浮点型*--------------------*/

void Dis_Float(unsigned char Position, float Number)
{
	signed char Temp_Buf[17];
	(void)Float_To_String(Number, Temp_Buf);
	Dis_String(Position, Temp_Buf);
}




/*---------------------*输入数字*---------------------*/

void Dis_Battery(void)
{
	signed char Temp_Battery;
	if (Battery > 820)
		Temp_Battery = 10;
	else
		if (Battery < 720)
			Temp_Battery = 0;
		else
			Temp_Battery = (signed char)(Battery / 10 - 72);

	OLED_Write_Draw(0, 15, Temp_Battery);
}

/*-------------------*列表更新显示*-------------------*/

void Paint_Fresh(float New_Data, float Paint_Ave, float Paint_Margin)
// 将某个数据以列表刷新的形式显示出来
{
	static unsigned char Temp_i;
	unsigned char cross;
	unsigned char Temp_Data;

	Temp_Data = (signed char)(22.0 / Paint_Margin * (New_Data - Paint_Ave) + 26);

	if (Temp_i > 120)
	{
		Temp_i = 0;
		OLED_Clear();
	}
	for (cross = 0; cross < 6; cross++)
	{
		if (Temp_Data >> 3 == cross)
		{
			OLED_Set_Pos(Temp_i, 7 - cross);
			OLED_Write_Data(Change_To_Bit[Temp_Data & 0x07]);
		}
	}
	Temp_i++;
}

/*---------------------*列表移动*---------------------*/

void List_Move(signed char New_Data)
// 对数组进行移位
{
	static signed int Temp_i;
	signed int Temp_j;
	if (Temp_i < 120)
		Temp_List[Temp_i] = New_Data;
	else
	{
		for (Temp_j = 0; Temp_j < 119; Temp_j++)
			Temp_List[Temp_j] = Temp_List[Temp_j + 1];
		Temp_List[119] = New_Data;
	}

	if (Temp_i >= 120)
		Temp_i = 120;
	else
		Temp_i++;
}
/*-------------------*显示列表图形*-------------------*/


/*-------------------*递推显示数据*-------------------*/


void Paint_List(float New_Data, float Paint_Ave, float Paint_Margin)
// 将某个数据以列表刷新的形式显示出来
{
	unsigned char Temp_Data;
	unsigned char Temp_i;
	unsigned char cross;

	Temp_Data = (signed char)(22.0 / Paint_Margin * (New_Data - Paint_Ave) + 20);
	List_Move(Temp_Data);

	for (cross = 0; cross < 6; cross++)
	{
		OLED_Set_Pos(4, 7 - cross);
		for (Temp_i = 0; Temp_i < 128; Temp_i++)
		{
			if (Temp_List[Temp_i] >> 3 == cross)
			{
				OLED_Write_Data(Change_To_Bit[Temp_List[Temp_i] & 0x07]);
			}
			else
			{
				OLED_Write_Data(0x00);
			}
		}
	}

}
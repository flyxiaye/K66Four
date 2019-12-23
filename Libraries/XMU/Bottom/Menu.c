#include  "headfile.h"
#include  "GlobalVar.h"
#include  "CameraDisplay.h"
#include  "TFCard.h"
#include  "Menu.h"
#include  "OLED.h"
#include  "keyboard.h"
#include "Bluetooth.h"
#include "Screen.h"
//#include "program.h"

#if _USE_LCD
#define LINE_MAX 8
#elif _USE_IPS
#define LINE_MAX 8
#else
#define LINE_MAX 4                                           //一页能显示的总行数 OLED为4 LCD横向为8 竖向为10
#endif

signed char g_pageNum = 1;                                   //当前所在页
signed char g_lineNum = 1;                                   //当前所在行
signed char g_pageCount = 0;
signed char g_lineCount = 0;
signed char g_pageMax = 0;								     //页面总数
signed char g_lineMax = 0;                                   //当前页变量的个数

signed char g_lineLock = 0;                                  //锁定标志

signed char g_plusFlg = 0;                                   //加法标志
signed char g_minisFlg = 0;                                  //减法标志
signed char g_imgFlg = 0;

signed char g_Rate = 0;                                      //倍率指数 方便加减
float g_addValue = 1;                                        //倍率
signed char g_picture_show = 0;
signed char g_explore_page = 0;								//调节曝光功能页

unsigned char Cursor[][16] =

{
#if _USE_LCD || _USE_IPS
	{0x00,0x00,0x00,0x02,0x04,0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x04,0x02,0x00,0x00},//>
	{0x00,0x00,0x00,0x05,0x0A,0x14,0x28,0x50,0xA0,0x50,0x28,0x14,0x0A,0x05,0x00,0x00}//》
#else
	{0x00,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,0x01,0x00},//>
	{0x10,0x20,0x40,0x90,0x20,0x40,0x80,0x00,0x10,0x08,0x04,0x12,0x09,0x04,0x02,0x01}//》
#endif
};

void FlagChange(int* flag)
{
	if (1 == *flag)
	{
		*flag = 0;
		return;
	}
	else if (0 == *flag)
	{
		*flag = 1;
		return;
	}
}

//*
//*  @brief:	光标
//*  @param:	void
//*  @return:   void
//*
void Dis_Cursor(unsigned char Position, signed char Number)
{
#if _USE_LCD
	Lcd_SetRegion(0, 16 * Position, 7, 16 * Position + 15);
	uint8 i, j;
	uint8 temp;
	for (i = 0; i < 16; i++)
	{
		//		Lcd_SetRegion(x,y+i,x+7,y+i);
		temp = Cursor[Number][i];
		for (j = 0; j < 8; j++)
		{
			if (temp & 0x01)	LCD_WriteData_16Bit(PENCOLOR);
			else			LCD_WriteData_16Bit(BGCOLOR);
			temp >>= 1;
		}
	}
#elif _USE_IPS
	ips_set_region(0, 16 * Position, 7, 16 * Position + 15);
	uint8 i, j;
	uint8 temp;
	for (i = 0; i < 16; i++)
	{
		temp = Cursor[Number][i];
		for (j = 0; j < 8; j++)
		{
			if (temp & 0x01) ips_writedata_16bit(IPS_PENCOLOR);
			else ips_writedata_16bit(IPS_BGCOLOR);
			temp >>= 1;
		}
	}
#else
	OLED_Set_Pos(0, 2 * Position);
	for (unsigned char i = 0; i < 8; i++)
		OLED_Write_Data(Cursor[Number][i]);
	OLED_Set_Pos(0, 2 * Position + 1);
	for (unsigned char i = 0; i < 8; i++)
		OLED_Write_Data(Cursor[Number][i + 8]);
#endif
}

//*
//*  @brief:		向左翻页
//*  @param:		void
//*  @return:	    void
//*
void PageNumReduce(void)
{
#if _USE_LCD
	LCD_TFT_Clear(BGCOLOR);
#elif _USE_IPS
	ips_clear(IPS_BGCOLOR);
#else
	OLED_Clear();
#endif
	g_pageNum--;
	if (g_pageNum < 1) g_pageNum = g_pageMax;
	g_lineNum = 1;
}

//*
//*  @brief:		向右翻页
//*  @param:		void
//*  @return:	    void
//*
void PageNumAdd(void)
{
#if _USE_LCD
	LCD_TFT_Clear(BGCOLOR);
#elif _USE_IPS
	ips_clear(IPS_BGCOLOR);
#else
	OLED_Clear();
#endif
	g_pageNum++;
	if (g_pageNum > g_pageMax) g_pageNum = 1;
	g_lineNum = 1;
}

//*
//*  @brief:		行数上移
//*  @param:		void
//*  @return:	    void
//*
void LineNumReduce(void)
{
	g_lineNum--;
	if (g_lineNum < 1) g_lineNum = g_lineMax;
}

//*
//*  @brief:		行数下移
//*  @param:		void
//*  @return:	    void
//*
void LineNumAdd(void)
{
	g_lineNum++;
	if (g_lineNum > g_lineMax) g_lineNum = 1;
}


void GotoGivenPage(void)
{
	OLED_Clear();

	g_pageNum = GIVEN_PAGE;
	g_lineLock = 0;
	g_lineNum = 1;
}

//*
//*  @brief:		插入一个页面
//*  @param:		需要显示的标题，当不需要显示标题时字符串写"\0"
//*  @return:	    void
//*  @note:     
//*
void Insert_Page(signed char String[])
{
	g_pageCount++;

	if (g_pageMax < g_pageCount) g_pageMax = g_pageCount;

	if (g_pageNum == g_pageCount)
	{
		g_imgFlg = 0;
		if (String != "\0")
		{
			Dis_Head(String);
		}
		g_lineMax = 0;
	}
	g_lineCount = 0;
}

//*
//*  @brief:		插入一幅图像
//*  @param:		在菜单中的位置 需要显示的标题以及图像显示的函数，当不需要显示标题时字符串写"\0"
//*  @return:	    void
//*  @note:     
//*
void Insert_Img(signed char String[], void pfunc_show(void))
{
	g_pageCount++;

	if (g_pageMax < g_pageCount) g_pageMax = g_pageCount;

	if (g_pageNum == g_pageCount)
	{
		g_imgFlg = 1;
		if (String != "\0")
		{
			Dis_Head(String);
		}
		g_lineMax = 0;
	}
	g_lineCount = 0;

	if (g_pageNum == g_pageCount)pfunc_show();
}

//*
//*  @brief:		插入字符变量
//*  @param:		在菜单中的位置 需要插入的变量名以及变量的地址
//*  @return:	    void
//*  @note:     
//*
void Insert_Char(signed char String[], signed char* Number)
{
	char tmp_line;
	if (g_pageNum == g_pageCount)
	{
		g_lineCount++;
		if (g_lineMax < g_lineCount) g_lineMax = g_lineCount;

		if (g_lineNum < LINE_MAX)
		{
			tmp_line = g_lineCount;
			Dis_String(2 * tmp_line - 1, String);
			Dis_Int(2 * tmp_line, *Number);
		}
		else
		{
			if (g_lineNum >= g_lineCount)
			{
				tmp_line = LINE_MAX - 1 + g_lineCount - g_lineNum;
				Dis_String(2 * tmp_line - 1, String);
				Dis_Int(2 * tmp_line, *Number);
			}
		}

		if (g_lineNum == g_lineCount)
		{
			if (g_plusFlg)
			{
				*Number += (int)g_addValue;
				g_plusFlg = 0;
			}
			else if (g_minisFlg)
			{
				*Number -= (int)g_addValue;
				g_minisFlg = 0;
			}
		}
	}
}

//*
//*  @brief:		插入整型变量
//*  @param:		在菜单中的位置 需要插入的变量名以及变量的地址
//*  @return:	    void
//*  @note:     
//*
void Insert_Int(signed char String[], signed int* Number)
{
	char tmp_line;
	if (g_pageNum == g_pageCount)
	{
		g_lineCount++;
		if (g_lineMax < g_lineCount) g_lineMax = g_lineCount;

		if (g_lineNum < LINE_MAX)
		{
			tmp_line = g_lineCount;
			Dis_String(2 * tmp_line - 1, String);
			Dis_Int(2 * tmp_line, *Number);
		}
		else
		{
			if (g_lineNum >= g_lineCount)
			{
				tmp_line = LINE_MAX - 1 + g_lineCount - g_lineNum;
				Dis_String(2 * tmp_line - 1, String);
				Dis_Int(2 * tmp_line, *Number);
			}
		}

		if (g_lineNum == g_lineCount)
		{
			if (g_plusFlg)
			{
				*Number += (int)g_addValue;
				g_plusFlg = 0;
			}
			else if (g_minisFlg)
			{
				*Number -= (int)g_addValue;
				g_minisFlg = 0;
			}
		}
	}
}

//*
//*  @brief:		插入浮点型变量
//*  @param:		在菜单中的位置(0~127) 需要插入的变量名以及变量的地址
//*  @return:	  void
//*
void Insert_Float(signed char String[], float* Number)
{
	char tmp_line;

	if (g_pageNum == g_pageCount)
	{
		g_lineCount++;
		if (g_lineMax < g_lineCount) g_lineMax = g_lineCount;

		if (g_lineNum < LINE_MAX)
		{
			tmp_line = g_lineCount;
			Dis_String(2 * tmp_line - 1, String);
			Dis_Float(2 * tmp_line, *Number);
		}
		else
		{
			if (g_lineNum >= g_lineCount)
			{
				tmp_line = LINE_MAX - 1 + g_lineCount - g_lineNum;
				Dis_String(2 * tmp_line - 1, String);
				Dis_Float(2 * tmp_line, *Number);
			}
		}
		if (g_lineNum == g_lineCount)
		{
			if (g_plusFlg)
			{
				*Number += g_addValue;
				g_plusFlg = 0;
			}
			else if (g_minisFlg)
			{
				*Number -= g_addValue;
				g_minisFlg = 0;
			}
		}
	}
}



//*
//*  @brief:		插入volatile浮点型变量
//*  @param:		在菜单中的位置(0~127) 需要插入的变量名以及变量的地址
//*  @return:	  void
//*
void Insert_VFloat(signed char String[], volatile float* Number)
{
	char tmp_line;

	if (g_pageNum == g_pageCount)
	{
		g_lineCount++;
		if (g_lineMax < g_lineCount) g_lineMax = g_lineCount;

		if (g_lineNum < LINE_MAX)
		{
			tmp_line = g_lineCount;
			Dis_String(2 * tmp_line - 1, String);
			Dis_Float(2 * tmp_line, *Number);
		}
		else
		{
			if (g_lineNum >= g_lineCount)
			{
				tmp_line = LINE_MAX - 1 + g_lineCount - g_lineNum;
				Dis_String(2 * tmp_line - 1, String);
				Dis_Float(2 * tmp_line, *Number);
			}
		}
		if (g_lineNum == g_lineCount)
		{
			if (g_plusFlg)
			{
				*Number += g_addValue;
				g_plusFlg = 0;
			}
			else if (g_minisFlg)
			{
				*Number -= g_addValue;
				g_minisFlg = 0;
			}
		}
	}
}

//*
//*  @brief:		主菜单
//*  @param:		void
//*  @return:	    void
//*
void Menu(void)
{
	//g_Key = Key_Check();
	Key_Function();
	if (!g_drive_flag)
	{
		if (g_explore_page)
			ExploreTime();
		else
		{
			Extra_Show();
			Main_Show();
		}
	}
}

//*
//*  @brief:		按键功能
//*  @param:		void
//*  @return:	    void
//*  @note:    
//* 
void Key_Function(void)
{
	if (!g_lineLock)
	{
		switch (g_Key)
		{
		case 1: if (g_steer_open)           //启动
		{
			FlagChange(&g_drive_flag);//开电机
			if (g_drive_flag)
			{
				g_StateMaster = 2;
				g_GetMeetingState = 0;
				g_MasterOutFlag = 0;
				CircleFlag = 0;
				CircleState = 0;
				Img_BrokenFlag = 0;
				Img_BlockFlag = 0;
				Img_RampFlag = 0;
				speed_type = 1;
				if (1 == g_camera_open)
					g_handle_open = 1;
				g_steer_open = 1;
				g_ad_flag = 1;
			}
		}
				break;
		case 2:
			if (g_pageNum == 1)
				g_steer_open = !g_steer_open;   //开舵机
			else
				LineNumReduce();                //行数上移
			break;
		case 3:
#if _USE_LCD
			lcd_init();
#elif _USE_IPS
			ips_init();
#else
			OLED_Init();               //刷新屏幕
			lcd_init();
#endif
			set_exposure_time(exp_time);  //曝光时间
			break;
		case 4:PageNumReduce();
			break;//向上翻页
		case 5:
			if (g_pageNum == 1)
				g_handle_open = !g_handle_open; //开补图
			else
				g_lineLock = 1;                 //进入数值修改
			break;
		case 6:  PageNumAdd();       break;//向下翻页
		case 7:
			if (g_steer_open)           //启动
			{
				g_StateMaster = 0;
				g_car_lanuch = 1;

			}
			break;
		case 8:
			if (1 == g_pageNum)
			{
				g_picture_show++;
				OLED_Clear();
			}
			else
				LineNumAdd();           //行数下移
			break;
		case 9:
			SD_Gather_Gray_Picture120x188();         break; //采图
		default:                     break;
		}
	}
	else
	{
		switch (g_Key)
		{
		case 1: MyFlash_Write(0);    break;    //写入FLASH
		case 2: LineNumReduce();    break;    //行数上移
		case 3: g_plusFlg = 1;      break;    //数值加
		case 4: g_Rate--;           break;    //减少倍率
		case 5: g_lineLock = 0;     break;    //退出数值修改
		case 6: g_Rate++;           break;    //增加倍率
		case 7:                     break;
		case 8: LineNumAdd();       break;    //行数下移
		case 9: g_minisFlg = 1;     break;    //数值减
		default:                    break;
		}
	}
	INTR_Keyboard_Init();
}


//*
//*  @brief:		显示倍率和光标
//*  @param:		void
//*  @return:	  void
//*
void Extra_Show(void)
{
	if (!g_imgFlg)
	{
		switch (g_lineNum)
		{
#if _USE_LCD
		case 1:
			Dis_Cursor(1, g_lineLock);
			LCD_DispString(0, 4, " ");
			LCD_DispString(0, 6, " ");
			LCD_DispString(0, 8, " ");
			LCD_DispString(0, 10, " ");
			LCD_DispString(0, 12, " ");
			LCD_DispString(0, 14, " ");
			break;
		case 2:
			Dis_Cursor(2, g_lineLock);
			LCD_DispString(0, 2, " ");
			LCD_DispString(0, 6, " ");
			LCD_DispString(0, 8, " ");
			LCD_DispString(0, 10, " ");
			LCD_DispString(0, 12, " ");
			LCD_DispString(0, 14, " ");
			break;
		case 3:
			Dis_Cursor(3, g_lineLock);
			LCD_DispString(0, 4, " ");
			LCD_DispString(0, 2, " ");
			LCD_DispString(0, 8, " ");
			LCD_DispString(0, 10, " ");
			LCD_DispString(0, 12, " ");
			LCD_DispString(0, 14, " ");
			break;
		case 4:
			Dis_Cursor(4, g_lineLock);
			LCD_DispString(0, 4, " ");
			LCD_DispString(0, 6, " ");
			LCD_DispString(0, 2, " ");
			LCD_DispString(0, 10, " ");
			LCD_DispString(0, 12, " ");
			LCD_DispString(0, 14, " ");
			break;
		case 5:
			Dis_Cursor(5, g_lineLock);
			LCD_DispString(0, 4, " ");
			LCD_DispString(0, 6, " ");
			LCD_DispString(0, 8, " ");
			LCD_DispString(0, 2, " ");
			LCD_DispString(0, 12, " ");
			LCD_DispString(0, 14, " ");
			break;
		case 6:
			Dis_Cursor(6, g_lineLock);
			LCD_DispString(0, 4, " ");
			LCD_DispString(0, 6, " ");
			LCD_DispString(0, 8, " ");
			LCD_DispString(0, 10, " ");
			LCD_DispString(0, 2, " ");
			LCD_DispString(0, 14, " ");
			break;
		default:
			Dis_Cursor(7, g_lineLock);
			LCD_DispString(0, 4, " ");
			LCD_DispString(0, 6, " ");
			LCD_DispString(0, 8, " ");
			LCD_DispString(0, 10, " ");
			LCD_DispString(0, 12, " ");
			LCD_DispString(0, 2, " ");
			break;
#elif _USE_IPS
		case 1:
			Dis_Cursor(1, g_lineLock);
			IPS_DispString(0, 4, " ");
			IPS_DispString(0, 6, " ");
			IPS_DispString(0, 8, " ");
			IPS_DispString(0, 10, " ");
			IPS_DispString(0, 12, " ");
			IPS_DispString(0, 14, " ");
			break;
		case 2:
			Dis_Cursor(2, g_lineLock);
			IPS_DispString(0, 2, " ");
			IPS_DispString(0, 6, " ");
			IPS_DispString(0, 8, " ");
			IPS_DispString(0, 10, " ");
			IPS_DispString(0, 12, " ");
			IPS_DispString(0, 14, " ");
			break;
		case 3:
			Dis_Cursor(3, g_lineLock);
			IPS_DispString(0, 4, " ");
			IPS_DispString(0, 2, " ");
			IPS_DispString(0, 8, " ");
			IPS_DispString(0, 10, " ");
			IPS_DispString(0, 12, " ");
			IPS_DispString(0, 14, " ");
			break;
		case 4:
			Dis_Cursor(4, g_lineLock);
			IPS_DispString(0, 4, " ");
			IPS_DispString(0, 6, " ");
			IPS_DispString(0, 2, " ");
			IPS_DispString(0, 10, " ");
			IPS_DispString(0, 12, " ");
			IPS_DispString(0, 14, " ");
			break;
		case 5:
			Dis_Cursor(5, g_lineLock);
			IPS_DispString(0, 4, " ");
			IPS_DispString(0, 6, " ");
			IPS_DispString(0, 8, " ");
			IPS_DispString(0, 2, " ");
			IPS_DispString(0, 12, " ");
			IPS_DispString(0, 14, " ");
			break;
		case 6:
			Dis_Cursor(6, g_lineLock);
			IPS_DispString(0, 4, " ");
			IPS_DispString(0, 6, " ");
			IPS_DispString(0, 8, " ");
			IPS_DispString(0, 10, " ");
			IPS_DispString(0, 2, " ");
			IPS_DispString(0, 14, " ");
			break;
		default:
			Dis_Cursor(7, g_lineLock);
			IPS_DispString(0, 4, " ");
			IPS_DispString(0, 6, " ");
			IPS_DispString(0, 8, " ");
			IPS_DispString(0, 10, " ");
			IPS_DispString(0, 12, " ");
			IPS_DispString(0, 2, " ");
			break;
#else
		case 1:
			Dis_Cursor(1, g_lineLock);
			OLED_Write_String(4, 0, " ");
			OLED_Write_String(6, 0, " ");
			break;
		case 2:
			OLED_Write_String(2, 0, " ");
			Dis_Cursor(2, g_lineLock);
			OLED_Write_String(6, 0, " ");
			break;
		default:
			OLED_Write_String(2, 0, " ");
			OLED_Write_String(4, 0, " ");
			Dis_Cursor(3, g_lineLock);
			break;
#endif
		}
	}
	switch (g_Rate)
	{
	case -4: g_Rate = 3;                         break;
	case -3: g_addValue = 0.001;                 break;
	case -2: g_addValue = 0.01;                  break;
	case -1: g_addValue = 0.1;                   break;
	case 0: g_addValue = 1;                      break;
	case 1: g_addValue = 10;                     break;
	case 2: g_addValue = 100;                    break;
	case 3: g_addValue = 1000;                  break;
	case 4: g_Rate = -3;                         break;
	default: g_Rate = 0;                       break;
	}

#if _USE_LCD
	if (g_Rate >= 0)                                      //显示倍率
	{
		LCD_DispChar(144, 0, ' ');
		OLED_Write_Int(0, 152, g_Rate);
	}
	else OLED_Write_Int(0, 144, g_Rate);
#elif _USE_IPS
	if (g_Rate >= 0)
	{
		IPS_DispChar(224, 0, ' ');
		OLED_Write_Int(0, 232, g_Rate);
	}
	else OLED_Write_Int(0, 224, g_Rate);
#else
	if (g_Rate >= 0)                                      //显示倍率
	{
		OLED_Write_String(0, 112, " ");
		OLED_Write_Int(0, 120, g_Rate);
	}
	else OLED_Write_Int(0, 112, g_Rate);
#endif  
}


//*
//*  @brief:		菜单主体显示 包括变量显示与图像显示
//*  @param:		void
//*  @return:	    void
//*
extern int speed_sum;
void Main_Show(void)
{
	g_pageCount = 0;

	//图像页
//	 Insert_Img("\0", DisPlayImage);
	Insert_Page("Display");
	if (g_pageNum == 1)
	{
		displayimage032_zoom(image[0], 64, 10, 70, 1);
		switch (g_picture_show)
		{
		case 0:
			g_picture_show = 1;
			break;
		case 1:        //圆环
			OLED_Write_Int(2, 8, Img_RampFlag);
			OLED_Write_Int(4, 8, Img_BrokenFlag);            //断路标志
			OLED_Write_Int(6, 8, Img_BlockFlag);			//路障标志
			OLED_Write_Int(2, 96, g_StateMaster);         //会车状态
			OLED_Write_Int(4, 96, g_StateSlave);            //左右环岛标志
			OLED_Write_Int(6, 96, CircleState);           //环岛状态标志

			break;
		case 2:
			Dis_Float(2, imu_data.pit);
			Dis_Float(4, imu_data.rol);
			Dis_Float(6, imu_data.yaw);
			Dis_Float(1, sensor.Gyro_deg.x);
			Dis_Float(3, sensor.Gyro_deg.y);
			Dis_Float(5, sensor.Gyro_deg.z);
			break;
			//           case 2:        //单车信息页
			//                 OLED_Write_Int(0,80,g_steer_type);
			//           OLED_Write_Int(0,80,Steer_P);
			//                 OLED_Write_Int(0,88,spdExp);
			////                 OLED_Write_Int(2,96,SearchUp(180,160));
			//                 OLED_Write_Int(2,80,IdentifyRamp());
			//                 OLED_Write_Int(4,80,g_ramp);
			//                 OLED_Write_Int(4,96,g_angleResult);
			//                 break;
			//           case 3:        //双车信息页
			//                 OLED_Write_Int(0,80,g_steer_type);
			//                 OLED_Write_Int(0,88,spdExp);
			//                 OLED_Write_Int(2,96,g_state);
			//                 OLED_Write_Int(2,112,g_state_opposite);
			//  //               OLED_Write_Int(4,96,g_delay_count);
			//                 OLED_Write_Int(4,96,g_distance_count);
			//                 OLED_Write_Int(6,96,IdentifyMeetingArea());
			//                 OLED_Write_Int(6,120,g_score);
			//                 break;
		case 3:      //调镜头信息页
			Dis_Int(1, ML[70]);
			Dis_Int(2, ML[60]);
			Dis_Int(3, ML[50]);
			Dis_Int(4, ML[40]);
			break;
		case 4:
			Dis_Int(1, ML[ProSpect]);
			Dis_Int(3, Error_1);
			Dis_Int(5, ErrorFlag);
                        Dis_Int(2, lCurSpeed);
                        Dis_Int(4, rCurSpeed);
			break;
		default:
			g_picture_show = 0;
			break;
		}
	}

	Insert_Page("QuickChange");
	Insert_Float("pmax", &p_max);
	Insert_Float("pmin", &p_min);
	Insert_Float("P", &Steer_P);                    //方向控制P
	Insert_Float("D", &Steer_D);
	Insert_Int("ProSpect", &ProSpect);              //摄像头前瞻
	Insert_Float("CI_P", &Steer_P_CI);
	Insert_Int("SpdRel", &spdExp1);                 //实际速度
	Insert_Int("SpdBasic", &spdBas1);
	Insert_Int("SpeedRow", &SpeedRow);
	Insert_Int("KDet", &KDet);

	Insert_Page("Obstacle");
	Insert_Char("BlockDir1", &BlockDir[0]);
	Insert_Char("BlockDir2", &BlockDir[1]);
	Insert_Char("BlockDir3", &BlockDir[2]);
	Insert_Char("BlockDir4", &BlockDir[3]);
	Insert_Char("BlockCount", &BlockCount);
	Insert_Int("g_inf", &g_inf);
	Insert_Int("stop_inf", &stop_inf);
	//Insert_Int("dist", &s1);
	Insert_Int("angle", &st);
	Insert_Int("end_angle", &st_end);
	Insert_Int("sum_dist", &sum_dist);
	Insert_Int("delay_dist", &delay_dist);
	Insert_Int("spdObst", &spdExp5);
	Insert_Int("DarkTH", &DarkThreshold);


	Insert_Page("PID_AD");
	Insert_Float("SteerPInd", &Steer_P_Ind);
	Insert_Float("SteerDInd", &Steer_D_Ind);
	Insert_Float("Pmax", &p_max_Ind);
	Insert_Float("Pmin", &p_min_Ind);
	Insert_Float("Plongroad", &p_LongRoad_Ind);
	Insert_Int("SpdBasic1", &spdBas1);				//平跑基准速度
	Insert_Int("SpdBasicInd", &spdBas3);			//电磁基准速度
	Insert_Int("SpdRelInd", &spdExp3);			//电磁基准速度
	//电感
	Insert_Page("AD");
	Insert_Float("ad_error_1", &ad_error_1);        //电磁偏差值
	Insert_Int("left", &ind_left);                  //电磁采集值 左
	Insert_Int("right", &ind_right);                //电磁采集值 右
	Insert_Int("mid", &ind_mid);					//电磁采集值 中
	Insert_Int("leftcol", &ind_leftcol);
	Insert_Int("rightcol", &ind_rightcol);
	Insert_Float("Ind_CI_TH", &Ind_CI_TH);
	Insert_Float("lnorm", &left_norm);              //电感左归一化值
	Insert_Float("rnorm", &right_norm);             //电感右归一化值
	Insert_Float("mnorm", &mid_norm);				//电感中归一化值
	Insert_Float("lcnorm", &leftcol_norm);
	Insert_Float("rcnorm", &rightcol_norm);
	//Insert_Int("ci", &ind_ci_th);
	Insert_Int("lmax", &ind_leftmax);               //电感左最大值
	Insert_Int("rmax", &ind_rightmax);              //电感右最大值
	Insert_Int("mmax", &ind_midmax);
	Insert_Int("lcmax", &ind_leftcolmax);
	Insert_Int("rcmax", &ind_rightcolmax);
	Insert_Int("lmin", &ind_leftmin);               //电感左最小值
	Insert_Int("rmin", &ind_rightmin);              //电感右最小值
	Insert_Int("g_ad_flag", &g_ad_flag);            //1：计算电磁偏差  0：采集电感值  
	Insert_Int("mul_ad_error", &mul_ad_error);      //电感error放大倍数

	Insert_Page("ImgFlag");
	Insert_Char("ImgCircle", &Img_CircleOpen);
	Insert_Char("ImgStraiBroken", &Img_StraightBrokenOpen);
	Insert_Char("ImgCurveBroken", &Img_CurveBrokenOpen);
	Insert_Char("ImgBlock", &Img_BlockOpen);
	Insert_Char("ImgRamp", &Img_RampOpen);
	Insert_Char("ImgStop", &Img_StopOpen);
	Insert_Char("ImgGrayJump", &Img_GrayJumpOpen);
	Insert_Char("BrokenMode", &BrokenMode);


	Insert_Page("Flag");
	Insert_Int("camera_open", &g_camera_open);      //1：使用摄像头 0：不使用摄像头
	Insert_Int("ind_open", &g_ind_open);            //1：使用电磁   0：不使用电磁
	Insert_Int("single_open", &g_single_open);
	//Insert_Int("ramp_open", &g_ramp_open);
	//Insert_Int("block_open", &g_block_open);		//中断路障控制函数开关

	Insert_Page("MeetFlag");
	Insert_Char("StopMode", &MeetingStopMode);
	Insert_Char("Mode", &g_MeetingMode);
	Insert_Char("Dir", &g_MeetingDir);
	Insert_Char("MasterS", &g_StateMaster);
	Insert_Char("SlaveS", &g_StateSlave);
	Insert_Char("MasterO", &g_MasterOutFlag);
	Insert_Char("SlaveO", &g_SlaveOutFlag);

	//舵机PID
	Insert_Page("PID_C");
	Insert_Float("SteerP", &Steer_P);                    //方向控制P
	Insert_Int("ProSpect", &ProSpect);              //摄像头前瞻
//         Insert_Float("Pmax",&p_max);
//         Insert_Float("Pmin",&p_min);
	Insert_Float("D", &Steer_D);                    //方向控制D
	Insert_Float("CI_P", &Steer_P_CI);
	//         Insert_Float("D1",&Steer_D1);
	//         Insert_Float("D2",&Steer_D2);
	//         Insert_Float("long_p",&long_steer_p);
	//         Insert_Float("long_d",&long_steer_d);
	//         Insert_Float("Plong",&p_long);
	//         Insert_Float("Dmax",&d_max);
	//         Insert_Int("E0",&E0);
	//         Insert_Float("P2",&Steer_P2);
	//         Insert_Float("P2",&Steer_P2);
	//         Insert_Float("D",&Steer_D);
	//         Insert_Float("D1",&Steer_D1);
	//         Insert_Float("D2",&Steer_D2);
	Insert_Int("steerduty", &SteerDuty);            //舵机占空比




	//         
	//         //期望速度，控速PID
	Insert_Page("SPEED");
	Insert_Int("SpdRel", &spdExp1);                 //实际速度
	Insert_Int("SpdBasic1", &spdBas1);				//平跑基准速度
	Insert_Int("SpdBasic2", &spdBas2);				//会车基准速度
	Insert_Int("SpdMeet", &spdExp2);				//会车速度
	Insert_Int("spdExp4", &spdExp4);				//电感速度
	Insert_Int("KDet", &KDet);
	//         Insert_Int("Small",&s_width);
	//         Insert_Int("Middle",&m_width);
	//         Insert_Int("Large",&l_width);


	//         Insert_Int("SpdMin",&spdExp2);
	//         Insert_Int("SpdCir",&spdExp3);
	//         Insert_Int("SpdMeet",&spdExp4);
	//         Insert_Int("SpdRamp",&spdExp5);
	//         Insert_Int("SpdBig",&spdExp6);         
	//         Insert_Int("SpdBack",&spdExpb);
	Insert_Float("SpeedP", &g_speed_p);             //速控P
	Insert_Float("SpeedI", &g_speed_i);             //速控I
	Insert_Float("SpeedD", &g_speed_d);             //速控D
//         Insert_Int("Lcur",&lCurSpeed);
//         Insert_Int("Rcur",&rCurSpeed);
//         Insert_Int("drif",&g_drive_flag);
	//Insert_Page("Meeting");
	//Insert_Int("dist1", &sum_distance_1);
	//Insert_Int("dist2", &sum_distance_2);
	//Insert_Int("dist3", &sum_distance_3);
	////         
	//Insert_Page("STOP");
	//Insert_Int("rount", &rount);


	Insert_Page("Camera");
	Insert_Int("exp_time", &exp_time);
	Insert_Int("HighTH", &HighThreshold);
	Insert_Int("LowTH", &LowThreshold);
	Insert_Int("BrokenTH", &BrokenThreshold);
	Insert_Int("ConMid", &ControlMid);
	Insert_Char("exp_page", &g_explore_page);
	Insert_Int("Diff", &DiffThreshold);
	Insert_Int("Light", &LightThreshold);
	//         //会车
	//         Insert_Page("meeting"); 
	//         Insert_Int("identify",&g_meeting_threshold);
	//         Insert_Int("offset",&g_meeting_offset);
	//         Insert_Float("dis_k",&g_distance_k);
	//         Insert_Int("protect",&g_protect_enable);
	//         
	//         //电感        
	Insert_Page("PWM");
	Insert_Int("LPWM", &Left_Pwm);//右后
	Insert_Int("LPWM2", &Left_Pwm2);//左前
	Insert_Int("RPWM", &Right_Pwm);//左后
	Insert_Int("RPWM2", &Right_Pwm2);//右前
//         
//         /*备用*/
////         Insert_Int("left",&ind_left);
////         Insert_Int("right",&ind_right);
////         Insert_Float("LN",&left_norm);
////         Insert_Float("RN",&right_norm);
//         
//         
//         //陀螺仪
//         Insert_Page("Angle");
//         Insert_Int("jolt",&g_gyro_jolt);
//         Insert_Float("Res",&g_angleResult);
//         Insert_Int("get",&g_gyroGet);
//         Insert_Float("gb",&g_gyroBalance);
//         Insert_Int("Judge",&g_ramp_judge);
//           
//         //定时停车
//         Insert_Page("stop") ;
//         Insert_Int("sec",&g_stop_time);
//         Insert_Int("pic240",&g_pic240);
//         Insert_Int("cir_able",&g_circle_open);
//         Insert_Int("pic_show",&g_picture_show);
//          //方向
	Insert_Page("0000");
	Insert_Char("CirIndOpen", &Ind_CircleOpen);
	Insert_Char("CirDir1", &CircleDir[0]);
	Insert_Char("CirDir2", &CircleDir[1]);
	Insert_Char("CirDir3", &CircleDir[2]);
	Insert_Char("CirDir4", &CircleDir[3]);
	Insert_Char("CirDir5", &CircleDir[4]);
	Insert_Char("CirDir6", &CircleDir[5]);
	//Insert_Int("BIG",&big_circle);        
	//Insert_Int("meet",&circle_meet_cnt);
	//Insert_Int("MAX",&circle_max);
	//Insert_Int("1or0",&g_JudgeValue);  
//         



}

//*
//*  @brief:		摄像头调节曝光时间
//*  @param:		void
//*  @return:	    void
//*
void ExploreTime(void)
{
	static unsigned char DispPicture = 0;	//显示整幅图像
	static unsigned char DispEage = 1;		//显示边缘
	static signed char AddMul = 0;
	static unsigned char AddValue = 1;
	CannyEage();
	//displayimage032(image[0]);
	displayimage032_zoom(image[0], 64, 10, 70, 0);
	if (DispEage)
	{
		unsigned char* p = ImageEage[0];
		for (int i = 10; i <= 70; i++)
		{
			for (int j = 0; j < 160; j++)
			{
				unsigned char temp = *(p + i * ROW / 120 * COL + j * (COL - 1) / (160 - 1));//读取像素点
				if (HIGH_TH == temp)
				{
					Lcd_SetRegion(j, i + 54, j, i + 54);		//坐标设置
					LCD_WriteData_16Bit(BLUE);
				}
			}
		}
	}
	//display
	Dis_String(1, "exp_time"); Dis_Int(2, exp_time);
	Dis_String(3, "ave_gray"); Dis_Int(4, LightThreshold);
	OLED_Write_Int(0, 152, AddMul);
	if (0 == AddMul) AddValue = 1;
	else if (1 == AddMul) AddValue = 10;
	else if (2 == AddMul) AddValue = 100;
	switch (g_Key)
	{
	case 1:
		MyFlash_Write(0);
		break;
	case 3:
		exp_time += AddValue;
		break;
	case 4:
		AddMul--;
		if (AddMul < 0)AddMul = 2;
		break;
	case 6:
		AddMul++;
		if (AddMul > 2) AddMul = 0;
		break;
	case 5:
		set_exposure_time(exp_time);
		break;
	case 8:
		g_explore_page = 0;
		g_pageNum = 1;
		g_lineNum = 1;
		OLED_Clear();
		break;
	case 9:
		exp_time -= AddValue;
		break;
	default:
		break;
	}
	INTR_Keyboard_Init();
}
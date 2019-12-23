/*!
 * 	@file       ZS_DFlash.c
 * 	@brief      XMU SmartCar -> Group -> Camera
 * 	@author     WP
 * 	@version    New Car
 * 	@date       The Tenth in 2015
 */

#include<stdio.h>
#include<stdlib.h>

#include "headfile.h"
#include "DFlash.h"
#include "MK60_flash.h"
#include "MK60_savedata.h"
#include  "OLED.h"
#include <GlobalVar.h>
#include "common.h"



#define First_Time_Init 35


 /********** ����Ϊ�洢������ **********/
#define DFLASH_PARAM1       Steer_P           //���P             
#define DFLASH_PARAM2       ProSpect  // ǰհ
#define DFLASH_PARAM3       spdExp1		//�ٶ�
#define DFLASH_PARAM4       KDet         //����
#define DFLASH_PARAM5       stop_inf	//�ߵ�ֹͣ����
#define DFLASH_PARAM6       sum_dist    //ǰ������
#define DFLASH_PARAM7       st   //ƫ�ƽǶ�
#define DFLASH_PARAM8       delay_dist	//ʶ���ӳپ���
#define DFLASH_PARAM9       spdExp2		
#define DFLASH_PARAM10      Steer_D	//���D
#define DFLASH_PARAM11      Steer_P_CI //����P
#define DFLASH_PARAM12      spdExp3		//�ᳵ�ٶ�
#define DFLASH_PARAM13      ind_ci_th	//�������жϾ�����ֵ
#define DFLASH_PARAM14      exp_time	//����ͷ�ع�ʱ��
#define DFLASH_PARAM15      g_ramp_open  //�µ�
#define DFLASH_PARAM16      g_block_open	//·��
#define DFLASH_PARAM17      g_single_open	//����ģʽ
#define DFLASH_PARAM18      g_speed_p	//�ٿ�p
#define DFLASH_PARAM19      g_speed_i	//�ٿ�i
#define DFLASH_PARAM20      HighThreshold	//canny����ֵ
#define DFLASH_PARAM21      LowThreshold	//canny����ֵ
#define DFLASH_PARAM22      ControlMid      //����ͷ��ֵ
#define DFLASH_PARAM23      Img_CircleOpen
#define DFLASH_PARAM24      Img_StraightBrokenOpen
#define DFLASH_PARAM25      Img_StopOpen
#define DFLASH_PARAM26      Img_BlockOpen
#define DFLASH_PARAM27      Img_RampOpen
#define DFLASH_PARAM28      g_speed_d
#define DFLASH_PARAM29      Ind_CircleOpen
#define DFLASH_PARAM30      spdBas1			//��׼�ٶ�
#define DFLASH_PARAM31      spdBas2			//�ᳵ��׼�ٶ�
#define DFLASH_PARAM32      Img_CurveBrokenOpen
#define DFLASH_PARAM33		Steer_P_Ind
#define DFLASH_PARAM34		Steer_D_Ind
#define DFLASH_PARAM35		p_max
#define DFLASH_PARAM36		p_min
#define DFLASH_PARAM37		p_max_Ind
#define DFLASH_PARAM38		p_min_Ind
#define DFLASH_PARAM39		p_LongRoad_Ind
#define DFLASH_PARAM40		spdBas3			//��Ż�׼�ٶ�
#define DFLASH_PARAM41		spdExp5
#define DFLASH_PARAM42		DarkThreshold
#define DFLASH_PARAM43		CircleDir[0]
#define DFLASH_PARAM44		CircleDir[1]
#define DFLASH_PARAM45		Ind_CI_TH
#define DFLASH_PARAM46		g_MeetingMode
#define DFLASH_PARAM47		g_MeetingDir
#define DFLASH_PARAM48		BlockDir[0]
#define DFLASH_PARAM49		BlockDir[1]
#define DFLASH_PARAM50		BlockDir[2]
#define DFLASH_PARAM51		BlockDir[3]
#define DFLASH_PARAM52		CircleDir[2]
#define DFLASH_PARAM53		CircleDir[3]
#define DFLASH_PARAM54		Img_GrayJumpOpen
#define DFLASH_PARAM55		BrokenThreshold
#define DFLASH_PARAM56		st_end
#define DFLASH_PARAM57		BlockCount
#define DFLASH_PARAM58		BrokenMode
#define DFLASH_PARAM59		MeetingStopMode





/********** ����Ϊ�洢������ **********/

//*************************************************************************//
//--------------------------------�洢����---------------------------------//
//*************************************************************************//
void MyFlash_Write(signed int flashnum)
{
	DisableInterrupts;
	FLASH_EraseSector(SECTOR_NUM + flashnum);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 0, First_Time_Init);



	/********** ����Ϊ�洢������ **********/
	DFlash_Write_Float(SECTOR_NUM + flashnum, 1, DFLASH_PARAM1);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 2, DFLASH_PARAM2);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 3, DFLASH_PARAM3);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 4, DFLASH_PARAM4);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 5, DFLASH_PARAM5);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 6, DFLASH_PARAM6);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 7, DFLASH_PARAM7);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 8, DFLASH_PARAM8);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 9, DFLASH_PARAM9);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 10, DFLASH_PARAM10);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 11, DFLASH_PARAM11);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 12, DFLASH_PARAM12);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 13, DFLASH_PARAM13);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 14, DFLASH_PARAM14);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 15, DFLASH_PARAM15);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 16, DFLASH_PARAM16);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 17, DFLASH_PARAM17);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 18, DFLASH_PARAM18);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 19, DFLASH_PARAM19);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 20, DFLASH_PARAM20);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 21, DFLASH_PARAM21);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 22, DFLASH_PARAM22);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 23, DFLASH_PARAM23);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 24, DFLASH_PARAM24);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 25, DFLASH_PARAM25);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 26, DFLASH_PARAM26);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 27, DFLASH_PARAM27);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 28, DFLASH_PARAM28);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 29, DFLASH_PARAM29);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 30, DFLASH_PARAM30);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 31, DFLASH_PARAM31);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 32, DFLASH_PARAM32);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 33, DFLASH_PARAM33);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 34, DFLASH_PARAM34);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 35, DFLASH_PARAM35);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 36, DFLASH_PARAM36);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 37, DFLASH_PARAM37);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 38, DFLASH_PARAM38);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 39, DFLASH_PARAM39);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 40, DFLASH_PARAM40);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 41, DFLASH_PARAM41);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 42, DFLASH_PARAM42);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 43, DFLASH_PARAM43);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 44, DFLASH_PARAM44);
	DFlash_Write_Float(SECTOR_NUM + flashnum, 45, DFLASH_PARAM45);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 46, DFLASH_PARAM46);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 47, DFLASH_PARAM47);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 48, DFLASH_PARAM48);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 49, DFLASH_PARAM49);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 50, DFLASH_PARAM50);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 51, DFLASH_PARAM51);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 52, DFLASH_PARAM52);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 53, DFLASH_PARAM53);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 54, DFLASH_PARAM54);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 55, DFLASH_PARAM55);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 56, DFLASH_PARAM56);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 57, DFLASH_PARAM57);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 58, DFLASH_PARAM58);
	DFlash_Write_Int(SECTOR_NUM + flashnum, 59, DFLASH_PARAM59);

	/********** ����Ϊ�洢������ **********/
#ifdef _USE_LCD
	LCD_DispChar(0, 0, '*');
#else
	OLED_Write_Char(0, 0, '*');
#endif
}
void MyFlash_Read(signed int flashnum)
{
	int FirstStart = 0;
	FirstStart = DFlash_Read_Int(SECTOR_NUM + flashnum, 0);
	if (FirstStart == First_Time_Init)
	{
		/********** ����Ϊ�洢������ **********/




		DFLASH_PARAM1 = DFlash_Read_Float(SECTOR_NUM + flashnum, 1);
		DFLASH_PARAM2 = DFlash_Read_Int(SECTOR_NUM + flashnum, 2);
		DFLASH_PARAM3 = DFlash_Read_Int(SECTOR_NUM + flashnum, 3);
		DFLASH_PARAM4 = DFlash_Read_Int(SECTOR_NUM + flashnum, 4);
		DFLASH_PARAM5 = DFlash_Read_Float(SECTOR_NUM + flashnum, 5);
		DFLASH_PARAM6 = DFlash_Read_Int(SECTOR_NUM + flashnum, 6);
		DFLASH_PARAM7 = DFlash_Read_Int(SECTOR_NUM + flashnum, 7);
		DFLASH_PARAM8 = DFlash_Read_Int(SECTOR_NUM + flashnum, 8);
		DFLASH_PARAM9 = DFlash_Read_Int(SECTOR_NUM + flashnum, 9);
		DFLASH_PARAM10 = DFlash_Read_Float(SECTOR_NUM + flashnum, 10);
		DFLASH_PARAM11 = DFlash_Read_Float(SECTOR_NUM + flashnum, 11);
		DFLASH_PARAM12 = DFlash_Read_Int(SECTOR_NUM + flashnum, 12);
		DFLASH_PARAM13 = DFlash_Read_Int(SECTOR_NUM + flashnum, 13);
		DFLASH_PARAM14 = DFlash_Read_Int(SECTOR_NUM + flashnum, 14);
		DFLASH_PARAM15 = DFlash_Read_Int(SECTOR_NUM + flashnum, 15);
		DFLASH_PARAM16 = DFlash_Read_Int(SECTOR_NUM + flashnum, 16);
		DFLASH_PARAM17 = DFlash_Read_Int(SECTOR_NUM + flashnum, 17);
		DFLASH_PARAM18 = DFlash_Read_Float(SECTOR_NUM + flashnum, 18);
		DFLASH_PARAM19 = DFlash_Read_Float(SECTOR_NUM + flashnum, 19);
		DFLASH_PARAM20 = DFlash_Read_Int(SECTOR_NUM + flashnum, 20);
		DFLASH_PARAM21 = DFlash_Read_Int(SECTOR_NUM + flashnum, 21);
		DFLASH_PARAM22 = DFlash_Read_Int(SECTOR_NUM + flashnum, 22);
		DFLASH_PARAM23 = DFlash_Read_Int(SECTOR_NUM + flashnum, 23);
		DFLASH_PARAM24 = DFlash_Read_Int(SECTOR_NUM + flashnum, 24);
		DFLASH_PARAM25 = DFlash_Read_Int(SECTOR_NUM + flashnum, 25);
		DFLASH_PARAM26 = DFlash_Read_Int(SECTOR_NUM + flashnum, 26);
		DFLASH_PARAM27 = DFlash_Read_Int(SECTOR_NUM + flashnum, 27);
		DFLASH_PARAM28 = DFlash_Read_Int(SECTOR_NUM + flashnum, 28);
		DFLASH_PARAM29 = DFlash_Read_Int(SECTOR_NUM + flashnum, 29);
		DFLASH_PARAM30 = DFlash_Read_Int(SECTOR_NUM + flashnum, 30);
		DFLASH_PARAM31 = DFlash_Read_Int(SECTOR_NUM + flashnum, 31);
		DFLASH_PARAM32 = DFlash_Read_Int(SECTOR_NUM + flashnum, 32);
		DFLASH_PARAM33 = DFlash_Read_Float(SECTOR_NUM + flashnum, 33);
		DFLASH_PARAM34 = DFlash_Read_Float(SECTOR_NUM + flashnum, 34);
		DFLASH_PARAM35 = DFlash_Read_Float(SECTOR_NUM + flashnum, 35);
		DFLASH_PARAM36 = DFlash_Read_Float(SECTOR_NUM + flashnum, 36);
		DFLASH_PARAM37 = DFlash_Read_Float(SECTOR_NUM + flashnum, 37);
		DFLASH_PARAM38 = DFlash_Read_Float(SECTOR_NUM + flashnum, 38);
		DFLASH_PARAM39 = DFlash_Read_Float(SECTOR_NUM + flashnum, 39);
		DFLASH_PARAM40 = DFlash_Read_Int(SECTOR_NUM + flashnum, 40);
		DFLASH_PARAM41 = DFlash_Read_Int(SECTOR_NUM + flashnum, 41);
		DFLASH_PARAM42 = DFlash_Read_Int(SECTOR_NUM + flashnum, 42);
		DFLASH_PARAM43 = DFlash_Read_Int(SECTOR_NUM + flashnum, 43);
		DFLASH_PARAM44 = DFlash_Read_Int(SECTOR_NUM + flashnum, 44);
		DFLASH_PARAM45 = DFlash_Read_Float(SECTOR_NUM + flashnum, 45);
		DFLASH_PARAM46 = DFlash_Read_Int(SECTOR_NUM + flashnum, 46);
		DFLASH_PARAM47 = DFlash_Read_Int(SECTOR_NUM + flashnum, 47);
		DFLASH_PARAM48 = DFlash_Read_Int(SECTOR_NUM + flashnum, 48);
		DFLASH_PARAM49 = DFlash_Read_Int(SECTOR_NUM + flashnum, 49);
		DFLASH_PARAM50 = DFlash_Read_Int(SECTOR_NUM + flashnum, 50);
		DFLASH_PARAM51 = DFlash_Read_Int(SECTOR_NUM + flashnum, 51);
		DFLASH_PARAM52 = DFlash_Read_Int(SECTOR_NUM + flashnum, 52);
		DFLASH_PARAM53 = DFlash_Read_Int(SECTOR_NUM + flashnum, 53);
		DFLASH_PARAM54 = DFlash_Read_Int(SECTOR_NUM + flashnum, 54);
		DFLASH_PARAM55 = DFlash_Read_Int(SECTOR_NUM + flashnum, 55);
		DFLASH_PARAM56 = DFlash_Read_Int(SECTOR_NUM + flashnum, 56);
		DFLASH_PARAM57 = DFlash_Read_Int(SECTOR_NUM + flashnum, 57);
		DFLASH_PARAM58 = DFlash_Read_Int(SECTOR_NUM + flashnum, 58);
		DFLASH_PARAM59 = DFlash_Read_Int(SECTOR_NUM + flashnum, 59);



		/********** ����Ϊ�洢������ **********/

	}

}
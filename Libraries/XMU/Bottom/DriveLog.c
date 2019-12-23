#include "ff.h"
//#include "BMP.h"
#include "GlobalVar.h"
#include "DriveLog.h"
#include "OLED.h"

FIL logfsrc, logfdst;
FRESULT logres;
//FATFS fs;			/* 文件系统对象 */

#define LOG_DATA1 curSpeed
#define FILE_NAME "DrvieLog.txt"
unsigned int logbw;

void LogInit(void)
{
	//新建一个文件
	logres = f_open(&logfsrc, FILE_NAME, FA_CREATE_ALWAYS | FA_WRITE);
	if (FR_OK != logres) return;
	logres = f_close(&logfsrc);
}

void LogGetData(void)
{
	SaveFloat(LOG_DATA1);
}

void SaveInt(int data)
{
	logres = f_open(&logfsrc, FILE_NAME, FA_OPEN_EXISTING | FA_WRITE);
	if (FR_OK != logres) return;
	signed char Temp_Buf[17];
	(void)Float_To_Int(data, Temp_Buf);
	logres = f_lseek(&logfsrc, logfsrc.fptr + logfsrc.fsize);
	logres = f_write(&logfsrc, Temp_Buf, sizeof(Temp_Buf) - 1, &logbw);
	//logres = f_write(&logfsrc, " ", sizeof(unsigned char), &logbw);
	logres = f_close(&logfsrc);
}

void SaveFloat(float data)
{
	logres = f_open(&logfsrc, FILE_NAME, FA_OPEN_EXISTING | FA_WRITE);
	if (FR_OK != logres) return;
	signed char Temp_Buf[17];
	(void)Float_To_String(data, Temp_Buf);
	logres = f_lseek(&logfsrc, logfsrc.fptr + logfsrc.fsize);
	logres = f_write(&logfsrc, Temp_Buf, sizeof(Temp_Buf) - 1, &logbw);
	//logres = f_write(&logfsrc, " ", sizeof(unsigned char), &logbw);
	logres = f_close(&logfsrc);
}

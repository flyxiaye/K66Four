#include "GlobalVar.h"

#define ROW 51
#define COL 188
//int HighThreshold = 50;
//int LowThreshold = HighThreshold * 0.4;



//统计梯度直方图
int GetGradHist(unsigned char *p_image, unsigned char *hist)
{
	//获取梯度最值 
	unsigned char maxv = 0;
	unsigned char hist_size = 255;
	unsigned char *p = p_image;
	for (int i = 0; i < ROW * COL; i++)
	{
		if (*p > maxv) maxv = *p;
		p++;
	}
	hist_size = MIN(maxv, hist_size);
	//建立直方图
	p = p_image;
	for (int i = 0; i < ROW * COL; i++)
	{
		(*(hist + *(p++)))++;
	}
	return hist_size + 1;
}
//计算高低门限
void GetDoubleThreshold(unsigned char *hist, int hist_size, int *high, int *low)
{
	double dPercentOfPixelsNotEdges = 0.25;
	int total = (int)(ROW * COL * dPercentOfPixelsNotEdges);
	int sum = 0;
	int i;
	for (i = 0; i < hist_size; i++)
	{
		sum += *hist++;
		if (sum > total)
			break;
	}
	*high = i + 1;
	*low = *high * 0.4;
}


//高斯滤波
unsigned char Gauss(int row, int col)
{
	int sum = 0;
	unsigned char *p = image[0] + IMG_COL * (row - 1) + IMG_COL - 1;
	sum += *p++;sum += *p;sum += *p++;sum += *p++;
	p += (IMG_COL - 3);
	sum += *p;sum += *p++;sum += *p;sum += *p;sum += *p;sum += *p++;
	sum += *p;sum += *p++;
	p += (IMG_COL - 3);
	sum += *p++;sum += *p;sum += *p++;sum += *p;
	return sum >> 4;
}

//边缘检测算子
int GetEageThreshold(int row, int col)
{
	//row = MAX(row, UP_EAGE);
	//row = MIN(row, DOWN_EAGE);
	//IMG_COL = MAX(IMG_COL, LEFT_EAGE + 1);
	//IMG_COL = MIN(IMG_COL, RIGHT_EAGE - 1);
	//unsigned char *p = image[0] + (row - 1) * IMG_COL + IMG_COL - 1;
	//signed int sum_1 = 0, sum_2 = 0, sum_3 = 0, sum_4 = 0;
	////水平方向
	//sum_1 += *p;sum_2 -= *p++;sum_1 += *p;sum_1 += *p++;sum_1 += *p;sum_2 += *p++;
	//p += (IMG_COL - 3);
	//sum_2 -= *p; sum_2 -= *p++;sum_2 += *p;sum_2 += *p++;
	//p += (IMG_COL - 3);
	//sum_1 -= *p; sum_2 -= *p++;sum_1 -= *p;sum_1 -= *p++;sum_1 -= *p;sum_2 += *p++;
	////斜线方向
	//p = image[0] + (row - 1) * IMG_COL + IMG_COL - 1;
	//sum_4 += *p;sum_4 += *p++;sum_3 += *p;sum_4 += *p++;sum_3 += *p;sum_3 += *p++;
	//p += (IMG_COL - 3);
	//sum_3 -= *p;sum_4 += *p++;sum_3 += *p;sum_4 -= *p++;
	//p += (IMG_COL - 3);
	//sum_3 -= *p;sum_3 -= *p++;sum_3 -= *p;sum_4 -= *p++;sum_4 -= *p;sum_4 -= *p++;
	//if (sum_1 < 0) sum_1 = -sum_1;
	//if (sum_2 < 0) sum_2 = -sum_2;
	//if (sum_3 < 0) sum_3 = -sum_3;
	//if (sum_4 < 0) sum_4 = -sum_4;
	////return sum_1 + sum_2 + sum_3 + sum_4;
	//return sqrt(sum_1 * sum_1 + sum_2 * sum_2 + sum_3 * sum_3 + sum_4 * sum_4);

	col = MAX(col, LEFT_EAGE + 1);
	unsigned char *p = image[0] + (row - 1) * IMG_COL + col - 1;
	signed int sum_1 = 0, sum_2 = 0;
	sum_1 += *p++;sum_2 += *p++;
	p += (IMG_COL - 2);
	sum_2 -= *p++;sum_1 -= *p++;
	if (sum_1 < 0)sum_1 = -sum_1;
	if (sum_2 < 0)sum_2 = -sum_2;
	return sum_1 + sum_2;
}



//双阈值检测
void DoubleTh(unsigned char *source)
{
	for (int i = UP_EAGE; i <= DOWN_EAGE; i++)
	{
		unsigned char *p = source + i * IMG_COL;
		for (int j = 0; j < IMG_COL; j++)
		{
			if (*p > HighThreshold)
				*p = HIGH_TH;
			else if (*p > LowThreshold)
				*p = LOW_TH;
			else *p = NO;
			p++;
		}
	}
}

//判断边缘连接
int IsStrongEage(unsigned char *p)
{
	p -= (1 + IMG_COL);
	int ack = 0;
	ack += HIGH_TH == *p++;
	ack += HIGH_TH == *p++;
	ack += HIGH_TH == *p++;
	p += (IMG_COL - 3);
	ack += HIGH_TH == *p++;
	ack += HIGH_TH == *p++;
	ack += HIGH_TH == *p++;
	p += (IMG_COL - 3);
	ack += HIGH_TH == *p++;
	ack += HIGH_TH == *p++;
	ack += HIGH_TH == *p++;
	return ack;
}

//抑制孤立弱边缘
void PressEage(unsigned char *source)
{
	for (int i = UP_EAGE; i <= DOWN_EAGE; i++)
	{
		unsigned char *p = (source + IMG_COL * i + 1);
		for (int j = 1; j < IMG_COL - 1; j++)
		{
			if (LOW_TH == *p)
			{
				if (IsStrongEage(p))
					*p = HIGH_TH;
			}
			p++;
		}
	}
}




//统计某一行的灰度平均值
int GetAveGray(int row)
{
	unsigned char *p = image[row];
	int sum = 0;
	for (int i = 0; i < IMG_COL; i++)
	{
		sum += *p++;
	}
	return sum / 188;

}

void CannyEage(void)
{
	for (int i = UP_EAGE; i <= DOWN_EAGE; i++)
	{
		for (int j = LEFT_EAGE; j <= RIGHT_EAGE; j++)
		{
			ImageEage[i][j] = GetEageThreshold(i, j);
		}
	}
	//unsigned char GradHist[255] = { 0 };
	//for (int i = 0; i < 255; i++)
	//{
	//	GradHist[i] = 0;
	//}
	//int HistSize = GetGradHist(ImageEage[UP_EAGE], GradHist);
	//GetDoubleThreshold(GradHist, HistSize, &HighThreshold, &LowThreshold);
	//string.Format("\r\n High = %d \r\n", HighThreshold); PrintDebug(string);
	//string.Format("\r\n Low = %d \r\n", LowThreshold); PrintDebug(string);
	LightThreshold = GetAveGray(DOWN_EAGE);
	DoubleTh(ImageEage[0]);
	PressEage(ImageEage[0]);

}

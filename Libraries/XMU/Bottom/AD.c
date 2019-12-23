#include "GlobalVar.h"
#include "headfile.h"
#include "AD.h"

#define MAXIND 4000
#define MININD 20
int protect_flag = 0;
void ind_acq(void)                         //��вɼ�
{
	//ind_left = collect(ADC0_SE9);
	//ind_right = collect(ADC0_SE8);
	//ind_mid = collect(ADC0_SE17);
	//�ɼ����
//	ind_left = collect(ADC0_SE8);
//	ind_right = collect(ADC0_SE13);
//	ind_mid = collect(ADC0_SE18);
//	ind_leftcol = ad_ave(ADC0_SE9, ADC_12bit, 10);
//	ind_rightcol = ad_ave(ADC0_SE12, ADC_12bit, 10);
  	//�ɼ����
	ind_left = collect(ADC0_SE8);
	ind_right = collect(ADC0_SE17);
	ind_mid = collect(ADC0_SE9);
	ind_leftcol = ad_ave(ADC0_SE18, ADC_12bit, 10);
	ind_rightcol = ad_ave(ADC0_SE12, ADC_12bit, 10);
	//���ֵ�޷�
	ind_left = MIN(ind_left, MAXIND);
	ind_left = MAX(ind_left, MININD);
	ind_right = MIN(ind_right, MAXIND);
	ind_right = MAX(ind_right, MININD);
	ind_mid = MIN(ind_mid, MAXIND);
	ind_mid = MAX(ind_mid, MININD);
	ind_leftcol = MIN(ind_leftcol, MAXIND);
	ind_leftcol = MAX(ind_leftcol, MININD);
	ind_rightcol = MIN(ind_rightcol, MAXIND);
	ind_rightcol = MAX(ind_rightcol, MININD);
}

void ind_norm_maxmin(void)                //���ҵ�������С
{
	ind_leftmax = MAX(ind_leftmax, ind_left);
	ind_leftmin = MIN(ind_leftmin, ind_left);
	ind_rightmax = MAX(ind_rightmax, ind_right);
	ind_rightmin = MIN(ind_rightmin, ind_right);
	ind_midmax = MAX(ind_midmax, ind_mid);
	ind_midmin = MIN(ind_midmin, ind_mid);
	ind_leftcolmax = MAX(ind_leftcolmax, ind_leftcol);
	ind_leftcolmin = MIN(ind_leftcolmin, ind_leftcol);
	ind_rightcolmax = MAX(ind_rightcolmax, ind_rightcol);
	ind_rightcolmin = MIN(ind_rightcolmin, ind_rightcol);
}
void ind_protect(void)
{
	static int count = 0;
	if (ind_left < 100 && ind_right < 100 && ind_mid < 100 && !Img_BlockFlag && 2 != Img_BrokenFlag && !g_GetMeetingFlag
		&& g_StateMaster < StateGo)
	{
		speed_type = 0;
		protect_flag = 1;
	}
	if (protect_flag)
	{
		g_drive_flag = 0;
		count++;
	}
	if (count >= 500)
	{
		count = 0;
		protect_flag = 0;

	}
}
void ind_norm(void)                           //�ó���һֵ��
{
	left_norm = (float)((ind_left - ind_leftmin)) / (float)((ind_leftmax - ind_leftmin));
	right_norm = (float)((ind_right - ind_rightmin)) / (float)((ind_rightmax - ind_rightmin));
	mid_norm = (float)((ind_mid - ind_midmin)) / (float)((ind_midmax - ind_midmin));
	leftcol_norm = (float)(ind_leftcol - ind_leftcolmin) / (float)(ind_leftcolmax - ind_leftcolmin);
	rightcol_norm = (float)(ind_rightcol - ind_rightcolmin) / (float)(ind_rightcolmax - ind_rightcolmin);
}

void get_ind_error(void)  //��л�ȡErrorֵ note:�����ж� �����п� ��һֱ�ɼ����ֵ ����error��
{
	if (g_ad_flag == 1)
	{
		ind_acq();
		ind_norm();
		ad_error_1 = (left_norm - right_norm) / (left_norm + right_norm);
		//ind_protect();
	}
	else if (0 == g_ad_flag)
	{
		ind_acq();
		ind_norm_maxmin();
		ind_norm();
		//ind_protect();

	}
}

//================================================================//
//  @brief  :		���ʶ�������
//  @param  :		void
//  @return :		1�ǻ��� 0�ǻ���
//  @note   :		�������Ϊ0������ �������Ϊ1����ǿ
//================================================================//
unsigned char IndJudgeCircle(unsigned char type)
{
#if CI_IND
	if (type)
	{

	}
	else
	{
		if (mid_norm > 1.6f || leftcol_norm > 0.6f || rightcol_norm > 0.6f) return 1;
		else return 0;
	}
#else
	return 1;
#endif
}
//================================================================//
//  @brief  :		���ʶ���뻷��
//  @param  :		void
//  @return :		1�ǻ��� 0�ǻ���
//  @note   :		�������Ϊ0������ �������Ϊ1����ǿ
//================================================================//
unsigned char IndJudgeIntoCircle(unsigned char type)
{
#if CI_IND
	if (type)
	{

	}
	else
	{
		if (mid_norm > 1.3f) return 1;
		else return 0;
	}
#else
	if (mid_norm > Ind_CI_TH && (leftcol_norm > 0.15 || rightcol_norm > 0.15)) return 1;
	else return 0;
#endif
}
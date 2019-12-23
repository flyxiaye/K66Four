#include "headfile.h"
#include "AD.h"

//================================================================//
//  @brief  :		延距清环岛7状态标志位
//  @param  :		void
//  @return :		void
//  @note   :		放在中断
//================================================================//
void Dist_ClearIslandSeven(void)
{
	static int sum_dist = ONE_METER;
	static int acc_dist = 0;
	if (7 == CircleState)
	{
		acc_dist += curSpeed;
		if (acc_dist > sum_dist)
		{
			Dist_ClearSevenFlag = 1;
		}
		else Dist_ClearSevenFlag = 0;
	}
	else
	{
		Dist_ClearSevenFlag = 0;
		acc_dist = 0;
	}
}

//================================================================//
//  @brief  :		角速度积分跳环岛6状态
//  @param  :		void
//  @return :		void
//  @note   :		放在中断
//================================================================//
void Angle_GoIslandSix(void)
{
	static float acc_angle = 0;
	if (CL == CircleFlag && 6 > CircleState)
	{
		acc_angle += sensor.Gyro_deg.z * 0.002;
		if (acc_angle > 310)
		{
			if (CircleState < 6) CircleState = 6;
			acc_angle = 0;
		}
		else if (acc_angle < 280)
		{
			if (CircleState > 5) CircleState = 5;
		}
	}
	else if (CR == CircleFlag && 6 > CircleState)
	{
		acc_angle += sensor.Gyro_deg.z * 0.002;
		if (acc_angle < -310)
		{
			if (CircleState < 6) CircleState = 6;
			acc_angle = 0;
		}
		else if (acc_angle > -280)
		{
			if (CircleState > 5) CircleState = 5;
		}
	}
	else acc_angle = 0;
}



//================================================================//
//  @brief  :		延距清除错误环岛标志
//  @param  :		void
//  @return :		void
//  @note   :		放在中断
//================================================================//
void DistCI(void)
{
	static int CI_sum_dist = 0.6 * ONE_METER;
	static int CI_acc_dist = 0;
	static int state = 0;
	switch (state)
	{
	case 0:
		if (CircleFlag && 7 != CircleState)
			state = 1;
		break;
	case 1:
		CI_acc_dist += curSpeed;
		if (CI_acc_dist > CI_sum_dist)
		{
			state = 0;
			CircleFlag = 0;
			CircleState = 0;
			CI_acc_dist = 0;
		}
		else if (IndJudgeIntoCircle(0))
		{
			if (CircleState > 3) CircleState = 3;
			state = 2;
			CI_acc_dist = 0;
		}
		break;
	case 2:
		if (7 == CircleState)
		{
			state = 0;
		}
		break;
	default:
		break;
	}
}

//================================================================//
//  @brief  :		电磁判断入环岛
//  @param  :		void
//  @return :		void
//  @note   :		放在中断
//================================================================//
void IndCI(void)
{
	static int acc_speed = 0;
	static int mid_normflag = 0, col_normflag = 0;
	static unsigned char circle_count = 0;
	if (!g_drive_flag)circle_count = 0;
	if (!CircleFlag && !Img_BlockFlag)
	{
		acc_speed += curSpeed;
		if (mid_norm > Ind_CI_TH && (leftcol_norm > 0.15 || rightcol_norm > 0.15) && !CircleFlag && !col_normflag && !mid_normflag)
		{
			mid_normflag = 1;
		}
		else if ((leftcol_norm > 0.3 || rightcol_norm > 0.3) && mid_normflag)
		{
			col_normflag = 1;
		}
		else if (mid_normflag && col_normflag)
		{
			mid_normflag = 0;
			col_normflag = 0;
			if (CircleDir[circle_count])
				CircleFlag = CircleDir[circle_count++];
			CircleState = 2;
		}
		else if (acc_speed > 1000 && mid_normflag)
		{
			acc_speed = 0;
			mid_normflag = 0;
		}
		/*if (mid_norm > 1.4)
		{
			if (CircleDir[circle_count])
				CircleFlag = CircleDir[circle_count++];
			CircleState = 2;
		}*/
	}
}

//================================================================//
//  @brief  :		延距判断断路
//  @param  :		void
//  @return :		void
//  @note   :		放在中断
//================================================================//
void DistBroken(void)
{
	static int BR_sum_dist = ONE_METER;
	static int BR_acc_dist = 0;
	static unsigned char state = 0;

	switch (state)
	{
	case 0:
		if (Img_BrokenFlag == 1 || Img_BrokenFlag == 3)
		{
			state = 1;
			BR_acc_dist = 0;
		}
		break;
	case 1:
		BR_acc_dist += curSpeed;
		if (Img_BrokenFlag == 2 || Img_BrokenFlag == 0)
		{
			state = 0;
			BR_acc_dist = 0;
		}
		else if (BR_acc_dist > BR_sum_dist)
		{
			state = 0;
			Img_BrokenFlag = 0;
			BR_acc_dist = 0;
		}
		break;
	default:
		break;
	}
}

//================================================================//
//  @brief  :		延距清除标志
//  @param  :		void
//  @return :		void
//  @note   :		放在中断
//================================================================//
void ClearFlag(int* flag)
{
	static int state = 0;
	static int acc_speed = 0;
	static int sum_speed = 3000;
	switch (state)
	{
	case 0:
		if (*flag)
		{
			state = 1;
		}
		break;
	case 1:
		acc_speed += curSpeed;
		if (acc_speed > sum_speed)
		{
			state = 2;
			acc_speed = 0;
		}
		break;
	case 2:
		*flag = 0;
		state = 0;
		break;
	default:
		break;
	}
}

//================================================================//
//  @brief  :		标志位误判清除
//  @param  :		void
//  @return :		void
//  @note   :		void
//================================================================//
void ClearImgFlag(void)
{
	if (Img_RampFlag)
	{
		Img_RampFlag = 0;
	}
}
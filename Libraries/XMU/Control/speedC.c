#include "GlobalVar.h"
#include "headfile.h"
#include "speedC.h"
#include "AD.h"

int speed_sum = 0;
void SpeedGet(void)
{
	//脉冲读取
	rCurSpeed = ftm_quad_get(ftm1);
//        rCurSpeed = ftm_quad_get(ftm1);//4号车
	lCurSpeed = ftm_quad_get(ftm2);
	curSpeed = (lCurSpeed + rCurSpeed) >> 1;
	ftm_quad_clean(ftm1);
	ftm_quad_clean(ftm2);
	speed_sum += curSpeed;
	//舵机误差获取
	g_steer_error = SteerDuty - InitSteer;
}

//=================================================================//
//  @brief:	计算期望速度
//  @param: 	
//  @retirn: 
//  @note: 
//=================================================================//
void ExpectSpeedGet()
{
	switch (speed_type)
	{
	case 0:
		spdExp = spdExp0;    //停车用
		//g_speedUpFlg = 0;
		break;
	case 1:
		spdExp = spdExp1;
		//g_speedUpFlg = 1;
		break;
	case 2:
		spdExp = spdExp2;
		// g_speedUpFlg = 2;
		break;
	case 3:
		spdExp = spdExp3;
		//g_speedUpFlg = 3;
		break;
	case 4:
		spdExp = spdExp4;
		//g_speedUpFlg = 4;
		break;
	case 5:
		spdExp = spdExp5;
		break;
	default:
		spdExp = spdExp0;
		// g_speedUpFlg = 0;
	}
}
//=================================================================//
//  @brief:	左轮pwm输出计算
//  @param: 	
//  @retirn: 
//  @note: 
//=================================================================//
void LeftPiCalc()
{
	float kdet = (float)KDet / 10000.0f;
	if (!protect_flag)
	{
		leftExpect = (1 - kdet * g_steer_error) * spdExp;
	}
	leftDrv.error_1 = leftExpect - lCurSpeed;

	leftDrv.p = g_speed_p;
	leftDrv.i = g_speed_i;
	leftDrv.d = g_speed_d;

	//if (leftDrv.error_1 > 10 || leftDrv.error_1 < -10) leftDrv.i = 0;		//积分分离

	leftDrv.d_duty = (int)(0.5 + leftDrv.p * (leftDrv.error_1 - leftDrv.error_2) + leftDrv.i * leftDrv.error_1 + leftDrv.d * (leftDrv.error_1 - 2 * leftDrv.error_2 + leftDrv.error_3));
	leftSpeedOut += leftDrv.d_duty;

	if (g_StateMaster > WaitingStop && speed_type == 0)			//停车急停
	{
		if (leftDrv.error_1 < -3) leftSpeedOut = SPEED_OUT_MIN;
	}



	leftDrv.error_3 = leftDrv.error_2;
	leftDrv.error_2 = leftDrv.error_1;

	if (leftSpeedOut > SPEED_OUT_MAX)
		leftSpeedOut = SPEED_OUT_MAX;
	else if (leftSpeedOut < SPEED_OUT_MIN)
		leftSpeedOut = SPEED_OUT_MIN;
}
//=================================================================//
//  @brief:	右轮pwm输出计算
//  @param: 	
//  @retirn: 
//  @note: 
//=================================================================//
void RightPiCalc()
{
	float kdet = (float)KDet / 10000.0f;
	if (!protect_flag)
	{
		rightExpect = (1 + kdet * g_steer_error) * spdExp;
	}

	rightDrv.error_1 = rightExpect - rCurSpeed;

	rightDrv.p = g_speed_p;
	rightDrv.i = g_speed_i;
	rightDrv.d = g_speed_d;

	//if (rightDrv.error_1 > 10 || rightDrv.error_1 < -10) rightDrv.i = 0;		//积分分离

	rightDrv.d_duty = (int)(0.5 + rightDrv.p * (rightDrv.error_1 - rightDrv.error_2) + rightDrv.i * rightDrv.error_1 + rightDrv.d * (rightDrv.error_1 - 2 * rightDrv.error_2 + rightDrv.error_3));
	rightSpeedOut += rightDrv.d_duty;

	if (g_StateMaster > WaitingStop && speed_type == 0)				//停车急停
	{
		if (rightDrv.error_1 < -3) rightSpeedOut = SPEED_OUT_MIN;
	}


	rightDrv.error_3 = rightDrv.error_2;
	rightDrv.error_2 = rightDrv.error_1;

	if (rightSpeedOut > SPEED_OUT_MAX)
		rightSpeedOut = SPEED_OUT_MAX;
	else if (rightSpeedOut < SPEED_OUT_MIN)
		rightSpeedOut = SPEED_OUT_MIN;

}
//=================================================================//
// @brief:    电机pwm输出
// @param: 	
//@retirn: 
// @note: 
//=================================================================//
void GiveDriveDuty(void)
{
	//右轮
	if (rightSpeedOut >= 0)
	{
		ftm_pwm_duty(ftm0, ftm_ch4, rightSpeedOut); //右正
		ftm_pwm_duty(ftm0, ftm_ch5, 0);
	}
	else
	{
		ftm_pwm_duty(ftm0, ftm_ch4, 0);
		ftm_pwm_duty(ftm0, ftm_ch5, -rightSpeedOut);
	}
	//左轮
	if (leftSpeedOut >= 0)
	{
		ftm_pwm_duty(ftm0, ftm_ch6, 0);
		ftm_pwm_duty(ftm0, ftm_ch7, leftSpeedOut);//左正
	}
	else
	{
		ftm_pwm_duty(ftm0, ftm_ch6, -leftSpeedOut);//左反
		ftm_pwm_duty(ftm0, ftm_ch7, 0);
	}
}

void CleanTmp(void)
{
	leftSpeedOut = 0;
	rightSpeedOut = 0;
	leftDrv.error_1 = 0;
	leftDrv.error_2 = 0;
	leftDrv.error_3 = 0;
	leftDrv.d_duty = 0;
	rightDrv.error_1 = 0;
	rightDrv.error_2 = 0;
	rightDrv.error_3 = 0;
	rightDrv.d_duty = 0;
}


void SpeedControl(void)
{
	if (1 == g_drive_flag)
	{
		SpeedGet();
		ExpectSpeedGet();
		LeftPiCalc();
		RightPiCalc();
		GiveDriveDuty();
		// last_speed = speed_type;
	}
	else if (0 == g_drive_flag)
	{
		SpeedGet();
		CleanTmp();
		ftm_pwm_duty(ftm0, ftm_ch6, 0);
		ftm_pwm_duty(ftm0, ftm_ch4, 0);//右
		ftm_pwm_duty(ftm0, ftm_ch7, 0);//左轮
		ftm_pwm_duty(ftm0, ftm_ch5, 0);
		//    DistantStop += curSpeed;
	}
}

//=================================================================//
//  @brief:	停车与启动
//  @param: 	
//  @retirn: 
//  @note: 
//=================================================================//
void go_stop(void)
{
	static int nowrount = 0;                          //当前圈数
	static int ntime = 0;
	if (g_drive_flag)
	{
		if (g_stop_line)              //开始停车
		{
			if (nowrount >= rount)
			{
				//        g_handle_open = 0;
				speed_type = 4;
				nowrount = 0;
			}
		}

		if (g_stop_line)
			ntime++;
		else ntime = 0;
		if (ntime > 1000)
		{
			g_stop_line = 0;
			ntime = 0;
			++nowrount;
		}
	}
	else
	{
		nowrount = 0;
		ntime = 0;
		g_stop_line = 0;
	}
}

//延距刹车
void DelayDistantStop(void)
{
	static int GiveDistant = 2500;		//人为设定延迟距离

	if (g_stop_dis)		//接收到延距刹车标志 开始累计行驶距离
		DistantStop += curSpeed;
	else DistantStop = 0;

	if (DistantStop >= GiveDistant)
	{
		g_stop_line = 1;	//执行刹车
		g_stop_dis = 0;
	}
	else;
}



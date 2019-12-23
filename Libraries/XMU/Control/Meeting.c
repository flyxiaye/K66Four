#include "headfile.h"
#include "Meeting.h"
#include "Obstacle.h"
#include "GlobalVar.h"

#define MEETING_SPEED 10
#define _ANGLE imu_data.yaw

int StartDistance = 0;
int turn_angle = 80;

//================================================================//
//  @brief  :		补图变量与会车变量调整
//  @param  :		void
//  @return :		void
//  @note   :		放在补图后面	
//================================================================//
void MeetingToImage(void)
{
	if (!g_single_open)			//非单车模式
	{
		if (0 == g_GetMeetingState && !g_GetMeetingFlag			//未开始会车
			&& 2 == Img_BrokenFlag && CarGo == g_StateMaster)
			g_GetMeetingFlag = 1;
		else if (g_GetMeetingState && !g_GetMeetingFlag			//已经完成会车，不再识别断路
			&& Img_BrokenFlag)
			Img_BrokenFlag = 0;

		if (g_GetMeetingState && 1 == Img_StopLineFlag && WaitingStop <= g_StateMaster)		//已经完成会车，开始识别停车线
			;
		else Img_StopLineFlag = 0;
	}
}

//================================================================//
//  @brief  :		四轮车会车函数
//  @param  :		void
//  @return :		void
//  @note   :		放在中断	
//================================================================//
void MeetingFour(void)
{
	static int acc_speed = 0;
	static int sum_speed = 0.5 * ONE_METER;
	static float yaw_init = 0;
	static float yaw_init_1 = 0;
	static float yaw_init_2 = 0;
	static int get_flag = 0;
	static int count = 0;
	if (1 == g_MeetingMode)				//不掉头
	{

		switch (g_StateMaster)
		{
		case Ready:
			if (g_car_lanuch) g_StateMaster = WaitingBegin;
			break;
		case WaitingBegin:
			if (g_StateSlave > Ready)
			{
				g_StateMaster = CarGo;
				g_drive_flag = 1;
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
			break;
		case CarGo:				//等待进入会车区
			if (g_GetMeetingFlag) 			 //进入会车区 
			{
				acc_speed += curSpeed;
				if (acc_speed > StartDistance)
				{
					g_GetMeetingState = 1;
					Img_BlockFlag = 0;
					g_handle_open = 0;
					acc_speed = 0;
					//改变状态 调整下一状态标志
					g_StateMaster = StateOne;
					speed_type = 2; //会车速度
					spdExp2 = spdBas2;
					yaw_init = _ANGLE;
					if (2 == g_MeetingDir)		//右转
					{
						yaw_init_1 = yaw_init + turn_angle;
						if (yaw_init_1 > 180) yaw_init_1 -= 360;
					}
					else
					{
						yaw_init_1 = yaw_init - turn_angle;
						if (yaw_init_1 < -180) yaw_init_1 += 360;
					}
				}
			}
			else acc_speed = 0;
			break;
		case StateOne:		//会车区动作以及状态
			if (2 == g_MeetingDir)
			{
				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer - 120);
				if (_ANGLE > 0 && yaw_init_1 >= 0)
				{
					if (_ANGLE > yaw_init_1)
					{
						ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
						//改变状态 调整下一状态标志
						g_StateMaster = StateTwo;
						spdExp2 = 0;
						acc_speed += curSpeed;
					}
				}
				else if (_ANGLE <= 0 && yaw_init_1 < 0)
				{
					if (_ANGLE > yaw_init_1)
					{
						ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
						//改变状态 调整下一状态标志
						g_StateMaster = StateTwo;
						spdExp2 = 0;
						acc_speed += curSpeed;
					}
				}
			}
			else
			{
				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer + 120);
				if (_ANGLE >= 0 && yaw_init_1 > 0)
				{
					if (_ANGLE < yaw_init_1)
					{
						ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
						//改变状态 调整下一状态标志
						g_StateMaster = StateTwo;
						spdExp2 = 0;
						acc_speed += curSpeed;
					}
				}
				else if (_ANGLE < 0 && yaw_init_1 <= 0)
				{
					if (_ANGLE < yaw_init_1)
					{
						ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
						//改变状态 调整下一状态标志
						g_StateMaster = StateTwo;
						spdExp2 = 0;
						acc_speed += curSpeed;
					}
				}
			}
			break;
		case StateTwo:
			//等待接收信号
			if (g_StateSlave >= WaitingStop || g_SlaveOutFlag)		//从车已过断路
			{
				//改变状态 调整下一状态标志
				g_StateMaster = StateThree;
				speed_type = 2;
				spdExp2 = 0 - spdBas2;
				yaw_init = _ANGLE;
				if (2 == g_MeetingDir)
				{
					yaw_init_1 = yaw_init - turn_angle;
					if (yaw_init_1 < -180)yaw_init_1 += 360;
				}
				else
				{
					yaw_init_1 = yaw_init + turn_angle;
					if (yaw_init_1 > 180)yaw_init_1 -= 360;
				}
			}
			break;
		case StateThree:
			//倒车
			acc_speed += curSpeed;
			if (acc_speed > 0)
				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
			else if (2 == g_MeetingDir)
				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer - 120);
			else ftm_pwm_duty(ftm3, ftm_ch5, InitSteer + 120);
			if (2 == g_MeetingDir)
			{
				if (_ANGLE >= 0 && yaw_init_1 > 0)
				{
					if (_ANGLE < yaw_init_1)
					{
						ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
						//改变状态 调整下一状态标志
						g_StateMaster = StateFour;
						spdExp2 = spdBas2;
					}
				}
				else if (_ANGLE < 0 && yaw_init_1 <= 0)
				{
					if (_ANGLE < yaw_init_1)
					{
						ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
						//改变状态 调整下一状态标志
						g_StateMaster = StateFour;
						spdExp2 = spdBas2;
					}
				}
			}
			else
			{
				if (_ANGLE > 0 && yaw_init_1 >= 0)
				{
					if (_ANGLE > yaw_init_1)
					{
						ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
						//改变状态 调整下一状态标志
						g_StateMaster = StateFour;
						spdExp2 = spdBas2;
					}
				}
				else if (_ANGLE <= 0 && yaw_init_1 < 0)
				{
					if (_ANGLE > yaw_init_1)
					{
						ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
						//改变状态 调整下一状态标志
						g_StateMaster = StateFour;
						spdExp2 = spdBas2;
					}
				}
			}
			break;
		case StateFour:
			//改变状态 调整下一状态标志
			g_StateMaster = StateFive;
			Img_BrokenFlag = 2;
			g_handle_open = 1;
			acc_speed = 0;
			//舵机输出
			IndSteerControl();
			SteerControl();//舵机控制
			break;
		case StateFive:
			acc_speed += curSpeed;
			//舵机输出
			IndSteerControl();
			SteerControl();//舵机控制
			if (!Img_BrokenFlag || acc_speed > 2 * ONE_METER)			//出断路 进入下一状态
			{
				g_GetMeetingFlag = 0;
				acc_speed = 0;
				g_StateMaster = WaitingStop;
				speed_type = 1;
			}
			break;
		case WaitingStop:		//等待识别停车线
			if (1 == MeetingStopMode)				//新停车模式
			{
				if (Img_StopLineFlag)		//识别停车线 判断从车状态
				{
					g_StateMaster = StateGo;
				}
			}
			else
			{
				if (Img_StopLineFlag && !g_SlaveOutFlag)		//识别停车线 判断从车状态
				{
					if (g_StateSlave == StateStop) //从车已到 继续跑一段距离停下
						g_StateMaster = StateGo;
					else
					{
						g_StateMaster = StateStop; //停留等待
						speed_type = 0;
					}
				}
				else if (Img_StopLineFlag && g_SlaveOutFlag)
					g_StateMaster = StateGo;
			}
			break;
		case StateGo:
			if (1 == MeetingStopMode)
			{
				acc_speed += curSpeed;
				if (acc_speed > 3.5 * sum_speed)
				{
					acc_speed = 0;
					g_StateMaster = CarFinish;
				}
			}
			else
			{
				acc_speed += curSpeed;
				if (acc_speed > sum_speed)
				{
					acc_speed = 0;
					g_StateMaster = CarFinish;
				}
			}
			break;
		case StateStop:
			if (g_StateSlave >= StateGo || g_SlaveOutFlag)		//从车到达 或者从车出界
			{
				speed_type = 1;
				acc_speed += curSpeed;
				if (acc_speed > sum_speed)
				{
					g_StateMaster = CarFinish;
					acc_speed = 0;
				}

			}
			break;
		case CarFinish:
			speed_type = 0;
			break;
		default:
			break;
		}
	}
	else if (2 == g_MeetingMode)		//掉头
	{
		switch (g_StateMaster)
		{
		case Ready:
			if (g_car_lanuch) g_StateMaster = WaitingBegin;
			break;
		case WaitingBegin:
			if (g_StateSlave > Ready)
			{
				g_StateMaster = CarGo;
				g_drive_flag = 1;
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
			break;
		case CarGo:				//等待进入会车区
			if (g_GetMeetingFlag) 			 //进入会车区 
			{
				acc_speed += curSpeed;
				if (acc_speed > StartDistance)
				{
					g_GetMeetingState = 1;
					Img_BlockFlag = 0;
					g_handle_open = 0;
					acc_speed = 0;
					//改变状态 调整下一状态标志
					g_StateMaster = StateOne;
					speed_type = 2; //会车速度
					spdExp2 = spdBas2;
					yaw_init = _ANGLE;
					if (2 == g_MeetingDir)		//右转
					{
						yaw_init_1 = yaw_init + turn_angle;
						if (yaw_init_1 > 180) yaw_init_1 -= 360;
					}
					else
					{
						yaw_init_1 = yaw_init - turn_angle;
						if (yaw_init_1 < -180) yaw_init_1 += 360;
					}
				}
			}
			else acc_speed = 0;
			break;
		case StateOne:
			if (2 == g_MeetingDir)
			{
				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer - 120);
				if (_ANGLE > 0 && yaw_init_1 >= 0
					|| _ANGLE <= 0 && yaw_init_1 < 0)
				{
					if (_ANGLE > yaw_init_1)
					{
						ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
						//改变状态 调整下一状态标志
						g_StateMaster = StateTwo;
						spdExp2 = 0;
						acc_speed += curSpeed;
					}
				}
			}
			else
			{
				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer + 120);
				if (_ANGLE >= 0 && yaw_init_1 > 0
					|| _ANGLE < 0 && yaw_init_1 <= 0)
				{
					if (_ANGLE < yaw_init_1)
					{
						ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
						//改变状态 调整下一状态标志
						g_StateMaster = StateTwo;
						spdExp2 = 0;
						acc_speed += curSpeed;
					}
				}
			}
			break;
		case StateTwo:	//延时一段时间 等待车辆停稳
			count++;
			if (count > 100 && (g_StateSlave > StateOne || g_SlaveOutFlag))		//	等待对方掉头结束
			{
				count = 0;
				//改变状态 调整下一状态标志
				g_StateMaster = StateThree;
				spdExp2 = -spdBas2;
				acc_speed += curSpeed;
				yaw_init = _ANGLE;
				if (2 == g_MeetingDir)
				{
					yaw_init_1 = yaw_init + turn_angle;
					if (yaw_init_1 > 180) yaw_init_1 -= 360;
				}
				else
				{
					yaw_init_1 = yaw_init - turn_angle;
					if (yaw_init_1 < -180) yaw_init_1 += 360;
				}
			}
			break;
		case StateThree:
			//倒车
			acc_speed += curSpeed;
			if (acc_speed > 0)
				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
			else if (2 == g_MeetingDir)
				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer + 120);
			else
				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer - 120);
			if (2 == g_MeetingDir)
			{
				if (_ANGLE > 0 && yaw_init_1 >= 0 && _ANGLE > yaw_init_1
					|| _ANGLE <= 0 && yaw_init_1 < 0 && _ANGLE > yaw_init_1)
				{
					ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
					acc_speed = 0;
					//改变状态 调整下一状态标志
					g_StateMaster = StateFour;
					spdExp2 = spdBas2;
					g_handle_open = 1;
					Img_BrokenFlag = 2;
					Img_BlockFlag = 0;
					Img_SpecialElemFlag = 1;
					CircleFlag = 0;
					CircleState = 0;
				}
			}
			else
			{
				if (_ANGLE >= 0 && yaw_init_1 > 0 && _ANGLE < yaw_init_1
					|| _ANGLE < 0 && yaw_init_1 <= 0 && _ANGLE < yaw_init_1)
				{
					ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
					acc_speed = 0;
					//改变状态 调整下一状态标志
					g_StateMaster = StateFour;
					spdExp2 = spdBas2;
					g_handle_open = 1;
					Img_BrokenFlag = 2;
					Img_BlockFlag = 0;
					Img_SpecialElemFlag = 1;
					CircleFlag = 0;
					CircleState = 0;
				}
			}
			break;
		case StateFour:
			acc_speed += curSpeed;
			ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
			if (acc_speed > 0.2 * ONE_METER)
			{
				acc_speed = 0;
				g_StateMaster = StateFive;
				speed_type = 0;
			}
			break;
		case StateFive:
			if (g_StateSlave > CarGo || g_SlaveOutFlag)		//从车到达
			{
				g_StateMaster = StateSix;
				speed_type = 2;
			}
			break;
		case StateSix:
			acc_speed += curSpeed;
			IndSteerControl();
			SteerControl();//舵机控制
			if (!Img_BrokenFlag || acc_speed > 0.2 * ONE_METER)			//出断路 进入下一状态
			{
				g_StateMaster = WaitingStop;
				g_GetMeetingFlag = 0;
				speed_type = 1;
				Img_BrokenFlag = 0;
				Img_BlockFlag = 0;
				Img_SpecialElemFlag = 0;
				CircleFlag = 0;
				CircleState = 0;
			}
			break;
		case WaitingStop:		//等待识别停车线
			if (1 == MeetingStopMode)				//新停车模式
			{
				if (Img_StopLineFlag)		//识别停车线 判断从车状态
				{
					g_StateMaster = StateGo;
				}
			}
			else
			{
				if (Img_StopLineFlag && !g_SlaveOutFlag)		//识别停车线 判断从车状态
				{
					if (g_StateSlave == StateStop) //从车已到 继续跑一段距离停下
						g_StateMaster = StateGo;
					else
					{
						g_StateMaster = StateStop; //停留等待
						speed_type = 0;
					}
				}
				else if (Img_StopLineFlag && g_SlaveOutFlag)
					g_StateMaster = StateGo;
			}
			break;
		case StateGo:
			if(1 == MeetingStopMode)
			{
				acc_speed += curSpeed;
				if (acc_speed > 3.5 * sum_speed)
				{
					acc_speed = 0;
					g_StateMaster = CarFinish;
				}
			}
			else
			{
				acc_speed += curSpeed;
				if (acc_speed > sum_speed)
				{
					acc_speed = 0;
					g_StateMaster = CarFinish;
				}
			}
			break;
		case StateStop:
			if (g_StateSlave >= StateGo || g_SlaveOutFlag)		//从车到达 或者从车出界
			{
				speed_type = 1;
				acc_speed += curSpeed;
				if (acc_speed > sum_speed)
				{
					g_StateMaster = CarFinish;
					acc_speed = 0;
				}
			}
			break;
		case CarFinish:
			speed_type = 0;
			break;
		default:
			break;
		}
	}
}

//================================================================//
//  @brief  :		二轮车会车函数
//  @param  :		void
//  @return :		void
//  @note   :		void	
//================================================================//
void MeetingTwo(void)
{
	static int acc_speed = 0;
	static int sum_speed = 0.5 * ONE_METER;
	static float yaw_init = 0;
	static float yaw_init_1 = 0;
	static float yaw_init_2 = 0;
	static int get_flag = 0;
	static int count = 0;
	switch (g_StateMaster)
	{
	case Ready:
		if (g_car_lanuch) g_StateMaster = WaitingBegin;
		break;
	case WaitingBegin:
		if (g_StateSlave > Ready)
		{
			g_StateMaster = CarGo;
			g_drive_flag = 1;
		}
		break;
	case CarGo:
		if (g_GetMeetingFlag) 			 //进入会车区 
		{
			acc_speed += curSpeed;
			if (acc_speed > StartDistance)
			{
				g_StateMaster = StateOne;
				Img_BlockFlag = 0;
				g_handle_open = 0;
				acc_speed = 0;
				//改变状态 调整下一状态标志
				speed_type = 3; //会车速度
				spdExp3 = MEETING_SPEED;
				yaw_init = _ANGLE;
				yaw_init_1 = yaw_init + turn_angle;
				if (yaw_init_1 > 180) yaw_init_1 -= 360;
			}
		}
		else acc_speed = 0;
		break;
	case StateOne:		//会车区动作以及状态
		break;

	case WaitingStop:		//等待识别停车线
		if (Img_StopLineFlag && !g_SlaveOutFlag)		//识别停车线 判断从车状态
		{
			if (g_StateSlave == StateStop) //从车已到 继续跑一段距离停下
				g_StateMaster = StateGo;
			else
			{
				g_StateMaster = StateStop; //停留等待
				speed_type = 0;
			}
		}
		else if (Img_StopLineFlag && g_SlaveOutFlag)
			g_StateMaster = StateGo;
		break;
	case StateGo:
		acc_speed += curSpeed;
		if (acc_speed > sum_speed * 1.5)
		{
			acc_speed = 0;
			g_StateMaster = CarFinish;
		}
		break;
	case StateStop:
		if (g_StateSlave >= StateGo || g_SlaveOutFlag)		//从车到达 或者从车出界
		{
			speed_type = 1;
			acc_speed += curSpeed;
			if (acc_speed > sum_speed * 1.5)
			{
				g_StateMaster = CarFinish;
				acc_speed = 0;
			}
		}
		break;
	case CarFinish:
		speed_type = 0;
		break;
	default:
		break;
	}
}
//#define MEETING_MODE 2
//#define MEETING_SPEED 10
//#define _ANGLE imu_data.yaw
//#define WaitingMeet 1
//enum MeetingFlag {
//	CarWait = 30,
//	CarGo,
//	WaitingStop = 15,
//	IsStopLine,
//	StateGo,
//	StateStop,
//	CarFinish
//};
//int StartDistance = 0; //识别断路后延距距离
//int StartSum = 0;               //距离积累量

////================================================================//
////  @brief  :		四轮车会车函数
////  @param  :		void
////  @return :		void
////  @note   :		void	
////================================================================//
//void MeetingFour(void)
//{
//	static int acc_speed = 0;
//	static int sum_speed = 0.5 * ONE_METER;
//	static float yaw_init = 0;
//	static float yaw_init_1 = 0;
//	static float yaw_init_2 = 0;
//	static int get_flag = 0;
//	static int count = 0;
//	if (1 == g_MeetingMode)	//掉头
//	{
//		switch (g_StateMaster)
//		{
//		case 0:
//			if (BrokenFlag == 2 && !RampFlag && !g_GetMeetingMaster)             //进入断路区 
//			{
//				acc_speed += curSpeed;
//				if (acc_speed > StartDistance)
//				{
//					g_GetMeetingMaster = 1;
//					g_handle_open = 0;
//					BrokenFlag = 0;
//					BlockFlag = 0;
//					acc_speed = 0;
//					//改变状态 调整下一状态标志
//					g_StateMaster = 1;
//					speed_type = 3; //会车速度
//					spdExp3 = MEETING_SPEED;
//					yaw_init = _ANGLE;
//					yaw_init_1 = yaw_init + 90;
//					if (yaw_init_1 > 180) yaw_init_1 -= 360;
//				}
//			}
//			else acc_speed = 0;
//			break;
//		case 1:
//			ftm_pwm_duty(ftm3, ftm_ch5, InitSteer - 120);
//			if (_ANGLE > 0 && yaw_init_1 >= 0)
//			{
//				if (_ANGLE > yaw_init_1)
//				{
//					ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
//					//改变状态 调整下一状态标志
//					g_StateMaster = 3;
//					spdExp3 = 0;
//					acc_speed += curSpeed;
//				}
//			}
//			else if (_ANGLE <= 0 && yaw_init_1 < 0)
//			{
//				if (_ANGLE > yaw_init_1)
//				{
//					ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
//					//改变状态 调整下一状态标志
//					g_StateMaster = 3;
//					spdExp3 = 0;
//					acc_speed += curSpeed;
//				}
//			}
//			break;
//		case 3:	//延时一段时间 等待车辆停稳
//			count++;
//			if (count > 100)
//			{
//				count = 0;
//				//改变状态 调整下一状态标志
//				g_StateMaster = 4;
//				spdExp3 = -MEETING_SPEED;
//				acc_speed += curSpeed;
//				yaw_init = _ANGLE;
//				yaw_init_1 = yaw_init + 90;
//				if (yaw_init_1 > 180) yaw_init_1 -= 360;
//			}
//			break;
//		case 4:
//			acc_speed += curSpeed;
//			if (acc_speed > 0)
//				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
//			else
//				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer + 120);
//
//			if (_ANGLE > 0 && yaw_init_1 >= 0 && _ANGLE > yaw_init_1
//				|| _ANGLE <= 0 && yaw_init_1 < 0 && _ANGLE > yaw_init_1)
//			{
//				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
//				acc_speed = 0;
//				//改变状态 调整下一状态标志
//				g_StateMaster = 5;
//				spdExp3 = 0;
//			}
//			break;
//		case 5:		//等待另一辆车
//			if (g_StateSlave > 0)	//从车到达
//			{
//				//改变状态 调整下一状态标志
//				g_StateMaster = 6;
//				spdExp3 = MEETING_SPEED;
//				g_handle_open = 1;
//				BrokenFlag = 2;
//				BlockFlag = 0;
//			}
//			break;
//		case 6:
//			if (!BrokenFlag)			//出断路 进入下一状态
//			{
//				g_StateMaster = WaitingStop;
//				speed_type = 1;
//			}
//			break;
//		case WaitingStop:		//等待识别停车线
//			if (StopLineFlag)		//识别停车线 判断从车状态
//			{
//				if (g_StateSlave == StateStop) //从车已到 继续跑一段距离停下
//					g_StateMaster = StateGo;
//				else
//				{
//					g_StateMaster = StateStop; //停留等待
//					speed_type = 0;
//				}
//			}
//			break;
//		case StateGo:
//			acc_speed += curSpeed;
//			if (acc_speed > sum_speed)
//			{
//				acc_speed = 0;
//				g_StateMaster = CarFinish;
//			}
//			break;
//		case StateStop:
//			if (g_StateSlave == StateGo || g_StateSlave == CarFinish)		//从车到达
//			{
//				speed_type = 1;
//				spdExp1 = 15;
//				acc_speed += curSpeed;
//				if (acc_speed > sum_speed)
//				{
//					g_StateMaster = CarFinish;
//					acc_speed = 0;
//				}
//			}
//			break;
//		case CarFinish:
//			speed_type = 0;
//			break;
//		default:
//			break;
//		}
//	}
//	else if (2 == g_MeetingMode)		//不掉头
//	{
//		switch (g_StateMaster)
//		{
//		case 0:
//			if (BrokenFlag == 2 && !RampFlag && !g_GetMeetingMaster)             //进入断路区 
//			{
//				acc_speed += curSpeed;
//				if (acc_speed > StartDistance)
//				{
//					g_GetMeetingMaster = 1;
//					BlockFlag = 0;
//					g_handle_open = 0;
//					acc_speed = 0;
//					//改变状态 调整下一状态标志
//					g_StateMaster = 1;
//					speed_type = 3; //会车速度
//					spdExp3 = MEETING_SPEED;
//					yaw_init = _ANGLE;
//					yaw_init_1 = yaw_init + 90;
//					if (yaw_init_1 > 180) yaw_init_1 -= 360;
//				}
//			}
//			else acc_speed = 0;
//			break;
//		case 1:
//			ftm_pwm_duty(ftm3, ftm_ch5, InitSteer - 120);
//			if (_ANGLE > 0 && yaw_init_1 >= 0)
//			{
//				if (_ANGLE > yaw_init_1)
//				{
//					ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
//					//改变状态 调整下一状态标志
//					g_StateMaster = 3;
//					spdExp3 = 0;
//					acc_speed += curSpeed;
//				}
//			}
//			else if (_ANGLE <= 0 && yaw_init_1 < 0)
//			{
//				if (_ANGLE > yaw_init_1)
//				{
//					ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
//					//改变状态 调整下一状态标志
//					g_StateMaster = 3;
//					spdExp3 = 0;
//					acc_speed += curSpeed;
//				}
//			}
//			break;
//		case 3:
//			//等待接收信号
//			if (g_StateSlave > 1 || g_SlaveOutFlag)		//从车已过断路
//			{
//				//改变状态 调整下一状态标志
//				g_StateMaster = 4;
//				spdExp3 = 0 - MEETING_SPEED;
//				yaw_init = _ANGLE;
//				yaw_init_1 = yaw_init - 90;
//				if (yaw_init_1 < -180)yaw_init_1 += 360;
//			}
//			break;
//		case 4:
//			//倒车
//			acc_speed += curSpeed;
//			if (acc_speed > 0)
//				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
//			else
//				ftm_pwm_duty(ftm3, ftm_ch5, InitSteer - 120);
//			if (_ANGLE >= 0 && yaw_init_1 > 0)
//			{
//				if (_ANGLE < yaw_init_1)
//				{
//					ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
//					//改变状态 调整下一状态标志
//					g_StateMaster = 5;
//					spdExp3 = MEETING_SPEED;
//				}
//			}
//			else if (_ANGLE < 0 && yaw_init_1 <= 0)
//			{
//				if (_ANGLE < yaw_init_1)
//				{
//					ftm_pwm_duty(ftm3, ftm_ch5, InitSteer);
//					//改变状态 调整下一状态标志
//					g_StateMaster = 5;
//					spdExp3 = MEETING_SPEED;
//				}
//			}
//			break;
//		case 5:
//			//改变状态 调整下一状态标志
//			g_StateMaster = 6;
//			BrokenFlag = 2;
//			g_handle_open = 1;
//
//			//舵机输出
//			g_camera_ctrl_flag = 0;
//			g_ind_ctrl_flag = 1;
//			GetSteerOut();//计算舵机输出
//			SteerControl();//舵机控制
//			break;
//		case 6:
//			//舵机输出
//			g_camera_ctrl_flag = 0;
//			g_ind_ctrl_flag = 1;
//			GetSteerOut();//计算舵机输出
//			SteerControl();//舵机控制
//			if (!BrokenFlag)			//出断路 进入下一状态
//			{
//				g_GetMeetingMaster = 0;
//				acc_speed = 0;
//				g_StateMaster = WaitingStop;
//				speed_type = 1;
//			}
//			break;
//		case WaitingStop:		//等待识别停车线
//			if (StopLineFlag && !g_SlaveOutFlag)		//识别停车线 判断从车状态
//			{
//				if (g_StateSlave == StateStop) //从车已到 继续跑一段距离停下
//					g_StateMaster = StateGo;
//				else
//				{
//					g_StateMaster = StateStop; //停留等待
//					speed_type = 0;
//					//spdExp1 = 0;
//				}
//			}
//			else if (StopLineFlag && g_SlaveOutFlag)
//				g_StateMaster = StateGo;
//			break;
//			//case IsStopLine:
//			//	if (g_StateSlave == StateStop) //从车已到 继续跑一段距离停下
//			//		g_StateMaster = StateGo;
//			//	else
//			//	{
//			//		g_StateMaster = StateStop; //停留等待
//			//		speed_type = 1;
//			//		spdExp1 = 0;
//			//	}
//			//	break;
//		case StateGo:
//			acc_speed += curSpeed;
//			if (acc_speed > sum_speed * 1.5)
//			{
//				acc_speed = 0;
//				g_StateMaster = CarFinish;
//			}
//			break;
//		case StateStop:
//			if (g_StateSlave >= StateGo || g_SlaveOutFlag)		//从车到达 或者从车出界
//			{
//				speed_type = 1;
//				acc_speed += curSpeed;
//				if (acc_speed > sum_speed * 1.5)
//				{
//					g_StateMaster = CarFinish;
//					acc_speed = 0;
//				}
//			}
//			break;
//		case CarFinish:
//			speed_type = 0;
//			break;
//		default:
//			break;
//		}
//	}
//}
//
////================================================================//
////  @brief  :		二轮车会车函数
////  @param  :		void
////  @return :		void
////  @note   :		void	
////================================================================//
//void MeetingTwo(void)
//{
//	static int acc_speed = 0;
//	static int sum_speed = 2500;
//	static float yaw_init = 0;
//	static float yaw_init_1 = 0;
//	static float yaw_init_2 = 0;
//	static int get_flag = 0;
//	static int count = 0;
//	if (1 == g_MeetingMode)
//	{
//		switch (g_StateMaster)
//		{
//		case 0:
//			if (BrokenFlag == 2 && !RampFlag && !g_GetMeetingMaster)             //进入断路区 
//			{
//				BlockFlag = 0;//clear block 
//				acc_speed += curSpeed;
//				if (acc_speed > StartDistance)
//				{
//					g_GetMeetingMaster = 1;
//					//改变状态 调整下一状态标志
//					g_StateMaster = 1;
//					g_drive_flag = 0;//停车
//					BrokenFlag = 0;
//
//				}
//			}
//			break;
//		case 1://等待从车到达标志
//			if (g_StateSlave > 0)
//			{
//				g_StartMaster = 2;
//			}
//			break;
//		case 2:
//			//转弯
//			//此处添加转弯函数 车直立起来 然后慢慢转弯
//
//			//结束标志位变化
//			g_StateMaster = 3;
//			BrokenFlag = 2;
//
//			break;
//		case 3:
//			//启动
//			if (0 == BrokenFlag)
//			{
//				//出断路结束会车状态发送标志位
//				g_StateMaster = WaitingStop;		//结束会车
//				g_GetMeetingMaster = 0;
//			}
//			break;
//		case WaitingStop:		//出断路 等待识别停车线
//			if (StopLineFlag)	//识别到停车线
//			{
//				if (g_StateSlave == IsStopLine) //从车已到 继续跑一段距离停下
//					g_StateMaster = StateGo;
//				else
//				{
//					g_StateMaster = StateStop; //停留等待
//					g_drive_flag = 0;
//				}
//			}
//			break;
//		case StateGo:
//			acc_speed += curSpeed;
//			if (acc_speed > sum_speed)
//			{
//				acc_speed = 0;
//				g_StateMaster = CarFinish;
//			}
//			break;
//		case StateStop:
//			if (g_StateSlave == StateGo || g_StateSlave == CarFinish)		//从车到达
//			{
//				g_drive_flag = 1;
//				acc_speed += curSpeed;
//				if (acc_speed > sum_speed)
//				{
//					g_StateMaster = CarFinish;
//					acc_speed = 0;
//				}
//			}
//			break;
//		case CarFinish:
//			g_drive_flag = 0;
//		default:
//			break;
//		}
//	}
//	else if (2 == g_MeetingMode)	//不掉头
//	{
//		switch (g_StateMaster)
//		{
//		case 0:
//			if (BrokenFlag == 2 && !RampFlag && !g_GetMeetingMaster)             //进入断路区 
//			{
//				BlockFlag = 0;//clear block 
//				if (acc_speed > StartDistance)
//				{
//					g_GetMeetingMaster = 1;
//					//改变状态 调整下一状态标志
//					g_StateMaster = 1;
//				}
//			}
//			break;
//		case 1:		//电磁跑断路
//			if (0 == BrokenFlag)
//			{
//				//出断路结束会车状态发送标志位
//				g_StateMaster = WaitingStop;		//结束会车
//				g_GetMeetingMaster = 0;
//			}
//			break;
//		case WaitingStop:		//出断路 等待识别停车线
//			if (StopLineFlag)	//识别到停车线
//			{
//				if (g_StateSlave == IsStopLine) //从车已到 继续跑一段距离停下
//					g_StateMaster = StateGo;
//				else
//				{
//					g_StateMaster = StateStop; //停留等待
//					g_drive_flag = 0;
//				}
//			}
//			break;
//			//case IsStopLine:
//			//	if (g_StateSlave == IsStopLine) //从车已到 继续跑一段距离停下
//			//		g_StateMaster = StateGo;
//			//	else
//			//		g_StateMaster = StateStop; //停留等待
//			//	break;
//		case StateGo:
//			acc_speed += curSpeed;
//			if (acc_speed > sum_speed)
//			{
//				acc_speed = 0;
//				g_StateMaster = CarFinish;
//			}
//			break;
//		case StateStop:
//			if (g_StateSlave == StateGo || g_StateSlave == CarFinish)		//从车到达
//			{
//				g_drive_flag = 1;
//				acc_speed += curSpeed;
//				if (acc_speed > sum_speed)
//				{
//					g_StateMaster = CarFinish;
//					acc_speed = 0;
//				}
//			}
//			break;
//		case CarFinish:
//			g_drive_flag = 0;
//		default:
//			break;
//		}
//	}
//}

//================================================================//
//  @brief  :		状态转化平滑输出
//  @param  :		dst 目标输出 src源输出 duty目标输出占空比
//  @return :		输出量
//  @note   :		源输出向目标输出过渡
//================================================================//
float LinearOut(float dst, float src, float duty)
{
	return (dst * duty + src * (1 - duty));
}


#undef _ANGLE 

//================================================================//
//  @brief  :		四轮车单车模式
//  @param  :		void
//  @return :		void
//  @note   :		void	
//================================================================//
void SingleFour(void)
{
	static int sum_speed = ONE_METER;
	static int acc_speed = 0;
	static unsigned char state = 0;
	switch (state)
	{
	case 0:
		if (Img_StopLineFlag)
		{
			state = 1;
		}
		break;
	case 1:
		acc_speed += curSpeed;
		if (acc_speed > sum_speed)
		{
			state = 0;
			acc_speed = 0;
			speed_type = 0;
			Img_StopLineFlag = 0;
		}
		break;
	default:
		break;
	}

}
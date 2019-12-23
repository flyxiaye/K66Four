#include <GlobalVar.h>
#include "headfile.h"
const int InitSteer = 1175;              //舵机初始值 75Hz 1149 50Hz 766  584 1435
const unsigned char g_Bit_Val_Up[8] = { 0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80 };//液晶图像压缩;
const unsigned char g_Bit_Val_Down[8] = { 0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01 };

//==========================方案选择模式============================//
unsigned char BlockCount = 2;			//路障计数
unsigned char Img_GrayJumpOpen = 0;		//灰度跳变检测断路开关
unsigned char BrokenMode = 0;			//进断路模式
unsigned char MeetingStopMode = 0;		//停车模式

//==========================元素计数变量============================//

unsigned char BlockDir[5];			//路障计数
unsigned char CircleDir[10];		//环岛计数
unsigned char Ind_CircleOpen = 0;	//电磁判断环岛开关

//==========================图像变量============================//
//#include "GlobalVar.h"
const int MidOffset[] = {
	 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 11, 12, 13, 14, 14, 15, 16, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 52, 53, 54, 55, 56, 57, 58, 59, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93,
};
unsigned char ImageEage[IMG_ROW][IMG_COL];
int ML[IMG_ROW], LL[IMG_ROW], RL[IMG_ROW];   //保存边缘线信息数组
int ML_Count;									//中线有效行
SpecialPoint LeftPnt, RightPnt;					//保存左右特殊点信息
int DiffThreshold = 25;							//边缘检测阈值
int DarkThreshold = 55;                         //暗阈值（路障）
int BrightThreshold = 15;                      //亮阈值（坡道）
int LightThreshold = 80;						//去高光噪声阈值
int LightThreshold2 = 150;						//去高光噪声阈值2
int BrokenThreshold = 35;						//断路灰度阈值
int FindLineType = 0;							//是否加入高光滤波标志
int g_RoadType = 0;
int ErrorFlag = 0;								//错误标志位
//1双侧十字错误  2误判为斜十字  3视野丢失错误  4中线错误  5双侧十字噪点错误  6平十字噪点错误
int LastMiddleLine = 0;

int SpeedRow = 0;			//控速边界行数

int CircleFlag = 0;
int CircleState = 0;
unsigned char Dist_ClearSevenFlag = 0;		//延距清环岛7标志位

int LeftIntLine = 0;		//记录左内跳行数
int RightIntLine = 0;		//记录右内跳行数

int Img_SpecialElemFlag = 0;    //特殊元素标志
int Img_BrokenFlag = 0;			//断路标志
int Img_BlockFlag = 0;			//路障标志
int Img_StopLineFlag = 0;		//停车线标志
int Img_RampFlag = 0;			//坡道标志

int BrokenLastAve = 0;
int StopLineDist = 0;

unsigned char Img_CircleOpen = 0;
unsigned char Img_StraightBrokenOpen = 0;
unsigned char Img_CurveBrokenOpen = 0;
unsigned char Img_StopOpen = 0;
unsigned char Img_BlockOpen = 0;
unsigned char Img_RampOpen = 0;
void VarInit(void)
{
	ML_Count = 0;
	ErrorFlag = 0;
	g_RoadType = 0;
	LeftIntLine = 0;
	RightIntLine = 0;
	//下面某些地方起了关键作用
	LeftPnt.ErrCol = RightPnt.ErrCol = 0;
	LeftPnt.ErrRow = RightPnt.ErrRow = DOWN_EAGE;
	for (int i = 0; i < IMG_ROW; i++)
	{
		LL[i] = LEFT_EAGE;
		RL[i] = RIGHT_EAGE;
		ML[i] = MIDDLE;
	}

}

//==========================摄像头硬件参数区============================//
unsigned char image[ROW][COL];        //摄像头数据接收
int exp_time = 300;                           //摄像头曝光时间
int HighThreshold = 50;						//canny高阈值
int LowThreshold = 20;						//canny低阈值
int ControlMid = 94;						//图像控制中值

//==========================菜单标志==============================//


int g_car_lanuch = 0;			//车辆启动标志
int g_drive_flag = 0;           //电机开关标志
int g_ad_flag = 0;              //电感采集标志
int g_steer_open = 1;           //舵机开关标志
int g_handle_open = 0;          //补图开关标志
int g_ramp_open = 1;            //坡道开关标志
//int g_broken_open = 1;          //断路开关标志
int g_block_open = 1;           //路障开关标志
int g_single_open = 0;          //单车模式开关标志
int g_camera_open = 1;			//摄像头开关
int g_ind_open = 1;				//电磁开关
int g_camera_ctrl_flag = 0;		//摄像头控制标志
int g_ind_ctrl_flag = 0;		//电磁控制标志
int g_broken_enable = 0;
int BugFlag = 0;

int speed_type = 1;

//====================================舵机控制================================//
int Error_1 = 0;
int Error_2 = 0;
float Steer_P = 2;
float Steer_P2 = 0;
float Steer_P_CI = 2.0f;		//环岛p
float p_max = 0;
float p_min = 0;
float p_long = 0;
float d_max = 0;
float Steer_D = 0;
float Steer_D1 = 0;     //入弯
float Steer_D2 = 0;     //出弯
int ProSpect = 55;
int SteerDuty = 0;
float long_steer_p = 0.5;
float long_steer_d = 0.5;

float Steer_P_Ind = 88.85f;
float Steer_D_Ind = 8470.0f;
float p_max_Ind = 100;
float p_min_Ind = 82;
float p_LongRoad_Ind = 40;
unsigned char Ind_LongRoadFlag = 0;


//=====================================电感===================================//
int ind_left, ind_right, ind_mid, ind_leftcol, ind_rightcol;	//记录电感
int ind_leftmax, ind_leftmin, ind_rightmax, ind_rightmin, ind_midmax, ind_midmin; 	//电感最值
int ind_leftcolmax, ind_leftcolmin, ind_rightcolmax, ind_rightcolmin;
float left_norm, right_norm, mid_norm, leftcol_norm, rightcol_norm;
float ad_error_1 = 0, ad_error_2 = 0;
int mul_ad_error = 120;         //电磁error放大倍数
int ind_ci_th = 2000;			//电磁判断环岛延距
float Ind_CI_TH = 1.5;			//电磁圆环中电感判据

/*=====================================增量式速度控制========================================*/
int KDet = 0;

float curSpeed = 0;
int lCurSpeed = 0;
int rCurSpeed = 0;
SPEED leftDrv = { 0, 0, 0, 0, 0 };
SPEED rightDrv = { 0, 0, 0, 0, 0 };
int g_steer_error = 0;
//期望速度
int spdExp = 0;
const int spdExp0 = 0;	//停车速度
int spdExp1 = 15;		//平跑速度
int spdExp2 = 10;		//会车速度
int spdExp3 = 15;		//纯电磁速度
int spdExp4 = 15;		//电感速度
int spdExp5 = 30;		//路障速度
int spdBas1 = 28;		//平跑基准速度
int spdBas2 = 10;		//会车基准速度
int spdBas3 = 15;		//纯电磁基准速度

float leftExpect; //左轮期望速度
float rightExpect;//右轮期望速度
signed int leftSpeedOut = 0; //左轮速度输出
signed int rightSpeedOut = 0;//右轮速度输出
//速度控制pi
//float g_speed_p = 280;
//float g_speed_i = 7;
float g_speed_p = 300;
float g_speed_i = 15;
float g_speed_d = 0;

signed int lOverflowFlg = 0;//左轮遇限方向积分溢出标志
signed int rOverflowFlg = 0;//右轮遇限方向积分溢出标志
//int s4=0;
//int s5 =0;
//int s6=0;
//int s7=0;


//==================================刹车=================//
int g_stop_line = 0;
int g_stop_dis = 0;
int times_go = 0;
int times_stop = 0;
int rount = 1;          //圈数
int DistantStop = 0;		//累计编码器脉冲

//==========================TF卡============================//

unsigned char g_Img_All_120x188[ROW][COL];

/*=====================================双车通讯======================================*/

signed char btMessage = 49;
signed char btRcvMessage = 0;
btInfo btSndInfo = { 0 };
btInfo btRcvInfo = { 0 };

/*=====================================双车状态======================================*/
//unsigned char g_GetMeetingMaster = 0;            //主车到达会车区标志
//unsigned char g_GetMeetingSlave = 0;             //从车到达会车区标志
//unsigned char g_EndMeetingMaster = 0;           //主车结束会车标志
unsigned char g_MasterOutFlag = 0;              //主车出界
unsigned char g_SlaveOutFlag = 0;               //从车出界
unsigned char g_StateMaster = 0;		//主车状态
unsigned char g_StateSlave = 0;			//从车状态
unsigned char g_MeetingMode = 2;		//会车模式， 1不掉头 2掉头
unsigned char g_MeetingDir = 2;			//转向方向 2为右转 1为左转
unsigned char g_GetMeetingFlag = 0;	//进入会车标志 通过其他动作改变
unsigned char g_GetMeetingState = 0;	//已经会车 0未会车 1已经会车 识别会车区后变为1

/*====================================会车区惯导变量=====================================*/
int sum_distance_1 = 3400;
int sum_distance_2 = 2000;
int sum_distance_3 = 2000;
/*====================================路障=====================================*/
int sum = 0;						//惯导距离累计变量
int sum_dist = 3800;				//前进距离
int delay_dist = 2900;				//识别后延距进入路障控制
int g_inf = 0;						//获取的红外数值
int st = 60;						//预期偏转角度
int st_end = 57;					//回正角度
int stop_inf = 5000;					//红外生效值
/*====================================坡道=====================================*/

/*===================================拨码开关===========================================*/
int dialSwitchFlg1 = 0;
int dialSwitchFlg2 = 0;
int dialSwitchFlg3 = 0;
int dialSwitchFlg4 = 0;
int dialSwitchcal = 0;

/*=====================================陀螺仪变量======================================*/
signed int I2C_Wait_Times = 300;//i2c等待次数
signed int I2C_Wait_Err_Flg = 0;//i2c等待死循环错误标志
float g_gyroBalance = -311;       //陀螺仪平衡值
float g_gyroIntegration = 0;       //陀螺仪积分量

int g_gyroDimension = 65536;       //陀螺仪转换加速度计量纲

float g_angleResult = 0;           //角度计算结果
float g_angleResult_last = 0;
float g_angle_temp = 0;

int g_gyroGet = 0;               //陀螺仪读取值
signed int g_gyroGet_max = 0;
signed int g_gyroGet_min = 0;
int g_gyro_jolt = 0;
int g_jolt_flag = 0;

signed int g_gyroClearRamp = 0;   //陀螺仪检测清坡道标志

int g_gyro_cnt = 0;//环岛
int g_ramp_judge = -10;//坡道角度阈值

/*=====================================pwm测试========================================*/
int Left_Pwm = 0;
int Left_Pwm2 = 0;
int Right_Pwm = 0;
int Right_Pwm2 = 0;
///*=====================================速度控制========================================*/
//int opposite_ready = 0,opposite_wait = 1,opposite_done = 0;
//int KDet = 20;
//
//
//float curSpeed=0;
//int lCurSpeed=0;
//int rCurSpeed=0;
//int sidetype=0;
//SPEED leftDrv;
//SPEED rightDrv;
//
//int g_steer_error = 0;
//
////期望速度
//int spdExp=0;
//int spdExp0=10;
//int spdExp1 = 0;
//int spdExp2=22;
//int spdExp3=0;
//int spdExp4=0;
//
//float leftExpect; //左轮期望速度
//float rightExpect;//右轮期望速度
//signed int leftSpeedOut; //左轮速度输出
//signed int rightSpeedOut;//右轮速度输出
////速度控制pi
//float g_speed_p=280;
//float g_speed_i=7;
//signed int lOverflowFlg=0;//左轮遇限方向积分溢出标志
//signed int rOverflowFlg=0;//右轮遇限方向积分溢出标志
////int s4=0;
////int s5 =0;
////int s6=0;
////int s7=0;
//
//int d_spd_max = 0;
//int d_spd_high = 0;
//int d_spd_mid = 0;
//int d_spd_low = 0;
//
//int s_width = 15;
//int m_width = 50;
//int l_width = 100;




//float EncoderCount = 0;			//编码器数值
//signed int DialSwitchFlag1;            //拨码开关1  舵机
//signed int DialSwitchFlag2;            //拨码开关2  
//signed int DialSwitchFlag3;            //拨码开关3
//signed int DialSwitchFlag4;            //拨码开关4
//
//
////==========================方向控制量============================//
//float SteeringMin = 3320;			//舵机最小值
//float SteeringMid = 4020;			//舵机中值
//float SteeringMax =4720;			//舵机最大值
//float SteerPWMOut = 0;		//舵机输出值
//
//float SteeringCtrlP =0;			//舵机控制参数P
//float SteeringCtrlDi = 0;			//舵机控制入弯参数
//float SteeringCtrlDo = 0;		//舵机控制出弯参数D
//
//
//
//
//
//float SteeringCtrlP_universe =13;			//舵机控制参数P(弯道)
//float SteeringCtrlDi_universe = 60;			//舵机控制入弯参数D（弯道）
//float SteeringCtrlDo_universe = 60;		//舵机控制出弯参数D（弯道）
//
//
//
//float SteeringCtrlP_circle = 7;			//舵机控制参数P
//float SteeringCtrlDi_circle = 50;			//舵机控制入弯参数
//float SteeringCtrlDo_circle = 50;		//舵机控制出弯参数D
//
//
//
//float SteerStartDist = 10;			//偏差获取距离底边起始距离
//float SteerStartDist_universe = 20;			//偏差获取距离底边起始距离
//float SteerEndDist = 50;			//偏差获取距离底边终止距离
//float SteerEndDist_circle = 60;			//偏差获取距离底边终止距离
//float High_pass_paramentA=1;                   //偏差高通系数
//float High_pass_paramentB=0;                   //偏差高通系数
//
//float  Compensate=1.2;      //偏差补偿系数
//
//
//
//
///*=====================================速度控制========================================*/
//int g_drive_flag=0;
//
//float curSpeed=0;       //
//int lCurSpeed=0;        //左轮速度
//int rCurSpeed=0;        //右轮速度
//int sidetype=0;         
//int car_start_flag=0;   
//SPEED leftDrv;          
//SPEED rightDrv;         
//int KDet = 20;          //差速
//int lock_count = 0;   
////轮胎卡死计数
//int g_speed_type=0;
////期望速度
//int spdExp=0;
//int spdExp0=0;         //长直道速度         //40
//int spdExp1=20;         //非长直道速度       //32
//int spdExp2=30;         //弯道速度          //27
//int spdExp3=40;          //停车
//int spdExp4=15;         //会车速度
//int spdExp5=15;         //弯道前减速速度
//
//float leftExpect;               //左轮期望速度
//float rightExpect;              //右轮期望速度
//signed int leftSpeedOut;        //左轮速度输出
//signed int rightSpeedOut;       //右轮速度输出
////速度控制pi
//float g_speed_p = 460;          //P
//float g_speed_i = 7;            //I
//float g_speed_d = 3;            //D
//signed int lOverflowFlg=0;      //左轮遇限方向积分溢出标志
//signed int rOverflowFlg=0;      //右轮遇限方向积分溢出标志
//
///*舵机控制*/
//
//int error=0;
//int error_cnt=0;
//int error_cnt_max=4950;
//int error_max=22;
//int last_error=0;
//int g_steer_duty=STEERMID;
//float g_p=0.7f;
//float g_d=1.0f;
//
//
//int second=5;
//int pit2_cnt=0;
////==========================坡道变量============================//
//unsigned char g_Ramp_Flg = 0;//坡道检测确认标志
//float g_Angle_Result = 0; //角度计算结果
//float g_Angle_threshold=10;        //下坡识别阈值
//float g_Gyro_Balance = -315;               //陀螺仪平衡值
//float g_Gyto_integration = 0;               //陀螺仪积分量
//float g_Gyro_Dimension = 65536;                //陀螺仪转换加速度计量纲
//signed int g_Gyro_Get = 0;                  //陀螺仪读取值  
//signed int Ramp_time=0;              //坡道间隔时间
//
////==========================颠簸路面变量============================//
//float Jolt_Open=1;            //颠簸路面识别开启标志
//signed int Jolt_Get=0;            //颠簸方向角速度
//signed int Jolt_Get_min=0;            //颠簸方向角速度最小值
//signed int Jolt_Get_max=0;            //颠簸方向角速度最大值
//signed int Jolt_Get_delta=0;            //颠簸方向角速度极差
//float Jolt_delta=10000;             //颠簸识别阈值
//signed int Jolt_Balance=-315;      //颠簸方向角速度平衡值
//unsigned char Jolt_Flag=0;        //颠簸标志位
//float Jolt_Down_Speed=10;           //颠簸路面下降速度
//float Jolt_time=100;            //颠簸路面时间



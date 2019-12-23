#include <GlobalVar.h>
#include "headfile.h"
const int InitSteer = 1175;              //�����ʼֵ 75Hz 1149 50Hz 766  584 1435
const unsigned char g_Bit_Val_Up[8] = { 0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80 };//Һ��ͼ��ѹ��;
const unsigned char g_Bit_Val_Down[8] = { 0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01 };

//==========================����ѡ��ģʽ============================//
unsigned char BlockCount = 2;			//·�ϼ���
unsigned char Img_GrayJumpOpen = 0;		//�Ҷ��������·����
unsigned char BrokenMode = 0;			//����·ģʽ
unsigned char MeetingStopMode = 0;		//ͣ��ģʽ

//==========================Ԫ�ؼ�������============================//

unsigned char BlockDir[5];			//·�ϼ���
unsigned char CircleDir[10];		//��������
unsigned char Ind_CircleOpen = 0;	//����жϻ�������

//==========================ͼ�����============================//
//#include "GlobalVar.h"
const int MidOffset[] = {
	 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 11, 12, 13, 14, 14, 15, 16, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 52, 53, 54, 55, 56, 57, 58, 59, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93, 93,
};
unsigned char ImageEage[IMG_ROW][IMG_COL];
int ML[IMG_ROW], LL[IMG_ROW], RL[IMG_ROW];   //�����Ե����Ϣ����
int ML_Count;									//������Ч��
SpecialPoint LeftPnt, RightPnt;					//���������������Ϣ
int DiffThreshold = 25;							//��Ե�����ֵ
int DarkThreshold = 55;                         //����ֵ��·�ϣ�
int BrightThreshold = 15;                      //����ֵ���µ���
int LightThreshold = 80;						//ȥ�߹�������ֵ
int LightThreshold2 = 150;						//ȥ�߹�������ֵ2
int BrokenThreshold = 35;						//��·�Ҷ���ֵ
int FindLineType = 0;							//�Ƿ����߹��˲���־
int g_RoadType = 0;
int ErrorFlag = 0;								//�����־λ
//1˫��ʮ�ִ���  2����Ϊбʮ��  3��Ұ��ʧ����  4���ߴ���  5˫��ʮ��������  6ƽʮ��������
int LastMiddleLine = 0;

int SpeedRow = 0;			//���ٱ߽�����

int CircleFlag = 0;
int CircleState = 0;
unsigned char Dist_ClearSevenFlag = 0;		//�Ӿ��廷��7��־λ

int LeftIntLine = 0;		//��¼����������
int RightIntLine = 0;		//��¼����������

int Img_SpecialElemFlag = 0;    //����Ԫ�ر�־
int Img_BrokenFlag = 0;			//��·��־
int Img_BlockFlag = 0;			//·�ϱ�־
int Img_StopLineFlag = 0;		//ͣ���߱�־
int Img_RampFlag = 0;			//�µ���־

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
	//����ĳЩ�ط����˹ؼ�����
	LeftPnt.ErrCol = RightPnt.ErrCol = 0;
	LeftPnt.ErrRow = RightPnt.ErrRow = DOWN_EAGE;
	for (int i = 0; i < IMG_ROW; i++)
	{
		LL[i] = LEFT_EAGE;
		RL[i] = RIGHT_EAGE;
		ML[i] = MIDDLE;
	}

}

//==========================����ͷӲ��������============================//
unsigned char image[ROW][COL];        //����ͷ���ݽ���
int exp_time = 300;                           //����ͷ�ع�ʱ��
int HighThreshold = 50;						//canny����ֵ
int LowThreshold = 20;						//canny����ֵ
int ControlMid = 94;						//ͼ�������ֵ

//==========================�˵���־==============================//


int g_car_lanuch = 0;			//����������־
int g_drive_flag = 0;           //������ر�־
int g_ad_flag = 0;              //��вɼ���־
int g_steer_open = 1;           //������ر�־
int g_handle_open = 0;          //��ͼ���ر�־
int g_ramp_open = 1;            //�µ����ر�־
//int g_broken_open = 1;          //��·���ر�־
int g_block_open = 1;           //·�Ͽ��ر�־
int g_single_open = 0;          //����ģʽ���ر�־
int g_camera_open = 1;			//����ͷ����
int g_ind_open = 1;				//��ſ���
int g_camera_ctrl_flag = 0;		//����ͷ���Ʊ�־
int g_ind_ctrl_flag = 0;		//��ſ��Ʊ�־
int g_broken_enable = 0;
int BugFlag = 0;

int speed_type = 1;

//====================================�������================================//
int Error_1 = 0;
int Error_2 = 0;
float Steer_P = 2;
float Steer_P2 = 0;
float Steer_P_CI = 2.0f;		//����p
float p_max = 0;
float p_min = 0;
float p_long = 0;
float d_max = 0;
float Steer_D = 0;
float Steer_D1 = 0;     //����
float Steer_D2 = 0;     //����
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


//=====================================���===================================//
int ind_left, ind_right, ind_mid, ind_leftcol, ind_rightcol;	//��¼���
int ind_leftmax, ind_leftmin, ind_rightmax, ind_rightmin, ind_midmax, ind_midmin; 	//�����ֵ
int ind_leftcolmax, ind_leftcolmin, ind_rightcolmax, ind_rightcolmin;
float left_norm, right_norm, mid_norm, leftcol_norm, rightcol_norm;
float ad_error_1 = 0, ad_error_2 = 0;
int mul_ad_error = 120;         //���error�Ŵ���
int ind_ci_th = 2000;			//����жϻ����Ӿ�
float Ind_CI_TH = 1.5;			//���Բ���е���о�

/*=====================================����ʽ�ٶȿ���========================================*/
int KDet = 0;

float curSpeed = 0;
int lCurSpeed = 0;
int rCurSpeed = 0;
SPEED leftDrv = { 0, 0, 0, 0, 0 };
SPEED rightDrv = { 0, 0, 0, 0, 0 };
int g_steer_error = 0;
//�����ٶ�
int spdExp = 0;
const int spdExp0 = 0;	//ͣ���ٶ�
int spdExp1 = 15;		//ƽ���ٶ�
int spdExp2 = 10;		//�ᳵ�ٶ�
int spdExp3 = 15;		//������ٶ�
int spdExp4 = 15;		//����ٶ�
int spdExp5 = 30;		//·���ٶ�
int spdBas1 = 28;		//ƽ�ܻ�׼�ٶ�
int spdBas2 = 10;		//�ᳵ��׼�ٶ�
int spdBas3 = 15;		//����Ż�׼�ٶ�

float leftExpect; //���������ٶ�
float rightExpect;//���������ٶ�
signed int leftSpeedOut = 0; //�����ٶ����
signed int rightSpeedOut = 0;//�����ٶ����
//�ٶȿ���pi
//float g_speed_p = 280;
//float g_speed_i = 7;
float g_speed_p = 300;
float g_speed_i = 15;
float g_speed_d = 0;

signed int lOverflowFlg = 0;//�������޷�����������־
signed int rOverflowFlg = 0;//�������޷�����������־
//int s4=0;
//int s5 =0;
//int s6=0;
//int s7=0;


//==================================ɲ��=================//
int g_stop_line = 0;
int g_stop_dis = 0;
int times_go = 0;
int times_stop = 0;
int rount = 1;          //Ȧ��
int DistantStop = 0;		//�ۼƱ���������

//==========================TF��============================//

unsigned char g_Img_All_120x188[ROW][COL];

/*=====================================˫��ͨѶ======================================*/

signed char btMessage = 49;
signed char btRcvMessage = 0;
btInfo btSndInfo = { 0 };
btInfo btRcvInfo = { 0 };

/*=====================================˫��״̬======================================*/
//unsigned char g_GetMeetingMaster = 0;            //��������ᳵ����־
//unsigned char g_GetMeetingSlave = 0;             //�ӳ�����ᳵ����־
//unsigned char g_EndMeetingMaster = 0;           //���������ᳵ��־
unsigned char g_MasterOutFlag = 0;              //��������
unsigned char g_SlaveOutFlag = 0;               //�ӳ�����
unsigned char g_StateMaster = 0;		//����״̬
unsigned char g_StateSlave = 0;			//�ӳ�״̬
unsigned char g_MeetingMode = 2;		//�ᳵģʽ�� 1����ͷ 2��ͷ
unsigned char g_MeetingDir = 2;			//ת���� 2Ϊ��ת 1Ϊ��ת
unsigned char g_GetMeetingFlag = 0;	//����ᳵ��־ ͨ�����������ı�
unsigned char g_GetMeetingState = 0;	//�Ѿ��ᳵ 0δ�ᳵ 1�Ѿ��ᳵ ʶ��ᳵ�����Ϊ1

/*====================================�ᳵ���ߵ�����=====================================*/
int sum_distance_1 = 3400;
int sum_distance_2 = 2000;
int sum_distance_3 = 2000;
/*====================================·��=====================================*/
int sum = 0;						//�ߵ������ۼƱ���
int sum_dist = 3800;				//ǰ������
int delay_dist = 2900;				//ʶ����Ӿ����·�Ͽ���
int g_inf = 0;						//��ȡ�ĺ�����ֵ
int st = 60;						//Ԥ��ƫת�Ƕ�
int st_end = 57;					//�����Ƕ�
int stop_inf = 5000;					//������Чֵ
/*====================================�µ�=====================================*/

/*===================================���뿪��===========================================*/
int dialSwitchFlg1 = 0;
int dialSwitchFlg2 = 0;
int dialSwitchFlg3 = 0;
int dialSwitchFlg4 = 0;
int dialSwitchcal = 0;

/*=====================================�����Ǳ���======================================*/
signed int I2C_Wait_Times = 300;//i2c�ȴ�����
signed int I2C_Wait_Err_Flg = 0;//i2c�ȴ���ѭ�������־
float g_gyroBalance = -311;       //������ƽ��ֵ
float g_gyroIntegration = 0;       //�����ǻ�����

int g_gyroDimension = 65536;       //������ת�����ٶȼ�����

float g_angleResult = 0;           //�Ƕȼ�����
float g_angleResult_last = 0;
float g_angle_temp = 0;

int g_gyroGet = 0;               //�����Ƕ�ȡֵ
signed int g_gyroGet_max = 0;
signed int g_gyroGet_min = 0;
int g_gyro_jolt = 0;
int g_jolt_flag = 0;

signed int g_gyroClearRamp = 0;   //�����Ǽ�����µ���־

int g_gyro_cnt = 0;//����
int g_ramp_judge = -10;//�µ��Ƕ���ֵ

/*=====================================pwm����========================================*/
int Left_Pwm = 0;
int Left_Pwm2 = 0;
int Right_Pwm = 0;
int Right_Pwm2 = 0;
///*=====================================�ٶȿ���========================================*/
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
////�����ٶ�
//int spdExp=0;
//int spdExp0=10;
//int spdExp1 = 0;
//int spdExp2=22;
//int spdExp3=0;
//int spdExp4=0;
//
//float leftExpect; //���������ٶ�
//float rightExpect;//���������ٶ�
//signed int leftSpeedOut; //�����ٶ����
//signed int rightSpeedOut;//�����ٶ����
////�ٶȿ���pi
//float g_speed_p=280;
//float g_speed_i=7;
//signed int lOverflowFlg=0;//�������޷�����������־
//signed int rOverflowFlg=0;//�������޷�����������־
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




//float EncoderCount = 0;			//��������ֵ
//signed int DialSwitchFlag1;            //���뿪��1  ���
//signed int DialSwitchFlag2;            //���뿪��2  
//signed int DialSwitchFlag3;            //���뿪��3
//signed int DialSwitchFlag4;            //���뿪��4
//
//
////==========================���������============================//
//float SteeringMin = 3320;			//�����Сֵ
//float SteeringMid = 4020;			//�����ֵ
//float SteeringMax =4720;			//������ֵ
//float SteerPWMOut = 0;		//������ֵ
//
//float SteeringCtrlP =0;			//������Ʋ���P
//float SteeringCtrlDi = 0;			//��������������
//float SteeringCtrlDo = 0;		//������Ƴ������D
//
//
//
//
//
//float SteeringCtrlP_universe =13;			//������Ʋ���P(���)
//float SteeringCtrlDi_universe = 60;			//��������������D�������
//float SteeringCtrlDo_universe = 60;		//������Ƴ������D�������
//
//
//
//float SteeringCtrlP_circle = 7;			//������Ʋ���P
//float SteeringCtrlDi_circle = 50;			//��������������
//float SteeringCtrlDo_circle = 50;		//������Ƴ������D
//
//
//
//float SteerStartDist = 10;			//ƫ���ȡ����ױ���ʼ����
//float SteerStartDist_universe = 20;			//ƫ���ȡ����ױ���ʼ����
//float SteerEndDist = 50;			//ƫ���ȡ����ױ���ֹ����
//float SteerEndDist_circle = 60;			//ƫ���ȡ����ױ���ֹ����
//float High_pass_paramentA=1;                   //ƫ���ͨϵ��
//float High_pass_paramentB=0;                   //ƫ���ͨϵ��
//
//float  Compensate=1.2;      //ƫ���ϵ��
//
//
//
//
///*=====================================�ٶȿ���========================================*/
//int g_drive_flag=0;
//
//float curSpeed=0;       //
//int lCurSpeed=0;        //�����ٶ�
//int rCurSpeed=0;        //�����ٶ�
//int sidetype=0;         
//int car_start_flag=0;   
//SPEED leftDrv;          
//SPEED rightDrv;         
//int KDet = 20;          //����
//int lock_count = 0;   
////��̥��������
//int g_speed_type=0;
////�����ٶ�
//int spdExp=0;
//int spdExp0=0;         //��ֱ���ٶ�         //40
//int spdExp1=20;         //�ǳ�ֱ���ٶ�       //32
//int spdExp2=30;         //����ٶ�          //27
//int spdExp3=40;          //ͣ��
//int spdExp4=15;         //�ᳵ�ٶ�
//int spdExp5=15;         //���ǰ�����ٶ�
//
//float leftExpect;               //���������ٶ�
//float rightExpect;              //���������ٶ�
//signed int leftSpeedOut;        //�����ٶ����
//signed int rightSpeedOut;       //�����ٶ����
////�ٶȿ���pi
//float g_speed_p = 460;          //P
//float g_speed_i = 7;            //I
//float g_speed_d = 3;            //D
//signed int lOverflowFlg=0;      //�������޷�����������־
//signed int rOverflowFlg=0;      //�������޷�����������־
//
///*�������*/
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
////==========================�µ�����============================//
//unsigned char g_Ramp_Flg = 0;//�µ����ȷ�ϱ�־
//float g_Angle_Result = 0; //�Ƕȼ�����
//float g_Angle_threshold=10;        //����ʶ����ֵ
//float g_Gyro_Balance = -315;               //������ƽ��ֵ
//float g_Gyto_integration = 0;               //�����ǻ�����
//float g_Gyro_Dimension = 65536;                //������ת�����ٶȼ�����
//signed int g_Gyro_Get = 0;                  //�����Ƕ�ȡֵ  
//signed int Ramp_time=0;              //�µ����ʱ��
//
////==========================����·�����============================//
//float Jolt_Open=1;            //����·��ʶ������־
//signed int Jolt_Get=0;            //����������ٶ�
//signed int Jolt_Get_min=0;            //����������ٶ���Сֵ
//signed int Jolt_Get_max=0;            //����������ٶ����ֵ
//signed int Jolt_Get_delta=0;            //����������ٶȼ���
//float Jolt_delta=10000;             //����ʶ����ֵ
//signed int Jolt_Balance=-315;      //����������ٶ�ƽ��ֵ
//unsigned char Jolt_Flag=0;        //������־λ
//float Jolt_Down_Speed=10;           //����·���½��ٶ�
//float Jolt_time=100;            //����·��ʱ��



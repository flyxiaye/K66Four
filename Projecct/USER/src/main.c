#include "headfile.h"
#include "MK60_uart.h"
#include "DFlash.h"
#include "Bluetooth.h"
#include "Menu.h"
#include "isr.h"
#include "Init.h"
#include "Meeting.h"
void SelectVar(int n);

int main(void)
{
	get_clk();
	system_Init();
//        ftm_pwm_init(ftm3, ftm_ch5, 75, 995);
//        ftm_pwm_init(ftm0, ftm_ch4, 16000, 1000);//左前
//	ftm_pwm_init(ftm0, ftm_ch5, 16000, 0);
//	ftm_pwm_init(ftm0, ftm_ch6, 16000, 1000);//右后
//	ftm_pwm_init(ftm0, ftm_ch7, 16000, 0);
	systick_delay_ms(1000);
//	MyFlash_Read(0);
	//dialSwitchFlg3 = gpio_get(DIALSWITCH_PIN3);
	//if (dialSwitchFlg3) g_StartSlave = 1;
	//else g_StartSlave = 0;
	//        gpio_init(A7,GPO,1);
	dialSwitchFlg1 = gpio_get(DIALSWITCH_PIN1); //保护
	dialSwitchFlg2 = gpio_get(DIALSWITCH_PIN2); //电磁模式
	dialSwitchFlg3 = gpio_get(DIALSWITCH_PIN3); //选择参数
	dialSwitchFlg4 = gpio_get(DIALSWITCH_PIN4); //选择参数
	SelectVar((dialSwitchFlg3 << 1) + dialSwitchFlg4);
	while (1)
	{
		if (mt9v032_finish_flag)					//图像接收完成标志位
		{
			mt9v032_finish_flag = 0; //标志位清零
                        //gray2Binary(image[0],image[0]);
			if (1 == g_handle_open)
			{
				if (1)//dialSwitchFlg2)			//摄像头模式
					GetML();
				else
					GetML_Ind();
//				MeetingToImage();
                                int sum = 0;
                                for (int i = 0; i < 188; i++)
                                {
                                  sum += image[DOWN_EAGE][i];
                                }
                                if (sum / 188 < 60)
                                  g_drive_flag = 0;
			}
		}
		if (1)//dialSwitchFlg2)
			CarControl();
		else
			CarControl_Ind();
		//		if (1 == dialSwitchFlg1)
		//		{
		//			//			ips_displayimage032(image[0], 188, 120);
		//			//			ips_displayimage032_zoom(image[0], 188, 120, 240, 135);
//		displayimage032(image[0]);
//					ShowEage();
		//		}
//		if (CircleFlag || Img_RampFlag)
//			gpio_init(A7, GPO, 1);
//		else
//			gpio_init(A7, GPO, 0);
//		if (CircleFlag)
//			gpio_init(D0, GPO, 0);
//		else gpio_init(D0, GPO, 1);
//		if (Img_RampFlag || CircleFlag)
//			gpio_init(D1, GPO, 0);
//		else gpio_init(D1, GPO, 1);
		//UserData();
//		SendData(); //会车
					//		UserData();
		//SendAngle();
		//                LogGetData();
		if (1)
			Menu();
		else displayimage032(image[0]);
		// Bluebooth_Push_Data();
		//ExploreTime();
	}

}

void SelectVar(int n)
{
	switch (n)
	{

	case 1:				//28线稳定走中线
		p_max = 1.75;
		p_min = 1.85;
		Steer_D = 1.99;
		ProSpect = 41;
		Steer_P_CI = 1.90;
		spdBas1 = 28;
		KDet = 20;
		break;
	case 2:			//31线稳妥寻中线
		p_max = 1.80;
		p_min = 1.83;
		Steer_D = 2.15;
		ProSpect = 39;
		Steer_P_CI = 1.80;
		spdBas1 = 31;
		KDet = 20;
		break;
	case 3:				//33线较稳定
		p_max = 1.79;
		p_min = 2.10;
		Steer_D = 2.05;
		ProSpect = 38;
		Steer_P_CI = 1.90;
		spdBas1 = 33;
		KDet = 20;
		break;
	default:
		break;
	}
}
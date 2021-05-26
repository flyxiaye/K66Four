/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,��ɿƼ�?
 * All rights reserved.
 * ��������QQȺ��179029047
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ����������������ҵ��;��?
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������?
 *
 * @file       		isr
 * @company	   		�ɶ���ɿƼ����޹��?
 * @author     		Go For It(1325536866)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK66FX
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-09-19
 ********************************************************************************************************************/



#include "isr.h"
#include "speedC.h"
#include "bluetooth.h"
#include "MK60_uart.h"
#include "Meeting.h"
#include "DelayDist.h"
#include "keyboard.h"


int startcarlinewait = 0;


//-------------------------------------------------------------------------------------------------------------------
//  @brief      PROTA�ж�ִ�к���
//  @return     void   
//  @since      v1.0
//  Sample usage:               ��A�������жϹ����ҷ����жϵ�ʱ����Զ�ִ�иú���?
////-------------------------------------------------------------------------------------------------------------------
void PORTA_IRQHandler(void)
{
	//����жϱ�־��һ�ַ���ֱ�Ӳ����Ĵ�����ÿһλ��Ӧһ������?
	g_Key = Key_Scan();		//键盘获取数�?
	PORTA->ISFR = 0xffffffff;
	//PORTA_FLAG_CLR(A13);
//ʹ�����Ǳ�д�ĺ궨����������жϵ�����?
//PORTA_FLAG_CLR(A1);

}


void PORTC_IRQHandler(void)
{
	//����жϱ�־��һ�ַ���ֱ�Ӳ����Ĵ�����ÿһλ��Ӧһ������?
	//PORTB->ISFR = 0xffffffff;
	//ʹ�����Ǳ�д�ĺ궨����������жϵ�����?
	PORTC_FLAG_CLR(C12);
	VSYNC();
}


void DMA0_IRQHandler(void)
{
	DMA_IRQ_CLEAN(DMA_CH0);
	row_finished();

}
int CircleTime = 0;
void PIT0_IRQHandler(void)
{
	PIT_FlAG_CLR(pit0);
	Mpu_Data_Prepare(0.002);
	My_IMU_update(0.002);
//	get_ind_error();
//	if (g_block_open)
//		inf();
	SpeedControl();
//	if (Img_RampFlag)
//	{
//		JudgeRamp();//ʶ���µ�
//	}
//	if (!g_single_open)
//	{
//		MeetingFour();
//	}
//	else
//	{
//		SingleFour();
//	}
//	if (Ind_CircleOpen)			//判断环岛状�?
//		IndCI();
//	else
//		DistCI();
//	if (Img_BlockFlag)
//	{
//		roadblock3();
//	}
////	if (gpio_get(C10))			//遥控停车
////	{
////		g_drive_flag = 0;
////	}
	Angle_GoIslandSix();
	Dist_ClearIslandSeven();		//清环�?状�        ?
	//if (1 == dialSwitchFlg4)
	//	ClearFlag(&BrokenFlag);
	//else
	//	ClearFlag(&BlockFlag);
//	ftm_pwm_duty(ftm0, ftm_ch6,Left_Pwm);
//	ftm_pwm_duty(ftm0, ftm_ch4,Left_Pwm2);
//	ftm_pwm_duty(ftm0, ftm_ch5,Right_Pwm);
//	ftm_pwm_duty(ftm0, ftm_ch7,Right_Pwm2);
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      UART3�ж�ִ�к���
//  @return     void   
//  @since      v1.0
//  Sample usage:               ��UART3�����жϹ����ҷ����жϵ�ʱ����Զ�ִ�иú���?
//-------------------------------------------------------------------------------------------------------------------
void UART3_RX_TX_IRQHandler(void)
{
	if (UART3->S1 & UART_S1_RDRF_MASK)                                     //�������ݼĴ�����
	{
		//�û���Ҫ������������
		mt9v032_cof_uart_interrupt();
	}
	if (UART3->S1 & UART_S1_TDRE_MASK)                                    //�������ݼĴ�����
	{
		//�û���Ҫ������������

	}
}
void UART4_RX_TX_IRQHandler(void)
{
	if (UART4->S1 & UART_S1_RDRF_MASK)                                     //�������ݼĴ�����
	{
		ReceiveData();
	}
	if (UART4->S1 & UART_S1_TDRE_MASK)                                    //�������ݼĴ�����
	{
		// �û���Ҫ������������

	}
}
/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ������DMA0�жϣ�Ȼ��͵�����ȥ���ĸ���DMA0���жϺ������ƣ��ҵ���дһ�������Ƶĺ�������
void DMA0_IRQHandler(void)
{
	;
}
�ǵý����жϺ������־�?


DMA0_IRQHandler
DMA1_IRQHandler
DMA2_IRQHandler
DMA3_IRQHandler
DMA4_IRQHandler
DMA5_IRQHandler
DMA6_IRQHandler
DMA7_IRQHandler
DMA8_IRQHandler
DMA9_IRQHandler
DMA10_IRQHandler
DMA11_IRQHandler
DMA12_IRQHandler
DMA13_IRQHandler
DMA14_IRQHandler
DMA15_IRQHandler
DMA_Error_IRQHandler
MCM_IRQHandler
FTFL_IRQHandler
Read_Collision_IRQHandler
LVD_LVW_IRQHandler
LLW_IRQHandler
Watchdog_IRQHandler
RNG_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
SPI0_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
CAN0_ORed_Message_buffer_IRQHandler
CAN0_Bus_Off_IRQHandler
CAN0_Error_IRQHandler
CAN0_Tx_Warning_IRQHandler
CAN0_Rx_Warning_IRQHandler
CAN0_Wake_Up_IRQHandler
I2S0_Tx_IRQHandler
I2S0_Rx_IRQHandler
CAN1_ORed_Message_buffer_IRQHandler
CAN1_Bus_Off_IRQHandler
CAN1_Error_IRQHandler
CAN1_Tx_Warning_IRQHandler
CAN1_Rx_Warning_IRQHandler
CAN1_Wake_Up_IRQHandler
Reserved59_IRQHandler
UART0_LON_IRQHandler
UART0_RX_TX_IRQHandler
UART0_ERR_IRQHandler
UART1_RX_TX_IRQHandler
UART1_ERR_IRQHandler
UART2_RX_TX_IRQHandler
UART2_ERR_IRQHandler
UART3_RX_TX_IRQHandler
UART3_ERR_IRQHandler
UART4_RX_TX_IRQHandler
UART4_ERR_IRQHandler
UART5_RX_TX_IRQHandler
UART5_ERR_IRQHandler
ADC0_IRQHandler
ADC1_IRQHandler
CMP0_IRQHandler
CMP1_IRQHandler
CMP2_IRQHandler
FTM0_IRQHandler
FTM1_IRQHandler
FTM2_IRQHandler
CMT_IRQHandler
RTC_IRQHandler
RTC_Seconds_IRQHandler
PIT0_IRQHandler
PIT1_IRQHandler
PIT2_IRQHandler
PIT3_IRQHandler
PDB0_IRQHandler
USB0_IRQHandler
USBDCD_IRQHandler
ENET_1588_Timer_IRQHandler
ENET_Transmit_IRQHandler
ENET_Receive_IRQHandler
ENET_Error_IRQHandler
Reserved95_IRQHandler
SDHC_IRQHandler
DAC0_IRQHandler
DAC1_IRQHandler
TSI0_IRQHandler
MCG_IRQHandler
LPTimer_IRQHandler
Reserved102_IRQHandler
PORTA_IRQHandler
PORTB_IRQHandler
PORTC_IRQHandler
PORTD_IRQHandler
PORTE_IRQHandler
Reserved108_IRQHandler
Reserved109_IRQHandler
SWI_IRQHandler
*/




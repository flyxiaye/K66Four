#include "Bluetooth.h"
#include "common.h"
#include "MK60_uart.h"

uint8 uSendBuf[UartDataNum * 2] = { 0 };
uint8 XmuDataNum = UartDataNum * 2;
extern UART_Type* uart[];

double UartData[9] = { 0 };
SerialPortType SerialPortRx;


void SendUserData()
{
	SendFloat(mid_norm);
	SendFloat(leftcol_norm);
	SendFloat(rightcol_norm);
	SendInt(CircleFlag);
	SendInt(CircleState);
	SendInt(Error_1);
	//SendInt(LightThreshold);
	//SendFloat(ind_left);
	//SendFloat(ind_right);
	//SendFloat(ind_left + ind_right);
	//SendFloat(imu_data.yaw);
	//SendInt(Img_BrokenFlag);
	//SendInt(Img_BlockFlag);
	//SendInt(LeftPnt.ErrRow);
	//SendInt(RightPnt.ErrRow);
	//SendInt(g_inf);
	//SendInt(SteerDuty);
	//SendFloat(leftExpect);
	//SendFloat(rightExpect);
	//SendInt(lCurSpeed);
	//SendInt(rCurSpeed);
}

void BluetoothInt()
{
	uart_init(XmuUARTPort, XmuBR);

	//  set_vector_handler(XmuUARTPortVECTORn,XmuUartHander);   // 设置中断服务函数到中断向量表里
	uart_rx_irq_en(XmuUARTPort);                                 //开串口接收中断
}
/*
向某个通道缓冲区填充数据
chanel：通道
data  ：数据-32768~32767
*/
void push(uint8 chanel, uint16 data)
{
	uSendBuf[chanel * 2] = data / 256;
	uSendBuf[chanel * 2 + 1] = data % 256;
}


void sendDataToScope(void)
{
	uint8 i, sum = 0;
	//使用轮询的方式发送数据，当数据未发送，程序停在此处直到发送完成
	uart_putchar(XmuUARTPort, 251);
	uart_putchar(XmuUARTPort, 109);
	uart_putchar(XmuUARTPort, 37);
	sum += (251);      //全部数据加入校验
	sum += (109);
	sum += (37);
	for (i = 0; i < XmuDataNum; i++)
	{
		uart_putchar(XmuUARTPort, uSendBuf[i]);
		sum += uSendBuf[i];         //全部数据加入校验
	}
	uart_putchar(XmuUARTPort, sum);
}

void BluetoothSendData(void)
{
	push(0, -1);
	sendDataToScope();
}


float DataTest, Datatest1;

//  赋值函数，主要改下面的内容
void UartDebug(void)
{
	DataTest = (float)UartData[0];
	Datatest1 = (float)UartData[1];
}
/*
命令接收函数
CmdNum：0~255号命令
DATA  ：0~255个命令所对应的命令字
暂时不用管
*/
void UartCmd(uint8 CmdNum, uint8 Data)
{
	//LedBlueRev;
}

//串口中断服务函数
void uart4_handler(void)
{
	int i, b, d1;
	uint32 d;
	uint8 ch;

	if (uart[uart4]->RCFIFO != 0)//是接收中断
	{
		uart_getchar(uart4, &ch);                    //无限等待接受1个字节
		SerialPortRx.Data = ch;

		if (SerialPortRx.Stack < UartRxBufferLen)
		{
			SerialPortRx.Buffer[SerialPortRx.Stack++] = SerialPortRx.Data;

			if (SerialPortRx.Stack >= UartRxDataLen  //UartRxDataLen个数为一帧
				&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen] == 0xff //校验字头
				&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen + 1] == 0x55
				&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen + 2] == 0xaa
				&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxDataLen + 3] == 0x10)
			{   //double data 9个通道数据校验
				SerialPortRx.Check = 0;
				b = SerialPortRx.Stack - UartRxDataLen; //起始位
				for (i = b; i < SerialPortRx.Stack - 1; i++)  //除校验位外的位进行校验
				{
					SerialPortRx.Check += SerialPortRx.Buffer[i];//校验
				}

				if (SerialPortRx.Check == SerialPortRx.Buffer[SerialPortRx.Stack - 1])
				{   //校验成功，进行数据解算
					for (i = 0; i < 9; i++)
					{
						d = SerialPortRx.Buffer[b + i * 4 + 4] * 0x1000000L + SerialPortRx.Buffer[b + i * 4 + 5] * 0x10000L + SerialPortRx.Buffer[b + i * 4 + 6] * 0x100L + SerialPortRx.Buffer[b + i * 4 + 7];
						if (d > 0x7FFFFFFF)
						{
							d1 = 0x7FFFFFFF - d;
						}
						else
						{
							d1 = d;
						}
						UartData[i] = d1;
						UartData[i] /= 65536.0;
					}
					UartDebug();  //转去处理，把受到的数据付给变量
				}
				SerialPortRx.Stack = 0;
			}
			else if (SerialPortRx.Stack >= UartRxCmdLen //UartRxDataLen个数为一帧
				&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen] == 0xff
				&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen + 1] == 0x55
				&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen + 2] == 0xaa
				&& SerialPortRx.Buffer[SerialPortRx.Stack - UartRxCmdLen + 3] == 0x77)//cmd
			{
				SerialPortRx.Check = 0;
				b = SerialPortRx.Stack - UartRxCmdLen; //起始位
				for (i = b; i < SerialPortRx.Stack - 1; i++)  //除校验位外的位进行校验
				{
					SerialPortRx.Check += SerialPortRx.Buffer[i];//校验
				}
				if (SerialPortRx.Check == SerialPortRx.Buffer[SerialPortRx.Stack - 1])
				{   //校验成功
					UartCmd(UartCmdNum, UartCmdData);//处理接收到的命令，付给MCU命令变量
				}
				SerialPortRx.Stack = 0;
			}
		}
		else
		{
			SerialPortRx.Stack = 0;
		}
	}
}

//串口发送姿态
void SendAngle(void)
{
	uint8 sendbuff[32] = { 0 };
	uint8_t temp = 0;
	int16_t yaww, rolll, pitchh; //??D?????
	int32_t alt;
	alt = 0;//z_est[0]*100;
	yaww = (int16_t)(imu_data.yaw * 100);
	rolll = (int16_t)(imu_data.rol * 100);
	pitchh = (int16_t)(imu_data.pit * 100);

	sendbuff[0] = 0xaa;
	sendbuff[1] = 0xaa;
	sendbuff[2] = 0x01;
	sendbuff[3] = 12;
	sendbuff[4] = rolll >> 8;
	sendbuff[5] = rolll;
	sendbuff[6] = pitchh >> 8;
	sendbuff[7] = pitchh;
	sendbuff[8] = yaww >> 8;
	sendbuff[9] = yaww;

	sendbuff[10] = alt >> 24;
	sendbuff[11] = alt >> 16;
	sendbuff[12] = alt >> 8;
	sendbuff[13] = alt;

	sendbuff[14] = 0;
	sendbuff[15] = 0;
	sendbuff[16] = 0;
	for (temp = 0; temp < 16; temp++)
	{
		sendbuff[16] += sendbuff[temp];
	}
	uart_putbuff(XmuUARTPort, sendbuff, 17);
}

unsigned char testdatatosend[50];	//发送数据缓存
unsigned char _cnt = 0;
unsigned char _count = 0;
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

void SendFloat(float Data)
{
	testdatatosend[_cnt++] = BYTE3(Data);//将要发送的数据放至发送缓冲区
	testdatatosend[_cnt++] = BYTE2(Data);
	testdatatosend[_cnt++] = BYTE1(Data);
	testdatatosend[_cnt++] = BYTE0(Data);
	_count += 4;
}
void SendInt(int Data)
{
	testdatatosend[_cnt++] = BYTE1(Data);
	testdatatosend[_cnt++] = BYTE0(Data);
	_count += 2;
}


void UserData(void)
{
	_cnt = 0;
	_count = 0;
	static unsigned char Length = 0;
	testdatatosend[_cnt++] = 0xAA;//0xAA为帧头
	testdatatosend[_cnt++] = 0xAA;
	testdatatosend[_cnt++] = 0xF1;
	testdatatosend[_cnt++] = 0;//本字节表示数据长度，这里先=0，函数最后再赋值，这样就不用人工计算长度了

	SendUserData();

	Length = _count;

	testdatatosend[3] = Length;
	unsigned char sum = 0;	//以下为计算sum校验字节，从0xAA也就是首字节，一直到sum字节前一字节
	for (unsigned char i = 0; i < _cnt; i++)
	{
		sum += testdatatosend[i];
	}

	testdatatosend[_cnt++] = sum;	//将sum校验数据放置最后一字节	

	uart_putbuff(XmuUARTPort, testdatatosend, _cnt);//调用发送数据函数
}


/*上位机看图像，感兴趣的可以试一下
#define ImgHeight 200 //????
#define ImgWidth 240 //????
uint8 imgData[ImgHeight*ImgWidth]={0};//?????,??????,??????

void sendCamImgToCamViewer(void)
{
  uint8 i,j;
  uint16 offset;
  uart_putchar(XmuUARTPort,0xFF);//FF,FE,FF,FE 表示一幅图的开始
  uart_putchar(XmuUARTPort,0xFE);
  uart_putchar(XmuUARTPort,0xFF);
  uart_putchar(XmuUARTPort,0xFE);
  for(i = 0;i < ImgHeight;i++)
  {
	offset = i*ImgWidth; //????????
	for(j = 0;j < ImgWidth;j++)
	{
	  uint8 d = imgData[offset + j];
	  if(d > 0xFD) d = 0xFD;
	  uart_putchar(XmuUARTPort,d);
	}
	uart_putchar(XmuUARTPort,0xFE);//FE,FE 表示换行
	uart_putchar(XmuUARTPort,0xFE);
  }
  j=0;
}

*/
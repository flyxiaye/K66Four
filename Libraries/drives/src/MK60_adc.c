/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2017,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：179029047
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		MK60_adc
 * @company	   		成都逐飞科技有限公司
 * @author     		Go For It(1325536866)
 * @version    		v2.0
 * @Software 		IAR 7.7 or MDK 5.17
 * @Target core		MK66FX
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2017-09-19
 ********************************************************************************************************************/
 
 
 
#include "MK60_adc.h"

ADC_Type * const ADCX[] = ADC_BASE_PTRS; //定义两个指针数组保存 ADCN 的地址




void adc_start(ADCn_Ch adcn_ch, ADC_nbit bit)
{
    ADCn adcn = (ADCn)(adcn_ch >> 5) ;
    uint8 ch = (uint8)(adcn_ch & 0x1F);

    //初始化ADC默认配置
    ADCX[adcn]->CFG1 = (0
                        //| ADC_CFG1_ADLPC_MASK         //ADC功耗配置，0为正常功耗，1为低功耗
                        | ADC_CFG1_ADIV(2)              //时钟分频选择,分频系数为 2^n,2bit
                        | ADC_CFG1_ADLSMP_MASK          //采样时间配置，0为短采样时间，1 为长采样时间
                        | ADC_CFG1_MODE(bit)
                        | ADC_CFG1_ADICLK(0)            //0为总线时钟,1为总线时钟/2,2为交替时钟（ALTCLK），3为 异步时钟（ADACK）。
                       );


    ADCX[adcn]->CFG2  = (0
                         //| ADC_CFG2_MUXSEL_MASK       	//ADC复用选择,0为a通道，1为b通道。
                         //| ADC_CFG2_ADACKEN_MASK      //异步时钟输出使能,0为禁止，1为使能。
                         | ADC_CFG2_ADHSC_MASK          //高速配置,0为正常转换序列，1为高速转换序列
                         | ADC_CFG2_ADLSTS(0)           //长采样时间选择，ADCK为4+n个额外循环，额外循环，0为20，1为12，2为6，3为2
                        );

    //写入 SC1A 启动转换
    ADCX[adcn]->SC1[0] = (0
                          | ADC_SC1_AIEN_MASK          // 转换完成中断,0为禁止，1为使能
                          //| ADC_SC1_DIFF_MASK        // 差分模式使能,0为单端，1为差分
                          | ADC_SC1_ADCH( ch )
                         );

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC初始化
//  @param      adcn_ch         选择ADC通道
//  @return     void
//  @since      v1.0
//  Sample usage:               adc_init(ADC0_SE8);
//-------------------------------------------------------------------------------------------------------------------
void adc_init(ADCn_Ch adcn_ch)
{
    uint8 adcn = adcn_ch >> 5 ;

    switch(adcn)
    {
    case adc0:       //   ADC0
        SIM->SCGC6 |= (SIM_SCGC6_ADC0_MASK );        //开启ADC0时钟
        SIM->SOPT7 &= ~(SIM_SOPT7_ADC0ALTTRGEN_MASK  | SIM_SOPT7_ADC0PRETRGSEL_MASK);
        SIM->SOPT7 |= SIM_SOPT7_ADC0TRGSEL(0);
        break;
    case adc1:       //   ADC1
        SIM->SCGC3 |= (SIM_SCGC3_ADC1_MASK );
        SIM->SOPT7 &= ~(SIM_SOPT7_ADC1ALTTRGEN_MASK  | SIM_SOPT7_ADC1PRETRGSEL_MASK) ;
        SIM->SOPT7 |= SIM_SOPT7_ADC1TRGSEL(0);
        break;
    default:
        break;
    }

    switch(adcn_ch)
    {
    case ADC0_SE8:       // B0
        port_init(B0, ALT0);
        break;
    case ADC0_SE9:       // B1
        port_init(B1, ALT0);
        break;
    case ADC0_SE10:      // A7
        port_init(A7, ALT0);
        break;
    case ADC0_SE11:      // A8
        port_init(A8, ALT0);
        break;
    case ADC0_SE12:      // B2
        port_init(B2, ALT0);
        break;
    case ADC0_SE13:      // B3
        port_init(B3, ALT0);
        break;
    case ADC0_SE14:      // C0
        port_init(C0, ALT0);
        break;
    case ADC0_SE15:      // C1
        port_init(C1, ALT0);
        break;
    case ADC0_SE17:      // E24
        port_init(E24, ALT0);
        break;
    case ADC0_SE18:      // E25
        port_init(E25, ALT0);
        break;

    case ADC0_DP0:
    case ADC0_DP1:
    case ADC0_DP3:
    case ADC0_DM0:       // ADC0_DM0
    case ADC0_DM1:       // ADC0_DM1
    case ADC0_SE16:      // ADC0_SE16
    case Temp0_Sensor:   // Temperature Sensor,内部温度测量，可用ADC函数
    case VREFH0:         // 参考高电压,可用ADC函数 ,结果恒为 2^n-1
    case VREFL0:         // 参考低电压,可用ADC函数 ,结果恒为 0
        break;          //这部分管脚不用配置复用

        // ---------------------------------ADC1-------------------------
    case ADC1_DP0:
    case ADC1_DP1:
    case ADC1_DP3:
        break;
    case ADC1_SE4a:      // E0
        port_init(E0, ALT0);
        break;
    case ADC1_SE5a:      // E1
        port_init(E1, ALT0);
        break;
    case ADC1_SE6a:      // E2
        port_init(E2, ALT0);
        break;
    case ADC1_SE7a:      // E3
        port_init(E3, ALT0);
        break;

    case ADC1_SE8:       // B0
        port_init(B0, ALT0);
        break;
    case ADC1_SE9:       // B1
        port_init(B1, ALT0);
        break;
    case ADC1_SE10:      // B4
        port_init(B4, ALT0);
        break;
    case ADC1_SE11:      // B5
        port_init(B5, ALT0);
        break;
    case ADC1_SE12:      // B6
        port_init(B6, ALT0);
        break;
    case ADC1_SE13:      // B7
        port_init(B7, ALT0);
        break;
    case ADC1_SE14:      // B10
        port_init(B10, ALT0);
        break;
    case ADC1_SE15:      // B11
        port_init(B11, ALT0);
        break;
    case ADC1_SE17:      // A17
        port_init(A17, ALT0);
        break;

    case ADC1_SE16:      // ADC1_SE16
    case VREF_OUTPUT:    // VREF Output
    case ADC1_DM0:       // ADC1_DM0
    case ADC1_DM1:       // ADC1_DM1
    case Temp1_Sensor:
    case VREFH1:         // 参考高电压,可用ADC函数 ,结果恒为 2^n-1
    case VREFL1:         // 参考低电压,可用ADC函数 ,结果恒为 0
        break;

    default:
        break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC采集
//  @param      adcn_ch         选择ADC通道
//  @param      bit             选择分辨率ADC_8bit、ADC_10bit、ADC_12bit、ADC_16bit
//  @return     void
//  @since      v1.0
//  Sample usage:               uint16 dat = adc_once(ADC0_SE8,ADC_16bit);//采集ADC0_SE8电压，分辨率16位
//-------------------------------------------------------------------------------------------------------------------
uint16 adc_once(ADCn_Ch adcn_ch, ADC_nbit bit) //采集某路模拟量的AD值
{
    uint16 result = 0;
    ADCn adcn = (ADCn)(adcn_ch >> 5) ;
    adc_start(adcn_ch, bit);      //启动ADC转换
    
    while (( ADCX[adcn]->SC1[0] & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK);   //只支持 A通道
    result = ADCX[adcn]->R[0];
    ADCX[adcn]->SC1[0] &= ~ADC_SC1_COCO_MASK;
    return result;
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ADC停止采集
//  @param      adcn_ch         选择ADC通道
//  @return     void
//  @since      v1.0
//  Sample usage:               adc_stop(adc0);
//-------------------------------------------------------------------------------------------------------------------
void adc_stop(ADCn adcn)
{
    ADCX[adcn]->SC1[0] = (0
                          | ADC_SC1_AIEN_MASK                       // 转换完成中断,0为禁止，1为使能
                          //| ADC_SC1_DIFF_MASK                     // 差分模式使能,0为单端，1为差分
                          | ADC_SC1_ADCH(Module0_Dis)               //输入通道选择,此处选择禁止通道
                         );
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：ad_ave
*  功能说明：多次采样，取平均值
*  参数说明：ADCx        模块号（ ADC0、 ADC1）
*            ADC_Channel 通道号
*            ADC_nbit    精度（ ADC_8bit,ADC_12bit, ADC_10bit, ADC_16bit ）
*            N           均值滤波次数(范围:0~255)
*  函数返回：16位无符号结果值
*  修改时间：2012-2-10
*  备    注：修改苏州大学的例程
*************************************************************************/

uint16 ad_ave(ADCn_Ch adcn_ch, ADC_nbit bit, uint8 N) //均值滤波
{
    uint32 tmp = 0;
    uint8  i;
    //ASSERT( ((adcn == ADC0) && (ch >= AD8 && ch <= AD18)) || ((adcn == ADC1) && (ch >= AD4a && ch <= AD17)) ) ; //使用断言检测ADCn_CHn是否正常

    for(i = 0; i < N; i++)
        tmp += adc_once(adcn_ch,bit);
    tmp = tmp / N;
    return (uint16)tmp;
}
uint16 ad_lvbo(ADCn_Ch adcn_ch, ADC_nbit bit, uint8 N)
{
    uint32 tmp[20], tmp0, tmpSum;
    uint8 i, j;
    
    
    for(i = 0;i < N; i++)
    {
        tmp[i] = ad_ave(adcn_ch, ADC_12bit, 10);
    }
    for(i = 0;i < N; i++)
    {
        for(j = i + 1; j < N; j++)
        {
            if(tmp[i] < tmp[j])
            {
                tmp0 = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = tmp0;
            }
        }
    }
    for(i = 1; i < N-1; i++)
    {
        tmpSum += tmp[i];
    }
    tmpSum = tmpSum / (N - 2);
    return (uint16)tmpSum;
}

/*************************************************************************
*                             野火嵌入式开发工作室
*
*  函数名称：ad_flt
*  功能说明：一次采样后，与前几次采样，求平均值
*  参数说明：ADCx        模块号（ ADC0、 ADC1）
*            ADC_Channel 通道号
*            ADC_nbit    精度（ ADC_8bit,ADC_12bit, ADC_10bit, ADC_16bit ）
*  函数返回：16位无符号结果值
*  修改时间：2012-2-10
*  备    注：
*************************************************************************/
#define SAMP_COUNT  2       //向前滤波 2^SAMP_COUNT 次数  不能太大，不然采样不准
uint16 ad_flt(ADCn_Ch adcn_ch, ADC_nbit bit)
{
    static uint16 buf[(1<<(SAMP_COUNT))] = {0};  //保存前  2^SAMP_COUNT 次 的采样数据
    static uint8 n = (uint8)(0x100 - (char)(1 << (SAMP_COUNT)));
    static uint32 sum = 0;

    //ASSERT( ((adcn == ADC0) && (ch >= AD8 && ch <= AD18)) || ((adcn == ADC1) && (ch >= AD4a && ch <= AD17)) ) ; //使用断言检测ADCn_CHn是否正常

    if(n >= (uint8)(0x100 - (char)(1 << (SAMP_COUNT))))
    {
        buf[(uint8)((1<<(SAMP_COUNT))+n)] = adc_once(adcn_ch,bit);
        sum += buf[(uint8)((1<<(SAMP_COUNT))+n)];
        n++;
        return ((uint16)(sum >> SAMP_COUNT));
    }

    sum -= buf[n];
    buf[n] = adc_once(adcn_ch,bit);
    sum += buf[n];

    if (++n >= (1 << (SAMP_COUNT)))
    {
        n = 0;
    }
    return ((uint16)(sum >> SAMP_COUNT)); /* ADC采样值由若干次采样值平均 */
}

/*自制*/



uint16 collect(ADCn_Ch adcn_ch) //一版
{
  int i,j,k,l;
   uint16 ave[5],max,min,avefinal,sum=0;
 for(i=0;i<5;i++)
 {
    ave[i]=ad_ave(adcn_ch, ADC_12bit, 10);
 }
  max=ave[0];
   for(j=0;j<5;j++)
   {  
     if (ave[j]>max)
      {
        max=ave[j];
      }
   }
  min=ave[0];
   for(k=0;k<5;k++)
   {  
     if (ave[k]<min)
      {min=ave[k];
      }
   }
   sum=0;
   for(l=0;l<5;l++)
   {   
        sum+=ave[l];
   
   }
   
  avefinal=(sum-min-max)/3;
  return avefinal;

}
#undef  SAMP_COUNT
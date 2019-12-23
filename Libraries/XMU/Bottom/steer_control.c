
#include "steer_control.h"
#include "headfile.h"



//================================================================//
//  @brief  :		系统初始化
//  @param  :		void
//  @return :		void
//  @note   :		void
//================================================================//
void Image_Init()
{
        for (int row_i = 0; row_i < IMG_ROWS; row_i++)
        {
                for (int col_i = 0; col_i < IMG_COLS; col_i++)
                {
                        PixelBuff[row_i][col_i] = 0;
                        for (int row_j =row_i*2; row_j < row_i*2+2; row_j++)
                        {
                                for (int col_j = col_i*2; col_j < col_i*2 + 2; col_j++)
                                {
                                        PixelBuff[row_i][col_i] += image[row_j][col_j];
                                }
                        }
                        PixelBuff[row_i][col_i] = PixelBuff[row_i][col_i]/4;
                }
        }
}


void ErrorCal()
{
        int sum[80]={0};
	//列平均去噪
	for (int i = 17; i < IMG_COLS-17; i++)
	{
		sum[i] = PixelBuff[0][i];
		for (int j = 1; j < IMG_ROWS; j++)
		{
			sum[i]= sum[i] + PixelBuff[j][i];
		}
		sum[i] = (int)(sum[i] / 60);
	}
	for (int i = 17; i < IMG_COLS - 17; i++)
	{
		for (int j = 1; j < IMG_ROWS; j++)
		{
			if (PixelBuff[j][i] > sum[i])
			{
				PixelBuff[j][i] =sum[i];
			}
		}
	}
	//行平均去噪
	int x_sum[60] = {0};
	for (int i = 0; i < IMG_ROWS; i++)
	{
		for (int j = 17; j < (IMG_COLS - 17); j++)
		{
			x_sum[i] = x_sum[i] + PixelBuff[i][j];
		}
		x_sum[i] =(int)(x_sum[i] /60 );
	}	
	for (int i = 0; i < IMG_ROWS; i++)
	{
		for (int j = 17; j < IMG_COLS - 17; j++)
		{
			if (PixelBuff[i][j] > x_sum[i])
			{
				PixelBuff[i][j] =x_sum[i];
			}
		}
	}


	//累加
	for (int i = 17; i < IMG_COLS - 17; i++)
	{
		sum[i] = PixelBuff[0][i];
		for (int j = 1; j < IMG_ROWS; j++)
		{
			sum[i] = sum[i] + PixelBuff[j][i];
		}
	}
	//迭代逼近
	sum[17] = 0;
	sum[IMG_COLS - 17] = 0;
	for (int t = 0; t < 10; t++)
	{
		for (int i = 17; i < IMG_COLS - 17; i++)
		{
			sum[i] = (sum[i - 1] + sum[i + 1]) / 2;
		}

	}
	int max;
	max = sum[17];
	int max_position = 0;
	for (int i = 17; i < IMG_COLS-17; i++)
	{
		if (sum[i] > max)
		{
			max = sum[i];
			max_position = i;
		}
	}
        
        
        last_error=error;
	error = (47 - max_position) ;
        if(error>error_max)
        {
          error=error_max;   
        }
        else if(error<-error_max)
        {
          error=-error_max;
        }
        
        
//        if(g_drive_flag!=0)
//        {
//           error_cnt+=error;
//        }
//        else
//        {
//          error_cnt=0;
//        }
//        
//        
//        
//        if(error_cnt>4950)
//        {
//          error=-error;
        
//        }
//        else if(error_cnt<-4950)
//        {
//          error=-error;
//        }
}


void SteerControl()
{
  int D_error=0;
  float P=0;
  float D=0;
  P=g_p;
  D=g_d;
  D_error=last_error;
  
  g_steer_duty = STEERMID +(D * D_error + P * error);
  
  if(g_steer_duty > STEERMAX )
    g_steer_duty = STEERMAX;
  else if(g_steer_duty < STEERMIN)
    g_steer_duty = STEERMIN;
  
  ftm_pwm_duty(ftm3, ftm_ch5,g_steer_duty);
  
}
//void SteerDutyCal(int select_row)
//{
// 
//  int D_error,error = 0;
//  float P = 0;
//  float D;
//  if (select_row < 25)
//    select_row = 25;
//  else if (select_row > 100)
//    select_row = 100;
//  //加权取error
//  for (int i = g_distance2row[select_row - 25];i <= BEHIND_LINE;i++)
//  {
//    if (i < g_end_line)
//      error += g_row_weight[BEHIND_LINE - i] * (TARGET_MID - mid[g_end_line]);
//    else
//      error += g_row_weight[BEHIND_LINE - i] * (TARGET_MID - mid[i]);
//  }
//  error /= BEHIND_LINE - g_distance2row[select_row - 25] + 1;//除以累加次数
//    OLED_Write_Int(4,96,error);
////  error = error* (g_curvature_a * error + g_curvature_b);
//  
//  D_error = error - g_error_last;       //误差差分
//  if (D_error > 30) 
//    D_error = 30;                       //差分限幅
//  else if (D_error < -30) 
//    D_error = -30;                      //差分限幅
//    
//  if(g_steer_type == 1)
//  {
//      P = g_steer_p_straight;           //短直道P
//  }
//  else if (g_steer_type == 2)
//  {
//      P = g_steer_p;                    //弯道P
//  }
//  else if (g_steer_type == 3)
//  {
//      P = g_steer_p_little;             //长直道P
//  }
//  
//  //圆环P
//  if(0==g_flag_circle||4==g_flag_circle)
//  {
//    
//  }
//  else
//  {
//    P=g_steer_p_circle;
//  }
//  
//  if (D_error * error > 0)              //出入弯D
//    D = g_steer_d_in;
//  else
//    D = g_steer_d_out;
//    
//  g_steer_duty = g_steermid +(D * D_error + P * error);     //计算舵机值
//
//  if (error > 0)                        //记录转弯方向
//    g_last_turn = 1;
//  else if (error < 0)
//    g_last_turn = -1;
//  
//  g_error_last = error;
//  g_steer_error = g_steer_duty - g_steermid;
//  
//}
//
//
//void GiveSteerDuty(void)
//{
//  if(g_steer_duty > g_steermax && g_steer_open)
//    g_steer_duty = g_steermax;
//  else if(g_steer_duty < g_steermin && g_steer_open)
//    g_steer_duty = g_steermin;
//  ftm_pwm_duty(ftm3, ftm_ch5,g_steer_duty);
//}
//
//void CarProtect(void)
//{
//       if (g_protect_enable && ImgBuff[IMG_HEIGHT * IMG_DATA - 10] && ImgBuff[IMG_HEIGHT * IMG_DATA - 30] )
//        g_drive_flag = 0;
//}
//
//void SteerControl(void)
//{
//  SteerDutyCal(g_control_row);
//  GiveSteerDuty();
//}
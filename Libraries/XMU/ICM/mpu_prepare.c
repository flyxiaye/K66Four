#include "mpu_prepare.h"


_sensor_st sensor={0,0,0,0};
int32 sum_temp[6]={0,0,0,0,0,0};
uint16 sum_cnt = 0;
float sensor_val[6];
struct _save_param_st_pk Param;

void Mpu_Data_Offset()
{
  if(sensor.acc_calibrate==1 || sensor.gyr_calibrate==1)
  {
    sum_cnt++;
    
    sum_temp[A_X] += sensor_val[A_X];
    sum_temp[A_Y] += sensor_val[A_Y];
    sum_temp[A_Z] += (sensor_val[A_Z]) - 4096;
  
    sum_temp[G_X] += sensor.Gyro_I16.x;
    sum_temp[G_Y] += sensor.Gyro_I16.y;
    sum_temp[G_Z] += sensor.Gyro_I16.z;
    
    if( sum_cnt >= OFFSET_AV_NUM )
    {
      Param.acc_offset.x = sum_temp[A_X]/OFFSET_AV_NUM;
      Param.acc_offset.y = sum_temp[A_Y]/OFFSET_AV_NUM;
      Param.acc_offset.z = sum_temp[A_Z]/OFFSET_AV_NUM;
      
      Param.gyr_offset.x = (float)sum_temp[G_X]/OFFSET_AV_NUM;
      Param.gyr_offset.y = (float)sum_temp[G_Y]/OFFSET_AV_NUM;
      Param.gyr_offset.z = (float)sum_temp[G_Z]/OFFSET_AV_NUM;
      
      sum_cnt =0;   
      sensor.acc_calibrate = 0;
      sensor.gyr_calibrate = 0;
      
      sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = 0;
      sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = 0;
    }
  }
}

/*
传感器默认方向
     +x
     |
 +y--|--
     |

0:默认  
1：传感器顺时针90 度
2：传感器顺时针180度
3：传感器顺时针270度
*/

void sensor_dir(uint8 dir,int16 itx,int16 ity,int16 itz,int16 *it_x,int16 *it_y,int16 *it_z)
{
  switch(dir)
  {
    case 1: //传感器顺时针90度
          *it_x = ity;
          *it_y = -itx;
          *it_z = itz;
    break;
    case 2://传感器顺时针180度
          *it_x = -itx;
          *it_y = -ity;
          *it_z = itz;
    break;
    case 3://传感器顺时针270度
          *it_x = -ity;
          *it_y = itx;
          *it_z = itz;
    break;
    default://传感器默认方向
          *it_x = itx;
          *it_y = ity;
          *it_z = itz;			
    break;
  }
	
}

#define MPU_WINDOW_NUM 5
#define MPU_STEEPEST_NUM 5

#define MPU_WINDOW_NUM_ACC 20
#define MPU_STEEPEST_NUM_ACC 20

_steepest_st steepest_ax;
_steepest_st steepest_ay;
_steepest_st steepest_az;
_steepest_st steepest_gx;
_steepest_st steepest_gy;
_steepest_st steepest_gz;

int32 steepest_ax_arr[MPU_WINDOW_NUM_ACC];
int32 steepest_ay_arr[MPU_WINDOW_NUM_ACC];
int32 steepest_az_arr[MPU_WINDOW_NUM_ACC];
int32 steepest_gx_arr[MPU_WINDOW_NUM];
int32 steepest_gy_arr[MPU_WINDOW_NUM];
int32 steepest_gz_arr[MPU_WINDOW_NUM];

void Mpu_Data_Prepare(float T)
{	
  _xyz_s16_st temp_acc,temp_gyr;
  
  //读取传感器数据
  icm20602_Read();
  
  /*合成加速度原始数据*/
  temp_acc.x = Accel.X;
  temp_acc.y = Accel.Y;
  temp_acc.z = Accel.Z;
  
  /*合成角速度原始数据*/
  temp_gyr.x = Gyro.X;
  temp_gyr.y = Gyro.Y;
  temp_gyr.z = Gyro.Z;
  
//  sensor.Tempreature = icm20602_GetDoubleData(MPUREG_TEMP_OUT_H); 
//  sensor.Ftempreature = sensor.Tempreature/326.8f + 25;
  
  /*梯度下降拟合数据*/
  steepest_descend(steepest_ax_arr ,MPU_WINDOW_NUM_ACC ,&steepest_ax ,MPU_STEEPEST_NUM_ACC,(int32)temp_acc.x);
  steepest_descend(steepest_ay_arr ,MPU_WINDOW_NUM_ACC ,&steepest_ay ,MPU_STEEPEST_NUM_ACC,(int32)temp_acc.y);
  steepest_descend(steepest_az_arr ,MPU_WINDOW_NUM_ACC ,&steepest_az ,MPU_STEEPEST_NUM_ACC,(int32)temp_acc.z);
  steepest_descend(steepest_gx_arr ,MPU_WINDOW_NUM ,&steepest_gx ,MPU_STEEPEST_NUM,(int32)temp_gyr.x);
  steepest_descend(steepest_gy_arr ,MPU_WINDOW_NUM ,&steepest_gy ,MPU_STEEPEST_NUM,(int32)temp_gyr.y);
  steepest_descend(steepest_gz_arr ,MPU_WINDOW_NUM ,&steepest_gz ,MPU_STEEPEST_NUM,(int32)temp_gyr.z);
  
  /*传感器方向调整*/
  sensor_dir( 0,steepest_ax.now_out,steepest_ay.now_out,steepest_az.now_out,
                                                          &sensor.Acc_I16.x ,&sensor.Acc_I16.y ,&sensor.Acc_I16.z );
                                                          
  sensor_dir( 0,steepest_gx.now_out,steepest_gy.now_out,steepest_gz.now_out,
                                                          &sensor.Gyro_I16.x ,&sensor.Gyro_I16.y ,&sensor.Gyro_I16.z );
  
  /*传感器零偏校准*/
  Mpu_Data_Offset();
  
  /*加速度六面校准*/
  //Acc_Six_surface();
  
  /*加速度六面校准后的数据*/
//  sensor_val[A_X] = sensor.Acc_I16.x*Param.acc_k.x - Param.acc_b.x;
//  sensor_val[A_Y] = sensor.Acc_I16.y*Param.acc_k.y - Param.acc_b.y;
//  sensor_val[A_Z] = sensor.Acc_I16.z*Param.acc_k.z - Param.acc_b.z;
  
  /*加速度零偏校准后的数据*/
//  sensor.Acc.x  = (float)sensor_val[A_X] - Param.acc_offset.x;
//  sensor.Acc.y  = (float)sensor_val[A_Y] - Param.acc_offset.y;
//  sensor.Acc.z  = (float)sensor_val[A_Z] - Param.acc_offset.z;
  
//  sensor.Acc.x  = sensor.Acc_I16.x - Param.acc_offset.x;
//  sensor.Acc.y  = sensor.Acc_I16.y - Param.acc_offset.y;
//  sensor.Acc.z  = sensor.Acc_I16.z - Param.acc_offset.z;
  sensor.Acc.x  = sensor.Acc_I16.x;
  sensor.Acc.y  = sensor.Acc_I16.y;
  sensor.Acc.z  = sensor.Acc_I16.z;
  
  /*角速度零偏校准后的数据*/
  sensor_val[G_X] = sensor.Gyro_I16.x - Param.gyr_offset.x ;
  sensor_val[G_Y] = sensor.Gyro_I16.y - Param.gyr_offset.y ;
  sensor_val[G_Z] = sensor.Gyro_I16.z - Param.gyr_offset.z ;
  
  sensor.Gyro.x = (float)sensor_val[G_X];
  sensor.Gyro.y = (float)sensor_val[G_Y];
  sensor.Gyro.z = (float)sensor_val[G_Z];
  
  /*加速度计转换到毫米每平方秒*/
  sensor.Acc_mmss.x = sensor.Acc.x *2.392615;
  sensor.Acc_mmss.y = sensor.Acc.y *2.392615;
  sensor.Acc_mmss.z = sensor.Acc.z *2.392615;
  
  /*陀螺仪转换到度每秒*/
  sensor.Gyro_deg.x = sensor.Gyro.x *0.0610361f;
  sensor.Gyro_deg.y = sensor.Gyro.y *0.0610361f;
  sensor.Gyro_deg.z = sensor.Gyro.z *0.0610361f;
  
//  /*角速度积分*/
//  flip.angle_x += sensor.Gyro_deg.x*T;
//  flip.angle_y -= sensor.Gyro_deg.y*T;
}





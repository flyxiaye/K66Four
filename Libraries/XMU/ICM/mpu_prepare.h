#ifndef _MPU_PREPARE_H_
#define	_MPU_PREPARE_H_

#include "headfile.h"

#define OFFSET_AV_NUM 50

enum
{
  A_X = 0,
  A_Y ,
  A_Z ,
  G_X ,
  G_Y ,
  G_Z ,
};


typedef struct
{
  float x;
  float y;
  float z;
} _xyz_f_st;

typedef struct
{
  int16 x;
  int16 y;
  int16 z;
} _xyz_s16_st;

typedef struct 
{
  uint8 six_calibrate;
  uint8 acc_calibrate;
  uint8 gyr_calibrate;
  uint8 mag_calibrate;
  
  _xyz_s16_st Acc_I16;
  _xyz_s16_st Gyro_I16;
  
  _xyz_f_st Acc;
  _xyz_f_st Acc_test;
  _xyz_f_st Acc_mmss;
  _xyz_f_st Gyro;
  _xyz_f_st Gyro_deg;
  
  _xyz_f_st Mag;
  float Mag_L;
  float W_acc;
  
  int16 Tempreature;
  float Ftempreature;
	
}_sensor_st;

typedef struct
{
  float x;
  float y;
  float z;
} _xyz_f_st_pk;

struct _save_param_st_pk
{	
  _xyz_f_st_pk acc_offset;
  _xyz_f_st_pk gyr_offset;
};



void sensor_dir(uint8 dir,int16 itx,int16 ity,int16 itz,int16 *it_x,int16 *it_y,int16 *it_z);
void Mpu_Data_Prepare(float T);
extern _sensor_st sensor;
extern float sensor_val[6];



#endif


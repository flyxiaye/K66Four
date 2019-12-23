#ifndef __XMU_IMU_H
#define __XMU_IMU_H

#include "headfile.h"

typedef struct
{
  float x;
  float y;
  float z;
} _xyz_f_st0;

typedef struct
{
  float w;//q0;
  float x;//q1;
  float y;//q2;
  float z;//q3;
  
  _xyz_f_st0 x_vec;
  _xyz_f_st0 y_vec;
  _xyz_f_st0 z_vec;
  
  _xyz_f_st0 a_acc;
  _xyz_f_st0 w_acc;
  _xyz_f_st0 h_mag;
  
  float rol;
  float pit;
  float yaw;
  
  float fus_rol;
  float fus_pit;
  float fus_yaw;
  
  float inter_rol;
  float inter_pit;
  float inter_yaw;
  
  float angle;
} _imu_st ;
extern _imu_st imu_data;

void My_IMU_update(float dT);
	
#endif
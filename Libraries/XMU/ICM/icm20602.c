#include "icm20602.h"
#include "headfile.h"
#include "MySPI.h"


#define DELAY_MS(ms)    systick_delay_ms(ms)
uint8 icm20602_Init(void)
{
  
  spi_init(icm20602_SPI, icm20602_CS, MASTER,10*1000*1000); 
  icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x80);
  DELAY_MS(10);
  icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x01);
  DELAY_MS(10);
  
  uint8 tmp = icm20602_readbyte(MPUREG_WHOAMI);
//  if(tmp != MPU_WHOAMI_20602)
//  {
//    return 1;
//  }
  
  /*复位reg*/
  icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET,0x03);
  DELAY_MS(10);
  /*复位reg*/
  icm20602_writebyte(MPU_RA_USER_CTRL,0x01);	
  DELAY_MS(10);
  
  icm20602_writebyte(0x70,0x40);//dmp 
  DELAY_MS(10);
  icm20602_writebyte(MPU_RA_PWR_MGMT_2,0x00);
  DELAY_MS(10);
  icm20602_writebyte(MPU_RA_SMPLRT_DIV,0); //1000Hz
  DELAY_MS(10);
  
  icm20602_writebyte(MPU_RA_CONFIG,ICM20602_LPF_20HZ);
  DELAY_MS(10);
  icm20602_writebyte(MPU_RA_GYRO_CONFIG,(3 << 3));
  DELAY_MS(10);
  icm20602_writebyte(MPU_RA_ACCEL_CONFIG,(2 << 3));
  DELAY_MS(10);
  /*加速度计LPF 20HZ*/
  icm20602_writebyte(0X1D,0x04);
  DELAY_MS(10);
  /*关闭低功耗*/
  icm20602_writebyte(0X1E,0x00);
  DELAY_MS(10);
  /*关闭FIFO*/
  icm20602_writebyte(0X23,0x00);
  DELAY_MS(10);
  
  return 0;
}

XYZ Accel,Gyro;

void icm20602_Read()
{
#if i2c
  Accel.X = icm_get(ICM20602_ACCEL_XOUT_H);
  Accel.Y = icm_get(ICM20602_ACCEL_YOUT_H);
  Accel.Z = icm_get(ICM20602_ACCEL_ZOUT_H);
  
  Gyro.X = icm_get(ICM20602_GYRO_XOUT_H);
  Gyro.Y = icm_get(ICM20602_GYRO_YOUT_H);
  Gyro.Z = icm_get(ICM20602_GYRO_ZOUT_H);  
//    Accel.X = GetData(ACCEL_XOUT_H);
//    Accel.Y = GetData(ACCEL_YOUT_H);
//    Accel.Z = GetData(ACCEL_ZOUT_H);
//    
//    Gyro.X = GetData(GYRO_XOUT_H);
//    Gyro.Y = GetData(GYRO_YOUT_H);
//    Gyro.Z = GetData(GYRO_ZOUT_H);  
#else
  Accel.X = icm20602_GetDoubleData(MPUREG_ACCEL_XOUT_H);
  Accel.Y = icm20602_GetDoubleData(MPUREG_ACCEL_YOUT_H);
  Accel.Z = icm20602_GetDoubleData(MPUREG_ACCEL_ZOUT_H);
  
  Gyro.X = icm20602_GetDoubleData(MPUREG_GYRO_XOUT_H);
  Gyro.Y = icm20602_GetDoubleData(MPUREG_GYRO_YOUT_H);
  Gyro.Z = icm20602_GetDoubleData(MPUREG_GYRO_ZOUT_H);  
#endif
}
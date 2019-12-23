#ifndef _MY_SPI_H
#define _MY_SPI_H

#include "headfile.h"


#define icm20602_SPI spi2
#define icm20602_CS  SPI_PCS0


void icm20602_writebyte(uint8 address,uint8 data);
uint8 icm20602_readbyte(uint8 address);
int16 icm20602_GetDoubleData(uint8 address);






#endif


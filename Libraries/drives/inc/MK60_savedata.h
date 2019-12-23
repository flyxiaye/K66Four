#ifndef __savedata_H__
#define __savedata_H__
#include "common.h"
#define DFLASHN 64

void DFlash_Write_Float(uint16 Block_Now,unsigned int Order,float Write_Float);
float DFlash_Read_Float(uint16 Block_Now,unsigned int Order);
void DFlash_Write_Int(uint16 Block_Now,unsigned int Order,signed int Write_Int);
signed int DFlash_Read_Int(uint16 Block_Now,unsigned int Order);
extern uint16	SECTOR_NUM; //取值为0-127，一个sector为2k

//自己定义的flash地址分配
#define Parameter_Block         200          
#define Parameter_One           123
#define Parameter_Two           124
#define Parameter_Three         125
#define Parameter_Four          126
#define Parameter_Five          122
#define Parameter_Six          121
#define Parameter_Seven        120
#define Track_Block1            119
#define Track_Block2            118
#define Track_Block3            117



#define	flag_place	(DFLASHN*4+4)	
#define	saved_flag	125	

#endif
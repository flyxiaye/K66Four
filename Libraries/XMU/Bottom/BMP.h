#ifndef __BSP_BMP_H
#define	__BSP_BMP_H


#include "headfile.h"

int  Screen_Shot( uint16_t x, uint16_t y, uint16_t Width, uint16_t Height, char * filename );
int SD_Gather_Gray_Picture120x188(void);
void SD_BMP_Init(void);



#endif /* __BSP_BMP_H */


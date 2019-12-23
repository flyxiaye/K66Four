#ifndef __CAMERADISPLAY_H__
#define __CAMERADISPLAY_H__

#include "OLED.h" 
           

void Screen_64x80Transform8x80(unsigned char img64x80[], unsigned char img8x80[]);
void Dis_Play_Data8x80(unsigned char arr[], unsigned position);
void Dis_Camera_Picture120x188(unsigned char *img,unsigned char position);
void Dis_Camera_Picture128x320(unsigned char img[], unsigned char position);

void DisPlayEage(void);

void DispalyBinary(uint8 *p, uint16 position);
void DisPlayImage(void);

#endif
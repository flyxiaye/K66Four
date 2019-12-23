#include "OTSU.h"
#include "GlobalVariate.h"
#include "math.h"

/*

unsigned char otsuThreshold2(uint8 *image, uint16 col, uint16 row,uint8 sparse)
{
    #define GrayScale 256
    uint16 width = col;
    uint16 height = row;
    int pixelCount[GrayScale];
    float pixelPro[GrayScale];
    int i, j, pixelSum = ROW*COL;//width * height/(sparse*sparse);
    uint8 threshold = 0;
    uint8* data = image;  //指向像素数据的指针
    for (i = 0; i < GrayScale; i++)
    {
        pixelCount[i] = 0;
        pixelPro[i] = 0;
    }

    //统计灰度级中每个像素在整幅图像中的个数  
    for (i = 0; i < height; i+=sparse)
    {
        for (j = 0; j < width; j+=sparse)
        {
            pixelCount[(int)data[i * width + j]]++;  //将像素值作为计数数组的下标
        }
    }
    //pixelCount[0]=20*188;

    //计算每个像素在整幅图像中的比例  
    float maxPro = 0.0;
    for (i = 0; i < GrayScale; i++)
    {
        pixelPro[i] = (float)pixelCount[i] / pixelSum;
        if (pixelPro[i] > maxPro)
        {
            maxPro = pixelPro[i];
        }
    }

    //遍历灰度级[0,255]  
    float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
    float w0_process,w1_process,u0tmp_process,u1tmp_process;
    for (i = threshold_down; i<threshold_up; i++)     // i作为阈值
    {
        w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
        if(i==threshold_down)
        {
        for (j = 0; j < GrayScale; j++)
        {
            if (j <= i)   //背景部分  
            {
                w0 += pixelPro[j];
                u0tmp += j * pixelPro[j];
            }
            else   //前景部分  
            {
                w1 += pixelPro[j];
                u1tmp += j * pixelPro[j];
            }
           
        }
            w0_process=w0;
            w1_process=w1;
            u0tmp_process=u0tmp;
            u1tmp_process=u1tmp;
        }
        else
        {
          w0=w0_process+pixelPro[i];
          w1=w1_process-pixelPro[i];
          u0tmp=u0tmp_process+i*pixelPro[i];
          u1tmp=u1tmp_process-i*pixelPro[i];
          w0_process=w0;
          w1_process=w1;
          u0tmp_process=u0tmp;
          u1tmp_process=u1tmp;
        }
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        u = u0tmp + u1tmp;
        deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
        if (deltaTmp > deltaMax)
        {
            deltaMax = deltaTmp;
            threshold = i;
        }
    }

    return threshold;
}*/
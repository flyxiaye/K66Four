#include "MySPI.h"

/*!
 *  @brief      icm20602+写寄存器
 *  @param      address   寄存器地址
 *  @param      data        需要写入的数据
 *  @return     无
 *  @since      XMU-智能车队
 *  Sample usage:    
 */
void icm20602_writebyte(uint8 address,uint8 data)
{
  spi_mosi_cmd(icm20602_SPI, icm20602_CS, &address, NULL, &data, NULL, 1 , 1); 
}

/*!
 *  @brief      icm20602+读寄存器
 *  @param      address   寄存器地址
 *  @return     读到的数据
 *  @since      XMU-智能车队
 *  Sample usage:       
 */
uint8 icm20602_readbyte(uint8 address)
{
  uint8 data;
  uint8 address_read = address | 0x80;
  spi_mosi_cmd(icm20602_SPI, icm20602_CS, &address_read , NULL, NULL, &data, 1 , 1);
  return data;
}

/*!
 *  @brief      icm20602+读取数据+数据合成
 *  @param      address   寄存器
 *  @return     合成的数据
 *  @since      XMU-智能车队
 *  Sample usage:    
 */

int16 icm20602_GetDoubleData(uint8 address)
{
  uint16 data=0x0000;
  data=icm20602_readbyte(address);
  data=(uint16)((data<<8)&0xff00);                              //icm20602_...._H的8位数据左移8位
  data+=icm20602_readbyte(address+1); 
  return (int16)data;//合成数据，为有符号整形数  icm20602_...._H与icm20602_...._L共同组成16位数据
}
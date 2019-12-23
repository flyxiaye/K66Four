#include "MK60_savedata.h"
#include "MK60_flash.h"
uint16	SECTOR_NUM=Parameter_Block;
      
/////////////////////////////////////////////////////////
/*void save(int t[DFLASHN])
  {
    DisableInterrupts;
    
    flash_erase_sector(Block_Now);		
   
    for(int i=0;i<DFLASHN;i++)
    {
        flash_write(Block_Now,i*4,t[i]);
    }
    flash_write(Block_Now,flag_place,saved_flag);
    EnableInterrupts;	
  }

void load(int t[DFLASHN])
  {
    DisableInterrupts;
    
    if(saved_flag==flash_read(Block_Now,flag_place,uint32))
    {
     for(int i=0;i<DFLASHN;i++)
        {
            t[i]=flash_read(Block_Now,i*4,uint32);	//读取4字节
        }
    }    
    else
    {
       save(t);
    }
   EnableInterrupts;	
  }
*/
void DFlash_Write_Float(uint16 Block_Now,unsigned int Order,float Write_Float)   
{

  union
  {
    float Temp_Data;
    uint8  Data_Part[8];
  }_Float_;
  
  _Float_.Temp_Data=Write_Float;
  
  FLASH_WriteSector(Block_Now,_Float_.Data_Part,Order*8);
  
}

/*-----------------*DFLASH读取浮点数*-----------------*/

float DFlash_Read_Float(uint16 Block_Now,unsigned int Order)
{
  union
  {
    float Temp_Data;
    uint64 Data_Part;
  }_Float_;
  
  _Float_.Data_Part=flash_read(Block_Now,Order*8,uint64);

  return(_Float_.Temp_Data);
}
    
/*---------------------*DFLASH写整数*---------------------*/ 
    

void DFlash_Write_Int(uint16 Block_Now,unsigned int Order,signed int Write_Int)   
{
    union
  {
    signed int Temp_Data;
    uint8  Data_Part[8];
  }_Int_;
  _Int_.Temp_Data=Write_Int; 

  FLASH_WriteSector(Block_Now,_Int_.Data_Part,Order*8);
}
/*---------------------*DFLASH读整数*---------------------*/ 

signed int DFlash_Read_Int(uint16 Block_Now,unsigned int Order)
{
  return flash_read(Block_Now,Order*8,uint64);
}



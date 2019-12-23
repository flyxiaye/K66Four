#include  "common.h"
#include  "MK60_gpio.h"
#include  "keyboard.h"
#include "headfile.h"

char g_Key = -1;


//================================================================//
//  @brief  :		键盘初始化
//  @param  :		void
//  @return :		void
//  @note   :		在Key_Function里最后面调用
//================================================================//
void INTR_Keyboard_Init(void)
{
	gpio_init(COL1, GPO, 0);
	gpio_init(COL2, GPO, 0);
	gpio_init(COL3, GPO, 0);
	port_init(ROW1, IRQ_FALLING | PF | ALT1 | PULLUP);
	port_init(ROW2, IRQ_FALLING | PF | ALT1 | PULLUP);
	port_init(ROW3, IRQ_FALLING | PF | ALT1 | PULLUP);
	enable_irq(PORTA_IRQn);
	g_Key = 0;
}

//*      Key_Scan(void)
//*	@brief:		获取矩阵键盘的引脚状态，每个按键返回特定的值
//*	@param:		void
//*	@return:	1~9 未按下键盘时返回-1 
//* @note:    void
//*
int Key_Scan(void)
{   
    char Scan_Result;
    gpio_init(ROW1,GPO,0);
    gpio_init(ROW2,GPO,0);
    gpio_init(ROW3,GPO,0);
    gpio_init(COL1,GPI,1);
    gpio_init(COL2,GPI,1);
    gpio_init(COL3,GPI,1);
    
    port_init_NoAlt(COL1,PULLUP);
    port_init_NoAlt(COL2,PULLUP);
    port_init_NoAlt(COL3,PULLUP);
    
    Scan_Result=gpio_get(COL1);
    Scan_Result+=gpio_get(COL2)<<1;    
    Scan_Result+=gpio_get(COL3)<<2;
    
    if(Scan_Result != 0x07)
    {
        systick_delay_ms(2);
        Scan_Result=Scan_Result<<4;
        gpio_init(COL1,GPO,0);
        gpio_init(COL2,GPO,0);
        gpio_init(COL3,GPO,0);
        gpio_init(ROW1,GPI,1);
        gpio_init(ROW2,GPI,1);
        gpio_init(ROW3,GPI,1);
        
        port_init_NoAlt(ROW1,PULLUP);
        port_init_NoAlt(ROW2,PULLUP);
        port_init_NoAlt(ROW3,PULLUP);
        
        systick_delay_ms(1);
        Scan_Result+=gpio_get(ROW1);
        systick_delay_ms(1);
        Scan_Result+=gpio_get(ROW2)<<1;
        systick_delay_ms(1);
        Scan_Result+=gpio_get(ROW3)<<2;
        
        switch(Scan_Result)
        { 
        case 0x33: return 3;break;
        case 0x35: return 6;break;
        case 0x36: return 9;break;
        case 0x53: return 2;break;
        case 0x55: return 5;break;
        case 0x56: return 8;break;
        case 0x63: return 1;break;
        case 0x65: return 4;break;
        case 0x66: return 7;break; 
        default :break;
        }       
    } 
    return -1;
}


//*    Key_Check(void)
//*	@brief:		检测上升沿
//*	@param:		void
//*	@return:	刚按下去时返回实际健值 未按下或按下不放时返回-1； 
//* @note:    void
//*


int Key_Check(void)
{
    char last_Key;
    static char temp_Key = -1;

    last_Key = temp_Key;
    temp_Key = (char)Key_Scan();
    if(temp_Key != last_Key)
    {
        return last_Key;
    }
    return -1;         
}





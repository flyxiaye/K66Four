#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#define ROW1 A13
#define ROW2 A5
#define ROW3 A6

#define COL1 E28
#define COL2 E26
#define COL3 E27

int Key_Scan(void);
int Key_Check(void);
void INTR_Keyboard_Init(void);
extern char g_Key;


#endif
#ifndef __MENU_H__
#define __MENU_H__


#define GIVEN_PAGE 1

void FlagChange(int * flag);
void Input_Password(void);
void Dis_Power(void);

void Insert_Page(signed char String[]);
void Insert_Img(signed char String[],void pfunc_show(void));
void Insert_Char(signed char String[],signed char *Number);
void Insert_Int(signed char String[],signed int *Number);
void Insert_Float(signed char String[],float *Number);
void Insert_VFloat(signed char String[],volatile float *Number);

void Menu(void);

void Key_Function(void);
void Extra_Show(void);
void Main_Show(void);

void ExploreTime(void);








#endif
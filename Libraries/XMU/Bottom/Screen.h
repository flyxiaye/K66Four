#ifndef __PINGMU_H
#define __PINGMU_H 

extern signed char Temp_List[128];
extern float Battery;
#define Cross 48
#define Row 128


/******************************************************/
/********************  生成接口    ********************/
/******************************************************/


/*--------------------*屏幕头显示*--------------------*/

void Dis_Head   (signed char String[]);
// 在标题栏居中显示字符串，调用格式Dis_Head("Head")

/*--------------------*显示字符串*--------------------*/

void Dis_String (unsigned char Position,signed char String[]);
// 二三四行的左右栏，依次分为1,2,3,4,5,6。靠右显示
// 调用格式Dis_String(1,"String")

/*--------------------*显示整型数*--------------------*/      

void Dis_Int    (unsigned char Position,signed int Number);
// 调用格式Dis_Int(2,123)或Dis_Int(3,Temp_Int)

/*--------------------*显示浮点型*--------------------*/

void Dis_Float  (unsigned char Position,float Number);
// 调用格式Dis_Float(3,12.34)或Dis_Float(4,Temp_Float)

/*---------------------*显示电量*---------------------*/
    
extern void Dis_Battery(void);
// 在右上角显示对Battery变量进行处理之后的电量值
// 在820时显示满电量，在720时为空
                             
/*-------------------*列表更新显示*-------------------*/      
   
void Paint_Fresh(float New_Data,float Paint_Ave,float Paint_Margin);
// 将数据从左到右显示在图上，达到屏幕最右端时，清空图形，重新从左开始
// 调用格式Paint_Fresh(Data,0,100)
// 表示画出Data这个变量的图形，图像以0为中线，显示正负100以内的数据

/*-------------------*递推显示数据*-------------------*/      
  
void Paint_List (float New_Data,float Paint_Ave,float Paint_Margin);
// 将数据显示，达到屏幕最右端时，将其他数据向左推
// 调用格式Paint_List(Data,0,50)
  
  
 
  
/******************************************************/
/********************  功能函数    ********************/
/******************************************************/

extern char Change_To_Bit[8];

/*-----------------*OLED写七位字符串*-----------------*/

void OLED_Write_String_7(unsigned char y,unsigned char x,signed char String[]);


/*---------------------*清空位置*---------------------*/

void Position_Clear(unsigned char Position);


/*--------------------*屏幕头显示*--------------------*/

void Dis_Head(signed char String[]);


/*--------------------*显示字符串*--------------------*/

void Dis_String(unsigned char Position,signed char String[]);


/*--------------------*显示整型数*--------------------*/      

void Dis_Int(unsigned char Position,signed int Number);
void Dis_Int_Flg(unsigned char Position,signed int Number);

/*--------------------*显示浮点型*--------------------*/

void Dis_Float(unsigned char Position,float Number);


/*-------------------*列表更新显示*-------------------*/      
   
void Paint_Fresh(float New_Data,float Paint_Ave,float Paint_Margin);
// 将某个数据以列表刷新的形式显示出来


/*---------------------*列表移动*---------------------*/      

void List_Move(signed char New_Data);
// 对数组进行移位

/*-------------------*显示列表图形*-------------------*/      

void List_Paint(void);



// 将Temp_List列表里面的数据用一整幅图画出来





/*-------------------*递推显示数据*-------------------*/     


  
#endif  
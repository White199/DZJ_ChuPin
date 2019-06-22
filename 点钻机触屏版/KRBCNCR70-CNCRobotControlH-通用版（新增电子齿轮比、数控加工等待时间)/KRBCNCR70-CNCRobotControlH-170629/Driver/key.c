/******************** (C) COPYRIGHT 2012 ********************
---------------------------------------------------------------------------------------------------------
*********************************************************************************************************
** 文件名称 ：	KEY.c
** 功能说明 ：	4*7按键功能定义及按键扫描
** 使用说明 ：  文件适用于STM32F103硬件平台
*********************************************************************************************************
---------------------------------------------------------------------------------------------------------
*/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "key.h"
#include "Usart1_Robot.h"	 
#include "DGUS.h"
#include "Parameter.h"
#include "PlatformInit.h"
#include "manuloperate.h"


u16 keyTempValue,keyValueStatus=0;
u16 keyFinishFlag = 0;
u16 g_Key_Value=0;
u8  Key_Pressed=FALSE;
u8  g_Key_Scan=TRUE;
u8  g_Key_Scan_Count=0;
u8  Key_Scan_Delay_Count=0;
u8  g_LCD_Off_Flag = FALSE;
u32 g_LCD_Off_Timer = 0;
static u16  Key_Link_Move_Flag=0;

void KeyInit(void);
void KeyOutput7(u16 key);
u16 KeyInput4(void);
u16 KeyScan(void);
u16 KeyValue2Physics(u16 keyValueTemp);
										
/*
*********************************************************************************************************
** 函数名称 ：KeyInit()  
** 函数功能 ：按键引脚初始化
** 入口参数 ：无
** 出口参数 ：无
*********************************************************************************************************
*/
void KeyInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 |GPIO_Pin_7 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
}

/*
*********************************************************************************************************
** 函数名称 ：KeyOutput7()  
** 函数功能 ：按键扫描输出
** 入口参数 ：扫描列key
** 出口参数 ：无
*********************************************************************************************************
*/
void KeyOutput7(u16 key)
{
    u16 ReadValue;
    ReadValue = GPIO_ReadOutputData(GPIOA);
	ReadValue =	(ReadValue&0xFF80)|(key&0x7F);
    GPIO_Write(GPIOA, ReadValue);      
}

/*
*********************************************************************************************************
** 函数名称 ：KeyOutput4()  
** 函数功能 ：按键扫描输出 wx
** 入口参数 ：扫描列key	 B12~B15
** 出口参数 ：无
*********************************************************************************************************
*/
void KeyOutput4(u16 key)
{
    u16 ReadValue;
    ReadValue = GPIO_ReadOutputData(GPIOB);
	ReadValue =	(ReadValue&0x0FFF)|((key<<12)&0xF000);
    GPIO_Write(GPIOB, ReadValue);      
}

/*
*********************************************************************************************************
** 函数名称 ：KeyInput4()  
** 函数功能 ：按键扫描输入 wx
** 入口参数 ：扫描横key	 C6~C9
** 出口参数 ：无
*********************************************************************************************************
*/
u16 KeyInput4(void)
{   
    u16 ReadValue;
	ReadValue = GPIO_ReadInputData(GPIOC);
    return ((ReadValue&0x03C0)<<2);
}

/*
*********************************************************************************************************
** 函数名称 ：KeyInput6()  
** 函数功能 ：按键扫描输入 wx
** 入口参数 ：扫描横key	 C6~C11
** 出口参数 ：无
*********************************************************************************************************
*/
u16 KeyInput6(void)
{   
    u16 ReadValue;
	ReadValue = GPIO_ReadInputData(GPIOC);
    return ((ReadValue&0x0FC0)>>2);   //3F
}


/*
*********************************************************************************************************
** 函数名称 ：KeyScan()  
** 函数功能 ：获取按键值
** 入口参数 ：无
** 出口参数 ：无
*********************************************************************************************************
*/
u16 KeyScan(void)
{
    u16 i=0,keyValue=0,keyOutput=1, keyInput = 0;
    for(i=0; i<4; i++)
    {
       KeyOutput4(keyOutput);
       keyInput = KeyInput6();	 //0x0FC0>>2	0x03F0
       if(keyInput)
       {
           keyValue = keyValue | (keyOutput | keyInput);  //0x0011~0x0308
       } 
       keyOutput=keyOutput<<1;
    }    
    return keyValue;    
}

/*
*********************************************************************************************************
** 函数名称 ：KeyValue2Physics()  
** 函数功能 ：获取按键功能值
** 入口参数 ：按键值keyValueTemp
** 出口参数 ：无
*********************************************************************************************************
*/
u16 KeyValue2Physics(u16 keyValueTemp)
{
    u16 keyValuePhysicsReturn;
    switch(keyValueTemp)
    {
		case 0x801:
            keyValuePhysicsReturn = KEY_PHYSICS1;
            break;
        case 0x401:
            keyValuePhysicsReturn = KEY_PHYSICS2;
            break;
        case 0x201:
            keyValuePhysicsReturn = KEY_PHYSICS3;
            break;
        case 0x101:
            keyValuePhysicsReturn = KEY_PHYSICS4;
            break;
        case 0x802:
            keyValuePhysicsReturn = KEY_PHYSICS5;
            break;
        case 0x402:
            keyValuePhysicsReturn = KEY_PHYSICS6;
            break;
        case 0x202:
            keyValuePhysicsReturn = KEY_PHYSICS7;
            break;
        case 0x102:
            keyValuePhysicsReturn = KEY_PHYSICS8;
            break;
        case 0x804:
            keyValuePhysicsReturn = KEY_PHYSICS9;
            break;
        case 0x404:
            keyValuePhysicsReturn = KEY_PHYSICS10;
            break;
        case 0x204:
            keyValuePhysicsReturn = KEY_PHYSICS11;
            break;
        case 0x104:
            keyValuePhysicsReturn = KEY_PHYSICS12;
            break;
        case 0x840:
            keyValuePhysicsReturn = KEY_PHYSICS13;
            break;
        case 0x440:
            keyValuePhysicsReturn = KEY_PHYSICS14;
            break;
        case 0x240:
            keyValuePhysicsReturn = KEY_PHYSICS15;
            break;
        case 0x140:
            keyValuePhysicsReturn = KEY_PHYSICS16;
            break;
        case 0x820:
            keyValuePhysicsReturn = KEY_PHYSICS17;
            break;
        case 0x420:
            keyValuePhysicsReturn = KEY_PHYSICS18;
            break;
        case 0x220:
            keyValuePhysicsReturn = KEY_PHYSICS19;
            break;
        case 0x120:
            keyValuePhysicsReturn = KEY_PHYSICS20;
            break;
        case 0x810:
            keyValuePhysicsReturn = KEY_PHYSICS21;
            break;
        case 0x410:
            keyValuePhysicsReturn = KEY_PHYSICS22;
            break;
        case 0x210:
            keyValuePhysicsReturn = KEY_PHYSICS23;
            break;
        case 0x110:
            keyValuePhysicsReturn = KEY_PHYSICS24;
            break;
        case 0x808:
            keyValuePhysicsReturn = KEY_PHYSICS25;
            break;
        case 0x408:
            keyValuePhysicsReturn = KEY_PHYSICS26;
            break;
        case 0x208:
            keyValuePhysicsReturn = KEY_PHYSICS27;
            break;
        case 0x108:
            keyValuePhysicsReturn = KEY_PHYSICS28;
            break;
        default:
            keyValuePhysicsReturn = KEY_ERROR; //包括所有其它情况，如多键等
            break; 
    }
    return keyValuePhysicsReturn;
}

//
//u16 ReadKey(void)
//{
//	  
//}


/*********************************************************************************************************
** 函数名称 ：OKKeyProcess()  
** 函数功能 ：确定按键处理函数
** 入口参数 ：按键值  wx
** 出口参数 ：无
********************************************************************************************************/
void OKKeyProcess()
{
	switch(LCD_Current_Page)
	{
		case 22:	   //开机界面
	     	 LCD_Var_Address = 0x1003;
			 LCD_New_Order=TRUE;
		     break;	
		case 40:	   //手动调试-保存X
	     	 LCD_Var_Address = 0x4042;
			 LCD_New_Order=TRUE;
		     break;	
		case 47:	   //软限位设置
	     	 LCD_Var_Address = 0x5058;
			 LCD_New_Order=TRUE;
		     break;	
		case 53:	   //修改密码
	     	 LCD_Var_Address = 0x506E;
			 LCD_New_Order=TRUE;
		     break;	
		case 62:	   //自由编程保存
	     	 LCD_Var_Address = 0x2620;
			 LCD_New_Order=TRUE;
		     break;	
		case 63:	   //自由编程清楚
	     	 LCD_Var_Address = 0x2622;
			 LCD_New_Order=TRUE;
		     break;	
		case 64:	   //自由编程速度设置
	     	 LCD_Var_Address = 0x2631;
			 LCD_New_Order=TRUE;
		     break;	
		case 65:	   //自由编程时间设置
	     	 LCD_Var_Address = 0x2635;
			 LCD_New_Order=TRUE;
		     break;	
		case 76:	   //功能设定-机械手设定
	     	 LCD_Var_Address = 0x5306;
			 LCD_New_Order=TRUE;
		     break;	
		case 78:	   //功能设定-料仓功能设定
	     	 LCD_Var_Address = 0x5348;
			 LCD_New_Order=TRUE;
		     break;	
		case 80:	   //功能设定-生产参数设定
	     	 LCD_Var_Address = 0x5358;
			 LCD_New_Order=TRUE;
		     break;	
		case 88:	   //机械手复位
	     	 LCD_Var_Address = 0x10A0;
			 LCD_New_Order=TRUE;
		     break;	
		case 95:	   //手动调试-保存Z
	     	 LCD_Var_Address = 0x4046;
			 LCD_New_Order=TRUE;
		     break;	
		case 104:	   //系统设置-加密锁设定
	     	 LCD_Var_Address = 0x5530;
			 LCD_New_Order=TRUE;
		     break;	
		case 108:	   //USB拷贝
	     	 LCD_Var_Address = 0x504A;
			 LCD_New_Order=TRUE;
		     break;	
		case 114:	   //手动调试-保存L
	     	 LCD_Var_Address = 0x404A;
			 LCD_New_Order=TRUE;
		     break;	
		case 116:	   //更换料盘
	     	 LCD_Var_Address = 0x1040;
			 LCD_New_Order=TRUE;
		     break;	
		case 120:	   //功能设定-料仓功能设定
	     	 LCD_Var_Address = 0x5348;
			 LCD_New_Order=TRUE;
		     break;	
	   default:
			 break;
	}
}


/*********************************************************************************************************
** 函数名称 ：EXITKeyProcess()  
** 函数功能 ：退出按键处理函数
** 入口参数 ：按键值  wx
** 出口参数 ：无
********************************************************************************************************/
void EXITKeyProcess()
{
	switch(LCD_Current_Page)
	{
		case 22:	   //开机界面
	     	 LCD_Var_Address = 0x1004;
			 LCD_New_Order=TRUE;
		     break;	
		case 40:	   //手动调试-保存X
	     	 LCD_Var_Address = 0x4043;
			 LCD_New_Order=TRUE;
		     break;	
		case 41:	   //点存储管理-X-返回
	     	 LCD_Var_Address = 0x502A;
			 LCD_New_Order=TRUE;
		     break;	
		case 44:	   //点存储管理-Z-返回
	     	 LCD_Var_Address = 0x502A;
			 LCD_New_Order=TRUE;
		     break;	
 		case 45:	   //程序管理-返回
	     	 LCD_Var_Address = 0x5045;
			 LCD_New_Order=TRUE;
		     break;	
		case 47:	   //软限位设置
	     	 LCD_Var_Address = 0x5059;
			 LCD_New_Order=TRUE;
		     break;	
		case 49:	   //用户设置-返回
	     	 LCD_Var_Address = 0x5067;
			 LCD_New_Order=TRUE;
		     break;	
		case 51:	   //安全区设置-返回
	     	 LCD_Var_Address = 0x5080;
			 LCD_New_Order=TRUE;
		     break;	
		case 53:	   //修改密码
	     	 LCD_Var_Address = 0x506F;
			 LCD_New_Order=TRUE;
		     break;	
		case 62:	   //自由编程保存
	     	 LCD_Var_Address = 0x2621;
			 LCD_New_Order=TRUE;
		     break;	
		case 63:	   //自由编程清除
	     	 LCD_Var_Address = 0x2623;
			 LCD_New_Order=TRUE;
		     break;	
		case 64:	   //自由编程速度设置
	     	 LCD_Var_Address = 0x2632;
			 LCD_New_Order=TRUE;
		     break;	
		case 65:	   //自由编程时间设置
	     	 LCD_Var_Address = 0x2636;
			 LCD_New_Order=TRUE;
		     break;	
		case 76:	   //功能设定-机械手设定
	     	 LCD_Var_Address = 0x5307;
			 LCD_New_Order=TRUE;
		     break;	
		case 78:	   //功能设定-料仓功能设定
	     	 LCD_Var_Address = 0x5349;
			 LCD_New_Order=TRUE;
		     break;	
		case 80:	   //功能设定-生产参数设定
	     	 LCD_Var_Address = 0x5359;
			 LCD_New_Order=TRUE;
		     break;	
		case 88:	   //机械手复位
	     	 LCD_Var_Address = 0x10A1;
			 LCD_New_Order=TRUE;
		     break;	
		case 95:	   //手动调试-保存Z
	     	 LCD_Var_Address = 0x4047;
			 LCD_New_Order=TRUE;
		     break;	
		case 98:	   //自由编程-参数设置X-返回
	     	 LCD_Var_Address = 0x400B;
			 LCD_New_Order=TRUE;
		     break;	
		case 99:	   //自由编程-参数设置Z-返回
	     	 LCD_Var_Address = 0x401B;
			 LCD_New_Order=TRUE;
		     break;	
		case 104:	   //系统设置-加密锁设定
	     	 LCD_Var_Address = 0x5531;
			 LCD_New_Order=TRUE;
		     break;	
		case 107:	   //USB拷贝-返回
	     	 LCD_Var_Address = 0x5049;
			 LCD_New_Order=TRUE;
		     break;	
		case 108:	   //USB拷贝完成-返回
	     	 LCD_Var_Address = 0x504B;
			 LCD_New_Order=TRUE;
		     break;	
		case 109:	   //脉冲微调-返回
	     	 LCD_Var_Address = 0x4054;
			 LCD_New_Order=TRUE;
		     break;	
		case 113:	   //自由编程-参数设置L-返回
	     	 LCD_Var_Address = 0x4067;
			 LCD_New_Order=TRUE;
		     break;	
		case 114:	   //手动调试-保存L
	     	 LCD_Var_Address = 0x404B;
			 LCD_New_Order=TRUE;
		     break;	
		case 115:	   //点存储L-返回
	     	 LCD_Var_Address = 0x502A;
			 LCD_New_Order=TRUE;
		     break;	
		case 116:	   //更换料盘
	     	 LCD_Var_Address = 0x1041;
			 LCD_New_Order=TRUE;
		     break;	
		case 120:	   //功能设定-料仓功能设定
	     	 LCD_Var_Address = 0x5349;
			 LCD_New_Order=TRUE;
		     break;	
	   default:
			 break;
	}
}

/*********************************************************************************************************
** 函数名称 ：KeyValueProcess()  
** 函数功能 ：获取按键功能值
** 入口参数 ：按键值  wx
** 出口参数 ：无
********************************************************************************************************/
void KeyValueProcess(u16 keyValue)
{
	switch(keyValue)
	{
		case 0x081:	   //启动
		   if(LCD_Current_Page==START_UP) //首界面
			 {
					LCD_Var_Address=0x1009;
					LCD_New_Order=TRUE;
			 }		     
		   break;
			 
		case 0x088:	   //停止
		   if(LCD_Current_Page==START_UP) //首界面
			 {
					if(Robot_Status == ACTION_RUN)	 //运行状态则暂停
					{
						LCD_Var_Address=0x100A;
						LCD_New_Order=TRUE;
					}
					else if(Robot_Status == ACTION_PAUSE)
					{
						LCD_Var_Address=0x100B;
						LCD_New_Order=TRUE;
					}
			 }
		   break;	
			 
		case 0x082:	   //X-	
			 if((LCD_Current_Page==14 || LCD_Current_Page==98)&&(g_Axis_Is_Run==FALSE)) 
			 {
					LCD_Var_Address=0x4020;
					LCD_Key_Value  =1;
					Key_Link_Move_Flag=LCD_Var_Address;
					LCD_New_Order=TRUE;
			 }
		     break;
		case 0x084:	   //X+		
			 if((LCD_Current_Page==14 || LCD_Current_Page==98)&&(g_Axis_Is_Run==FALSE)) 
			 {
					LCD_Var_Address=0x4022;
					LCD_Key_Value  =1;
					Key_Link_Move_Flag=LCD_Var_Address;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x101:	   //Y-		
			 if((LCD_Current_Page==67 || LCD_Current_Page==113)&&(g_Axis_Is_Run==FALSE))  
			 {
					LCD_Var_Address=0x4028;
					LCD_Key_Value  =1;
					Key_Link_Move_Flag=LCD_Var_Address;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x201:	   //Y+		
			 if((LCD_Current_Page==67 || LCD_Current_Page==113)&&(g_Axis_Is_Run==FALSE))  
			 {
					LCD_Var_Address=0x402A;
					LCD_Key_Value  =1;
					Key_Link_Move_Flag=LCD_Var_Address;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x102:	   //Z-		
			 if((LCD_Current_Page==91 || LCD_Current_Page==99)&&(g_Axis_Is_Run==FALSE)) 
			 {
					LCD_Var_Address=0x4024;
					LCD_Key_Value  =1;
					Key_Link_Move_Flag=LCD_Var_Address;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x202:	   //Z+		
			 if((LCD_Current_Page==91 || LCD_Current_Page==99)&&(g_Axis_Is_Run==FALSE))  
			 {
					LCD_Var_Address=0x4026;
					LCD_Key_Value  =1;
					Key_Link_Move_Flag=LCD_Var_Address;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x108:	   //A-		
			 if((LCD_Current_Page==69 || LCD_Current_Page==121)&&(g_Axis_Is_Run==FALSE)) 
			 {
					LCD_Var_Address=0x402C;
					LCD_Key_Value  =1;
					Key_Link_Move_Flag=LCD_Var_Address;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x208:	   //A+		
			 if((LCD_Current_Page==69 || LCD_Current_Page==121)&&(g_Axis_Is_Run==FALSE))  
			 {
					LCD_Var_Address=0x402E;
					LCD_Key_Value  =1;
					Key_Link_Move_Flag=LCD_Var_Address;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x104:	   //B-		

		     break;
		
		case 0x204:	   //B+		

		     break;
		
		case 0x011:	   //手/自动	
			 if(LCD_Current_Page==START_UP) //---2017.11.13开机界面
			 {
				  LCD_Var_Address=0x4000;
					LCD_New_Order=TRUE;
			 }
       else if(LCD_Current_Page==MANUL_DEBUG)
       {
				  LCD_Var_Address=0x1000;
					LCD_New_Order=TRUE;
			 }
       else{}			 
		   break;
		
		case 0x012:	   //向上		
			 if(LCD_Current_Page==FREE_PROGRAMME) //---2017.11.13自由编程界面
			 {
					LCD_Var_Address=0x2005;
					LCD_New_Order=TRUE;
			 }		
		   break;
			 
		case 0x014:	   //向下		
			 if(LCD_Current_Page==FREE_PROGRAMME) //---2017.11.13自由编程界面
			 {
					LCD_Var_Address=0x2006;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x018:	   //确定		
			 OKKeyProcess();			 
		   break;
		
		case 0x028:	   //退出-返回-取消		
			 EXITKeyProcess();
		   break;
		
		case 0x024:	   //“.”	
			 if(LCD_Current_Page==85)
			 {
					LCD_Var_Address = 0x1EE0;
					LCD_New_Order=TRUE;
			 }
			 else if(g_Robot_Error_Flag==TRUE)
			 {
					LCD_Var_Address = 0x6A20;
		   }
		   break;
			 
		case 0x022:	   //F1	开机界面	
			 if(LCD_Current_Page!=85)
			 {
					LCD_Var_Address = 0x1000;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x021:	   //F2	自由编程	
			 if(LCD_Current_Page!=85)
			 {
					LCD_Var_Address = 0x2000;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x041:	   //F3	IO调试	
			 if(LCD_Current_Page!=85)
			 {
					LCD_Var_Address = 0x3000;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x042:	   //F4	手动调试	
			 if(LCD_Current_Page!=85)
			 {
					LCD_Var_Address = 0x4000;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x044:	   //F5	系统设置	
			 if(LCD_Current_Page!=85)
			 {			 
					LCD_Var_Address = 0x5000;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		case 0x048:	   //F6	报警历史	
			 if(LCD_Current_Page!=85)
			 {			 
					LCD_Var_Address = 0x6000;
					LCD_New_Order=TRUE;
			 }
		   break;
			 
		default:
		     break;
	}
}

/*********************************************************************************************************
** 函数名称 ：RodProcess()  
** 函数功能 ：处理遥杆功能
** 入口参数 ：
** 出口参数 ：无
** 作者			：White_L
********************************************************************************************************/
u8 AXIS_Choose=0;//遥杆轴选择 X Y Z O V W   1 2 3 4 5 6
extern  u8 Run_Mode_Flag;
void RodProcess(void)
{
	if((Landing_Success==TRUE)&&(LCD_New_Order==FALSE))	 //&&(Robot_Origined == TRUE)
	{
		 if(LCD_Current_Page==0x94||LCD_Current_Page==0x95||LCD_Current_Page==0x96
			 ||LCD_Current_Page==0x97||LCD_Current_Page==0x98||LCD_Current_Page==0x99)
		 {
			Rob_Move();//手动调试实时获取遥杆数据			
		 }
		 if(LCD_Current_Page==0x04||LCD_Current_Page==0x09||LCD_Current_Page==0x0A)
		 {
			Rob_Move();//产品示教实时获取遥杆数据				 
		 }
		 if(LCD_Current_Page==8||LCD_Current_Page==2)
		 {
			 Rod_Select_Order(LCD_Current_Page);
		 }
		 if(LCD_Current_Page==0x33)
		 {
			 Rod_Adjust();
		 }
		Key_Confirm();	//遥杆按键处理
	}
}
/*********************************************************************************************************
** 函数名称 ：KeyProcess()  
** 函数功能 ：处理按键功能
** 入口参数 ：无  wx
** 出口参数 ：无
********************************************************************************************************/
void KeyProcess()
{
	if((Landing_Success==TRUE)&&(LCD_New_Order==FALSE))	 //&&(Robot_Origined == TRUE)
	{
		u16 Key_Value=0;
		if(g_Key_Scan==TRUE)
		{			
			g_Key_Value=KeyScan();
			g_Key_Scan=FALSE;			
			if((g_Key_Value!=0)&&(Key_Pressed==FALSE))	//检测到按键第一次按下
			{
				Key_Scan_Delay_Count=1;
				Key_Link_Move_Flag=0;
			}
			else if((g_Key_Value==0)&&(Key_Pressed==TRUE))   //按键按下-->松开
			{
				//手动连动按下-->松开  发送松开指令
				if(Key_Link_Move_Flag!=0)
				{
					 LCD_Var_Address=Key_Link_Move_Flag;
					 LCD_Key_Value  =0;
					 LCD_New_Order=TRUE;
					 Key_Link_Move_Flag=0;
				}
				Key_Pressed=FALSE;
				Key_Scan_Delay_Count=0;
			}
			else
			{
			  Key_Scan_Delay_Count=0;				
			}
		}//if(g_Key_Scan==TRUE)
		if(Key_Scan_Delay_Count>=2) //按键延时滤波20ms
		{
			 Key_Scan_Delay_Count=0;
			 Key_Value=KeyScan();			
		}
		if((g_Key_Value!=0)&&(g_Key_Value==Key_Value)&&(Key_Pressed==FALSE))	//按键按下进行判断
		{
		   Key_Pressed=TRUE;
		   KeyValueProcess(g_Key_Value);			
			 g_Key_Value=0;
		}
	}
}


/*********2018.6.11 White_L*******************/
void Mannul_Program_Process(void)
{
	
}
/******************* (C) COPYRIGHT 2011 YWPOLES *****END OF FILE****/

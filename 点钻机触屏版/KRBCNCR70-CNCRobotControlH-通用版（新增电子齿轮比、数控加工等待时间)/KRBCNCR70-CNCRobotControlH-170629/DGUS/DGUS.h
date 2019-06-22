/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __DGUS_H_
#define __DGUS_H_

#include "Usart.h"

#include "StartUp.h"
#include "FreeProgramme.h"
#include "IOControl.h"
#include "ManulOperate.h"
#include "SystemSetting.h"
#include "AlarmHistory.h"
#include "IOCommand.h"

#include "string.h"

#include "Parameter.h"
#include "Usart1_Robot.h"
#include "24Cxx.h"

//------------主界面页码------------//
#define START_UP       1
#define FREE_PROGRAMME 2
#define IO_CONTROL     12
#define MANUL_DEBUG    4
#define SYSTEM_SETTING 17
#define ALARM_HISTORY  16
//#define START_SYC_DATA 74

#define ORDINARY_AUTHORITY  1	//普通权限
#define MANAGE_AUTHORITY  2	//管理员权限

extern u16 LCD_Var_Address ;			   //变量地址
extern u32 LCD_Var_Data ;				   //变量数据
extern u16 LCD_Key_Value ;				   //变量键值
extern u16 LCD_GearRatio_Distance[6];   //坐标校准输入距离 mm
extern u16 LCD_RTC_Reg_len; 			   //实时时钟寄存器及数据长度
extern u32 LCD_RTC_Day;					   //年-月-日-周
extern u32 LCD_RTC_Time ;				   //时-分-秒
extern u8  LCD_New_Order ;		           //接收到新数据
extern u8  LCD_Data_Decode_Flag;		   //解码数据标志位
extern u8  LCD_Popup_Windows;			   //弹窗标识位
extern u8  LCD_Current_Page;			   //当前显示页码
extern u8  LCD_Pre_Page;
extern u8  LCD_Main_Page ;
extern u8  LCD_Page_Refresh;
extern u8  LCD_Display_ChineseString;
extern u8  LCD_Page_Cut;
extern u8  LCD_Hidden_Display;

extern u8  Dispaly_Order_Num ;		 //单次显示命令条数
extern u8  Dispaly_Order[20] ;	 //单次显示命令内容
extern u8  Order_Temp[60] ;			 //指令缓存，用于串口发送
extern u8  IIC_Temp[60];
//显示中文字符结构体
typedef struct 
{
	u16 addr ;
	u8 *data ;
	u8 len ;
}ChineseString;
extern ChineseString Display_ChineseString;

//图片剪切结构体
typedef struct 
{
	u16 Vary_addr ;
	u16 Pic_ID;
	u16 Left_X;
	u16 Up_Y;
	u16 Right_X;
	u16 Down_Y;
	u16 New_X;
	u16 New_Y;
}PageCut;
//extern PageCut  Display_PageCut;

//显示隐藏内容-描述指针
typedef struct 
{
	u16 Des_Point ;
	u16 Vary_addr ;
}UnHidden;
extern UnHidden UnHidden_Display;

//隐藏显示内容-描述指针
typedef struct 
{
	u16 Des_Point ;
}Hidden;
extern Hidden Hidden_Display;

extern u32 Pass_Word[2];
extern u8  Refresh_Admin_Flag;
extern u8  Admin_Authority;					   //0:普通权限 1:管理员权限
extern u8  Admin_Authority_Dis[2][10];
extern u8  Robot_Admin_Authority;
extern u32 g_Pass_Word_Count;
extern u32 g_Pass_Word_Delay; 
/*----------------Start_Up--界面1操作标志位----------------*/
extern u8 g_Alarm_Refresh_Flag ;	     //报警显示
extern u8 g_Welcom_Flag ;	             //显示欢迎
extern u8 g_Robot_Enable_Flag ;	         //使能
extern u8 g_Emergency_Stop_Flag ;	     //急停
extern u8 g_Cycle_Run_Flag ;	 		 //0:循环  1：单步
extern u8 g_X_Position_Flag;	 		 //X位置刷新
extern u8 g_Z_Position_Flag ;	 		 //Z位置刷新
extern u8 g_Robot_Origin_Flag ;	 		 //回零
extern u8 g_Robot_Run_Flag ;			 //启动
extern u8 g_Robot_Pause_Flag ;			 //暂停
extern u8 g_Robot_Stop_Flag ;			 //停止
extern u8 g_Program_Refresh_Flag;	     //显示程序刷新
extern u8 g_Run_Program_Num;
extern u16 g_Run_Program_Operater_List,g_Run_Program_Operater_List1;	 //当前选择运行程序正在运行的指令条：由主控板返回
extern u8 g_Run_Step;
extern u8 Auto_Run_Flag;
extern u8 Auto_Run_Count;
extern u16 g_Run_Program_List_Num;
extern u8 g_Reflash_Progress_Num ;
extern u8 g_Robot_Auto_Reset;
extern u8 g_Robot_Auto_Reset_Scan;
extern u8 Axis_Position_Scan ;
/*----------------Free_Program--界面2操作标志位--------------*/
extern u8 g_Background_Color;			  //编辑背景色
extern u8 g_Background_Color_Blink ;      //背光闪烁标志位
extern u8 g_Background_Blink_Delay ;      //背景闪烁延时标志位
extern u8 g_Background_Blink_Count;
extern u8 g_Event_Done_Flag  ; 
extern u8 g_SaveProgram_Flag ;            //程序保存
extern u32 Axis_Parameter_Speed;          //轴速度值百分比
extern u32 Delay_Parameter_Time ;				  //延时时间参数
extern u8 g_Current_Paramter_Num ;		  //参数设置行
extern u8 g_Current_Paramter_Key ;		  //参数设置指令编号
extern u8 g_Pre_SafeArea;   		      //参数设置时-保存当前选择安全区域
extern u8 g_FreeProgram_Setting_XZ;
extern u8 gs_FreeProgram_Setting_XZ;
/*----------------IO_Control--界面3操作标志位--------------*/
extern u8  g_Watch_IO_Control;
extern u16 g_LocalIO_Status ;	 //本地IO状态
extern u16 g_Input1_Status  ;	 //输入1状态
extern u16 g_Input2_Status  ;	 //输入2状态
extern u16 g_Input3_Status  ;	 //输入3状态
extern u16 g_Output1_Status ;	 //输出1状态
extern u16 g_Output2_Status ;	 //输出2状态
extern u16 g_Output3_Status ;	 //输出3状态
extern u8  g_Output1Ack_Flag;
extern u8  g_Output2Ack_Flag;
extern u8  g_Output3Ack_Flag;

/*----------------Manul_Operate--界面4操作标志位------------*/
extern u8  g_ManulOperate_Flag; 
extern u8  g_X_Origin_Flag;			     //X轴回零
extern u8  g_Z_Origin_Flag;			     //z轴回零
extern u8  g_L_Origin_Flag;			     //z轴回零
extern u8  g_X_SavePoint_Flag;	         //X保存
extern u8  g_Z_SavePoint_Flag ;	         //Z保存
extern u8  g_L_SavePoint_Flag ;	         //L保存
extern u8  g_O_SavePoint_Flag ;	         //O保存
extern u8  g_Current_Manul_Axis;		 //手动微调时当前选择轴
extern u16 g_Manule_Pulse ;
extern u16 gs_Manule_Pulse ;
/*---------------------------------------------------------*/

/*----------------SystemSetting--界面5操作标志位------------*/
extern u8  g_Buzzer_Switch_Flag;
extern u32 g_BackLight_Time;
extern u32 g_BackLight_Count;
extern u8  g_BackLight_Dark;
//extern u8  g_X_SoftLimit_flag; 
//extern u8  g_Z_SoftLimit_flag;
//extern u8  g_L_SoftLimit_flag;
extern PageCut Safe_Area_List[7];
extern u8  USBH_Process_Flag;       
extern u8  USBH_Direction;
extern u8  USBH_Dispaly_Fresh_Flag;
extern u8  USBH_And_Control[2][14];
extern u8  USBH_CopyStatus_String[12][22];
extern u8 USBH_Copy_Status;	   
extern u8 USB_Send_Program;

extern u8  USB_Update_Flag;               //---2018.5.28
extern u8  USB_Update_Status;             //---2018.5.28
extern u8  USB_Dispaly_Fresh_Flag;        //---2018.5.28
extern u8  USB_Update_Choice[4][14];      //---2018.5.28
extern u8  USB_UpdateStatus_String[8][24];//---2018.5.28
/*---------------------------------------------------------*/


//------------弹出菜单------------//
#define PW_1_PASSWORDERROR      0x10   //密码错误
#define PW_1_LOADFISRT          0x20   //请先登录
#define PW_1_HOMEFIRST          0x30   //请先回零
#define PW_1_ENABLEFIRST        0x40   //请先使能
#define PW_1_STOPROBOT          0x50   //请先停止机械手
#define PW_4_POINTSAVEFULLX     0x60   //X存储点位置已满
#define PW_4_POINTSAVEFULLZ     0x70   //Z存储点位置已满
#define PW_5_PREPASSWORDERROR   0x80   //原密码错误
#define PW_5_ENTERPASSWORD      0x90   //请输入有效六位密码
#define PW_5_NOTTHESAME         0xA0   //两次输入密码不一致
#define PW_6_HOMEFIRST1         0xB0   //报警历史-请先回零1
#define PW_6_HOMEFIRST2         0xC0   //报警历史-请先回零2
#define PW_4_HOMEFIRST          0xD0   //系统设置-请先回零
#define PW_2_PROGRAMSAVEFULL    0xE0   //存储程序位置已满
#define PW_2_PARAMETERUNSET     0xF0   //程序参数未设置完成

#define PW_5_SOFTLIMITERROR     0x15   //软限位参数错误
#define PW_5_SAFEAREAERROE      0x25   //安全区参数错误
#define PW_1_CHOOSEPROGRAM      0x35   //请先选择运行程序
#define PW_2_STARTENDERROR      0x45   //程序开始-结束格式不对
#define PW_1_NOINFISRTPAGE      0x55   //机械手失能-不在首页
#define PW_5_UNLOCKCODEEROOR    0x65   //解锁密码错误
//ZJ通用版本新增
#define PW_5_GEARRATIO          0x75   //电子齿轮比错误
#define PW_5_SAFEAREAHOME       0x85   //安全区域回零
#define PW_5_PASSWORDERROR      0xB5   //固件更新密码错误
#define PW_5_REBOOTSYSTEM		0xC5	//固件选择完成，提示重启

void DisplayChineseString(u16,u8*,u8 );
void DisplayPageCut(u16 Pic_addr,u16 Pic_ID,u16 row1,u16 column1,u16 row2,u16 column2,u16 row3,u16 column3);
void Display_PageCut(PageCut Icon);

void WriteNum2(u16 addr,u16 num);
void WriteNum4(u16 addr,u32 num);
//void WriteNum8(u16 addr,u64 num);
void WriteNum9(u16 addr);

void LCDDisplayFuction(void);	//LCD显示处功能实现函数
void LCDDisplay(void);			//LCD显示处理
void LCDProcess(void);			//触摸处理

void BuzzerRing(u8);
void BackLight(u8);
void ReadRTC(void);
void PageChange(u8);
void Popup_Windows(u8);

void HiddenDisplay(u16);
void UnHiddenDisplay(u16,u16);






#endif


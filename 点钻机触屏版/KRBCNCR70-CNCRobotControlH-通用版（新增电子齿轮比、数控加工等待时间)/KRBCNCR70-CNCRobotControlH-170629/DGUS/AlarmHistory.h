/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : ***.h
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 05/28/2015
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x.h"

#ifndef __ALARMHISTORY_H_
#define __ALARMHISTORY_H_


typedef struct{
	u8  Flag;  //是否有错误
	u16 year;   //日期
	u8  month;  //时间
	u8  day;
	u8  week;
	u8  hour;
	u8  minute;
	u8  second;
	u8  Type;  //错误类型
}AlarmHistory;
//extern AlarmHistory Alarm_History_Display;

typedef struct{
	u8  Flag; 
	u8  Day[10];
 	u8  Time[8];
	u8  Value;
	u8  Type;  //错误类型
}AlarmHistoryDisplay;

//extern u8  Alarm_Code;			 //报警编号
extern u8  Alarm_String[40][30]; //显示报警文档
extern u8  g_Current_Alarm;		 //当前报警类型-编号
extern u8 g_Alarm_Buzzer;
extern u8  g_Alarm_Buzzer_Timer;
extern u8 g_Alarm_Buzzer_Count;
extern AlarmHistoryDisplay Alarm_History_Display[21];
extern AlarmHistory Alarm_History_Time[20];
void LCDAlarmHistoryProcess(void);
void LCDAlarmHistoryDisply(void);
void LCDError(void);				//串口1错误情况














#endif


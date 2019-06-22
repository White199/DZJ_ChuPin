/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : AutoOperate.h
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 28/10/2015
* Description        : 各个动作操作函数头文件
***************************************************************************************/
#ifndef _AutoOperate_h_
#define _AutoOperate_h_


extern u8  g_Key_Delay_Flag ;
extern u32 g_Key_Delay_Timer;
extern u8  g_LC_Key_Delay_Flag ;
extern u32 g_LC_Key_Delay_Timer;
u8 Key_Delay(u32);
u8 LC_Key_Delay(u32);
void Action_Delay(u32);

u8 AutoActionOutControl(u16 ActionLine);
u8 AutoActionOutConfirm(SaveProgram Free_Program_Operate1,u16 ShuZu_Count);
u8 AutoActionOutDelay(SaveProgram Program_Operate,u8 ActionLine);
u8 ActionAllowJudge(SaveProgram Program_Operate,u8 ActionLine);
u8 AXisMove(u8 ,u32 ,u32 );

u8 LCActionOutDelay(SaveProgram Program_Operate,u8 ActionLine);

extern u8  g_ActionDelay_Step;
extern u32 g_ActionDelay_Timer;
extern u8  g_LCDelay_Step;
extern u32 g_LCDelay_Timer;

extern u8 g_Program_Start_Action;
extern u8 g_Program_Start_Action_Done;
extern u8 g_Program_Reset_Action;
extern u8 g_Program_Reset_Action_Done;

#endif

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/


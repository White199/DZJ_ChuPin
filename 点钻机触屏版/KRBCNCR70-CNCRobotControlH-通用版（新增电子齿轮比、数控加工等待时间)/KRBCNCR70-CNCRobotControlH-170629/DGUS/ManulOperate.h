/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : ***.h
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 05/28/2015
* Description        : This is the ....
***************************************************************************************/

#ifndef __MANULOPERATE_H_
#define __MANULOPERATE_H_



extern u8 g_Manul_Move_count;	 //X轴左右移连动计数
extern u8 g_Manul_Move_Flag;
extern u8 g_Manul_Link_Flag;
extern u8 Temp_Display_Data[4];
//extern u8 g_Servo_Is_Run ;	 //是否在发脉冲
void LCDManulOperateProcess(void);
void LCDManulOperateDisply(void);
void ManulOutputSet(void);
void Key_Confirm(void);
u8 Program_Save_Time(void);
void Program_Save(void);
void Insert_Order(u8 Order_type);
u8 Order_Check( u8 Order_Type );













#endif


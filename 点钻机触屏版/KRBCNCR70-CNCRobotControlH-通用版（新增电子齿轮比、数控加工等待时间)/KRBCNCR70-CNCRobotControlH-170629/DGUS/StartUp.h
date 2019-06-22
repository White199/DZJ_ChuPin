/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : ***.h
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 05/28/2015
* Description        : This is the ....
***************************************************************************************/

#ifndef __STARTUP_H_
#define __STARTUP_H_

#define DIS_STARTUP_PAGE    0x00	 //显示当前页
#define DIS_AlARM_DISPLAY   0x01	 //报警显示
#define DIS_ROBOT_ENABLE    0x02	 //使能
#define DIS_ROBOT_DISABLE   0x03	 //失能
#define DIS_EMERGENCY_STOP  0x04	 //急停
#define DIS_NOEMERGENCY     0x05	 //未急停
#define DIS_CYCLE_RUN       0x06	 //循环
#define DIS_SINGLE_RUN      0x07	 //单步
#define DIS_X_POSITION      0x08	 //X位置
#define DIS_Z_POSITION      0x09	 //Z位置
#define DIS_ORIGIN_Y        0x0A	 //回零效果
#define DIS_ORIGIN_N        0x0B	 //回零无效果
#define DIS_RUN_Y           0x0C	 //启动效果
#define DIS_RUN_N  		    0x0D	 //启动无效果
#define DIS_PAUSE_Y         0x10	 //暂停效果
#define DIS_PAUSE_N 	    0x11	 //暂停无效果
#define DIS_STOP_Y          0x12	 //停止效果
#define DIS_STOP_N 	        0x13	 //停止无效果
#define DIS_PROGRAM	        0x14	 //显示程序



#define AUTO_WAITE				0
#define AUTO_RUNNING			1
#define AUTO_PAUSE				2
#define AUTO_ERROR				3

extern u8 g_Auto_Status;
extern u8 Landing_Success ; 
extern u8  Change_User;
extern u8 gs_Run_Program_Num;


extern u8 Send_String_DL[];

void LCDStartUpDisply(void);
void LCDStartUpProcess(void);
void LandingProcess(void);
void BackToOrigin(void);
















#endif


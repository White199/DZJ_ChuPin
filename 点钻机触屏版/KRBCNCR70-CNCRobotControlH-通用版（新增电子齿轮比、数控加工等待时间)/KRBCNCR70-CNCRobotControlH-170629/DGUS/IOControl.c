/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ****************
* File Name          : 
* Author             : 
* Version            : 
* Date               : 
* Description        : 
******************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "DGUS.h"


static u16 gs_LocalIO_Status = 0;	 //本地IO状态  gs_LocalIO_Status[0]=[0~4]表示第一行五个点，第二行同理
static u16 gs_Input1_Status = 0;	 //输入1状态   gs_Input1_Status[0]=[0~7];gs_Input1_Status[1]=[0~6]--8+7=15
static u16 gs_Input2_Status = 0;	 //输入2状态
static u16 gs_Input3_Status = 0;	 //输入3状态
static u16 gs_Output1_Status = 0;	 //输出1状态
static u16 gs_Output2_Status = 0;	 //输出2状态
static u16 gs_Output3_Status = 0;	 //输出3状态

/*------本地IO--------*/
//启动图标
static PageCut L_PageCut_Local_1Y  = {0x3100,5,30 ,105,115,172,30 ,105};	 
static PageCut L_PageCut_Local_1N  = {0x3100,4,30 ,105,115,172,30 ,105};
//停止图标
static PageCut L_PageCut_Local_2Y  = {0x3110,5,160,105,245,172,160,105};	 
static PageCut L_PageCut_Local_2N  = {0x3110,4,160,105,245,172,160,105};
//复位图标
static PageCut L_PageCut_Local_3Y  = {0x3120,5,290,105,375,172,290,105};	 
static PageCut L_PageCut_Local_3N  = {0x3120,4,290,105,375,172,290,105};
//急停图标
static PageCut L_PageCut_Local_4Y  = {0x3130,5,420,105,505,172,420,105};	 
static PageCut L_PageCut_Local_4N  = {0x3130,4,420,105,505,172,420,105};
//润滑图标
static PageCut L_PageCut_Local_5Y  = {0x3140,5,550,105,635,172,550,105};	 
static PageCut L_PageCut_Local_5N  = {0x3140,4,550,105,635,172,550,105};
//气源图标
static PageCut L_PageCut_Local_6Y  = {0x3150,5,680,105,765,172,680,105};	 
static PageCut L_PageCut_Local_6N  = {0x3150,4,680,105,765,172,680,105};
//X左极限图标
static PageCut L_PageCut_Local_7Y  = {0x3160,5,30 ,215,115,280,30 ,215};	 
static PageCut L_PageCut_Local_7N  = {0x3160,4,30 ,215,115,280,30 ,215};
//X右极限图标
static PageCut L_PageCut_Local_8Y  = {0x3170,5,160,215,245,280,160,215};	 
static PageCut L_PageCut_Local_8N  = {0x3170,4,160,215,245,280,160,215};
//Z左极限图标
static PageCut L_PageCut_Local_9Y  = {0x3180,5,290,215,375,280,290,215};	 
static PageCut L_PageCut_Local_9N  = {0x3180,4,290,215,375,280,290,215};
//Z右极限图标
static PageCut L_PageCut_Local_10Y = {0x3190,5,420,215,505,280,420,215};	 
static PageCut L_PageCut_Local_10N = {0x3190,4,420,215,505,280,420,215};	
//Z左极限图标
static PageCut L_PageCut_Local_11Y = {0x31A0,5,550,215,635,280,550,215};	 
static PageCut L_PageCut_Local_11N = {0x31A0,4,550,215,635,280,550,215};
//Z右极限图标
static PageCut L_PageCut_Local_12Y = {0x31B0,5,680,215,765,280,680,215};	 
static PageCut L_PageCut_Local_12N = {0x31B0,4,680,215,765,280,680,215};
/*------输入点1--------*/
//上料准备
static PageCut I_PageCut_Input1_1Y    = {0x3200,7,90 ,105,175,172,90 ,105};	 
static PageCut I_PageCut_Input1_1N    = {0x3200,6,90 ,105,175,172,90 ,105};
//上料允许
static PageCut I_PageCut_Input1_2Y    = {0x3210,7,240,105,325,172,240,105};	 
static PageCut I_PageCut_Input1_2N    = {0x3210,6,240,105,325,172,240,105};
//卸料准备
static PageCut I_PageCut_Input1_3Y    = {0x3220,7,385,105,470,172,385,105};	 
static PageCut I_PageCut_Input1_3N    = {0x3220,6,385,105,470,172,385,105};
//卸料允许
static PageCut I_PageCut_Input1_4Y    = {0x3230,7,535,105,620,172,535,105};	 
static PageCut I_PageCut_Input1_4N    = {0x3230,6,535,105,620,172,535,105};
//料仓-预留1
static PageCut I_PageCut_Input1_5Y    = {0x3240,7,680,105,765,172,680,105};	 
static PageCut I_PageCut_Input1_5N    = {0x3240,6,680,105,765,172,680,105};
//机床1-卡盘A开
static PageCut I_PageCut_Input1_6Y    = {0x3250,7,90 ,220,175,284,90 ,220};	 
static PageCut I_PageCut_Input1_6N    = {0x3250,6,90 ,220,175,284,90 ,220};
//机床1-卡盘A关
static PageCut I_PageCut_Input1_7Y    = {0x3260,7,240,220,325,284,240,220};	 
static PageCut I_PageCut_Input1_7N    = {0x3260,6,240,220,325,284,240,220};
//机床1-处于原点 X8
static PageCut I_PageCut_Input1_8Y    = {0x3270,7,370,220,480,305,370,220};	 
static PageCut I_PageCut_Input1_8N    = {0x3270,6,370,220,480,305,370,220};
//机床1-主轴停止 X9
static PageCut I_PageCut_Input1_9Y    = {0x3280,7,520,220,625,305,520,220};	 
static PageCut I_PageCut_Input1_9N    = {0x3280,6,520,220,625,305,520,220};
//机床1-加工完成
static PageCut I_PageCut_Input1_10Y   = {0x3290,7,680,220,765,284,680,220};	 
static PageCut I_PageCut_Input1_10N   = {0x3290,6,680,220,765,284,680,220};
//机床1-顶门开
static PageCut I_PageCut_Input1_11Y   = {0x32A0,7,90 ,328,175,390,90 ,328};	 
static PageCut I_PageCut_Input1_11N   = {0x32A0,6,90 ,328,175,390,90 ,328};
//机床1-顶门关
static PageCut I_PageCut_Input1_12Y   = {0x32B0,7,240,328,325,390,240,328};	 
static PageCut I_PageCut_Input1_12N   = {0x32B0,6,240,328,352,390,240,328};
//机床1-报警  x13
static PageCut I_PageCut_Input1_13Y   = {0x32C0,7,370,330,480,415,370,330};	 
static PageCut I_PageCut_Input1_13N   = {0x32C0,6,370,330,480,415,370,330};
//机床1-预留1
static PageCut I_PageCut_Input1_14Y   = {0x32D0,7,535,328,620,390,535,328};	 
static PageCut I_PageCut_Input1_14N   = {0x32D0,6,535,328,620,390,535,328};
//机床1-预留2
static PageCut I_PageCut_Input1_15Y   = {0x32E0,7,680,328,765,390,680,328};	 
static PageCut I_PageCut_Input1_15N   = {0x32E0,6,680,328,765,390,680,328};

/*------输入点2--------*/
//机床2-卡盘A开
static PageCut I_PageCut_Input2_1Y    = {0x3300,9,90 ,105,175,172,90 ,105};	 
static PageCut I_PageCut_Input2_1N    = {0x3300,8,90 ,105,175,172,90 ,105};
//机床2-卡盘A关
static PageCut I_PageCut_Input2_2Y    = {0x3310,9,240,105,325,172,240,105};	 
static PageCut I_PageCut_Input2_2N    = {0x3310,8,240,105,325,172,240,105};
//机床2-处于原点
static PageCut I_PageCut_Input2_3Y    = {0x3320,9,385,105,470,172,385,105};	 
static PageCut I_PageCut_Input2_3N    = {0x3320,8,385,105,470,172,385,105};
//机床2-主轴停止
static PageCut I_PageCut_Input2_4Y    = {0x3330,9,535,105,620,172,535,105};	 
static PageCut I_PageCut_Input2_4N    = {0x3330,8,535,105,620,172,535,105};
//机床2-加工完成
static PageCut I_PageCut_Input2_5Y    = {0x3340,9,680,105,765,172,680,105};	 
static PageCut I_PageCut_Input2_5N    = {0x3340,8,680,105,765,172,680,105};
//机床2-顶门开
static PageCut I_PageCut_Input2_6Y    = {0x3350,9,90 ,220,175,284,90 ,220};	 
static PageCut I_PageCut_Input2_6N    = {0x3350,8,90 ,220,175,284,90 ,220};
//机床2-顶门关
static PageCut I_PageCut_Input2_7Y    = {0x3360,9,240,220,325,284,240,220};	 
static PageCut I_PageCut_Input2_7N    = {0x3360,8,240,220,325,284,240,220};
//机床2-报警
static PageCut I_PageCut_Input2_8Y    = {0x3370,9,385,220,470,284,385,220};	 
static PageCut I_PageCut_Input2_8N    = {0x3370,8,385,220,470,284,385,220};
//机床2-预留1
static PageCut I_PageCut_Input2_9Y    = {0x3380,9,535,220,620,284,535,220};	 
static PageCut I_PageCut_Input2_9N    = {0x3380,8,535,220,620,284,535,220};
//机床2-预留2
static PageCut I_PageCut_Input2_10Y   = {0x3390,9,680,220,765,284,680,220};	 
static PageCut I_PageCut_Input2_10N   = {0x3390,8,680,220,765,284,680,220};
//翻转台-正摆
static PageCut I_PageCut_Input2_11Y   = {0x33A0,9,90 ,328,175,390,90 ,328};	 
static PageCut I_PageCut_Input2_11N   = {0x33A0,8,90 ,328,175,390,90 ,328};
//翻转台-反摆
static PageCut I_PageCut_Input2_12Y   = {0x33B0,9,240,328,325,390,240,328};	 
static PageCut I_PageCut_Input2_12N   = {0x33B0,8,240,328,325,390,240,328};
//翻转台-松开
static PageCut I_PageCut_Input2_13Y   = {0x33C0,9,385,328,470,390,385,328};	 
static PageCut I_PageCut_Input2_13N   = {0x33C0,8,385,328,470,390,385,328};
//翻转台-夹紧						  
static PageCut I_PageCut_Input2_14Y   = {0x33D0,9,535,328,620,390,535,328};	 
static PageCut I_PageCut_Input2_14N   = {0x33D0,8,535,328,620,390,535,328};
//翻转台-吹气
static PageCut I_PageCut_Input2_15Y   = {0x33E0,9,680,328,765,390,680,328};	 
static PageCut I_PageCut_Input2_15N   = {0x33E0,8,680,328,765,390,680,328};

/*------输入点3--------*/
//料爪-上料爪1松
static PageCut I_PageCut_Input3_1Y    = {0x3400,30,90 ,105,175,172,90 ,105};	 
static PageCut I_PageCut_Input3_1N    = {0x3400,29,90 ,105,175,172,90 ,105};
//料爪-上料爪1紧
static PageCut I_PageCut_Input3_2Y    = {0x3410,30,240,105,325,172,240,105};	 
static PageCut I_PageCut_Input3_2N    = {0x3410,29,240,105,325,172,240,105};
//料爪-上料爪2松
static PageCut I_PageCut_Input3_3Y    = {0x3420,30,385,105,470,172,385,105};	 
static PageCut I_PageCut_Input3_3N    = {0x3420,29,385,105,470,172,385,105};
//料爪-上料爪2紧
static PageCut I_PageCut_Input3_4Y    = {0x3430,30,535,105,620,172,535,105};	 
static PageCut I_PageCut_Input3_4N    = {0x3430,29,535,105,620,172,535,105};
//料爪-卸料爪1松
static PageCut I_PageCut_Input3_5Y    = {0x3440,30,680,105,765,172,680,105};	 
static PageCut I_PageCut_Input3_5N    = {0x3440,29,680,105,765,172,680,105};
//料爪-卸料爪1紧
static PageCut I_PageCut_Input3_6Y    = {0x3450,30,90 ,220,175,284,90 ,220};	 
static PageCut I_PageCut_Input3_6N    = {0x3450,29,90 ,220,175,284,90 ,220};
//料爪-卸料爪2松
static PageCut I_PageCut_Input3_7Y    = {0x3460,30,240,220,325,284,240,220};	 
static PageCut I_PageCut_Input3_7N    = {0x3460,29,240,220,325,284,240,220};
//料爪-卸料爪2紧
static PageCut I_PageCut_Input3_8Y    = {0x3470,30,385,220,470,284,385,220};	 
static PageCut I_PageCut_Input3_8N    = {0x3470,29,385,220,470,284,385,220};
//料爪-摆缸正摆
static PageCut I_PageCut_Input3_9Y    = {0x3480,30,535,220,620,284,535,220};	 
static PageCut I_PageCut_Input3_9N    = {0x3480,29,535,220,620,284,535,220};
//料爪-摆缸复位
static PageCut I_PageCut_Input3_10Y   = {0x3490,30,680,220,765,284,680,220};	 
static PageCut I_PageCut_Input3_10N   = {0x3490,29,680,220,765,284,680,220};
//Y向气缸伸出
static PageCut I_PageCut_Input3_11Y   = {0x34A0,30,90 ,328,175,390,90 ,328};	 
static PageCut I_PageCut_Input3_11N   = {0x34A0,29,90 ,328,175,390,90 ,328};
//Y向气缸缩回
static PageCut I_PageCut_Input3_12Y   = {0x34B0,30,240,328,325,390,240,328};	 
static PageCut I_PageCut_Input3_12N   = {0x34B0,29,240,328,325,390,240,328};
//料爪-预留1
static PageCut I_PageCut_Input3_13Y   = {0x34C0,30,385,328,470,390,385,328};	 
static PageCut I_PageCut_Input3_13N   = {0x34C0,29,385,328,470,390,385,328};
//料爪-预留2
static PageCut I_PageCut_Input3_14Y   = {0x34D0,30,535,328,620,390,535,328};	 
static PageCut I_PageCut_Input3_14N   = {0x34D0,29,535,328,620,390,535,328};
//料爪-预留3
static PageCut I_PageCut_Input3_15Y   = {0x34E0,30,680,328,765,390,680,328};	 
static PageCut I_PageCut_Input3_15N   = {0x34E0,29,680,328,765,390,680,328};
 
/*------输出点1--------*/
//1  料仓-上料完成  
static PageCut O_PageCut_LC_LoadFinish_Y   = {0x3500,11,70 ,114,195,194,70 ,114};	 
static PageCut O_PageCut_LC_LoadFinish_N   = {0x3500,10,70 ,114,195,194,70 ,114};
//2  料仓-卸料完成  
static PageCut O_PageCut_LC_UnloadFinish_Y = {0x3510,11,218,114,342,194,218,114};	 
static PageCut O_PageCut_LC_UnloadFinish_N = {0x3510,10,218,114,342,194,218,114};
//3  料仓-上料仓报警  
static PageCut O_PageCut_LC_LoadAlarm_Y    = {0x3520,11,365,114,490,194,365,114};	 
static PageCut O_PageCut_LC_LoadAlarm_N    = {0x3520,10,365,114,490,194,365,114};
//4  料仓-卸料仓报警  
static PageCut O_PageCut_LC_UnloadAlarm_Y  = {0x3530,11,515,114,640,194,515,114};	 
static PageCut O_PageCut_LC_UnloadAlarm_N  = {0x3530,10,515,114,640,194,515,114};
//5  料仓-预留1  
static PageCut O_PageCut_LC_Reserve1_Y     = {0x3540,11,660,114,785,194,660,114};	 
static PageCut O_PageCut_LC_Reserve1_N     = {0x3540,10,660,114,785,194,660,114};
//6  机床1-卡盘A开  
static PageCut O_PageCut_M1_KaPanAOpen_Y   = {0x3550,11,70 ,220,195,305,70 ,220};	 
static PageCut O_PageCut_M1_KaPanAOpen_N   = {0x3550,10,70 ,220,195,305,70 ,220};
//7  机床1-卡盘A关  
static PageCut O_PageCut_M1_KaPanAClose_Y  = {0x3560,11,220,220,345,305,220,220};	 
static PageCut O_PageCut_M1_KaPanAClose_N  = {0x3560,10,220,220,345,305,220,220};
//8  机床1-吹气  	  Y8
static PageCut O_PageCut_M1_Blow_Y         = {0x3570,11,350,220,500,305,350,220};	 
static PageCut O_PageCut_M1_Blow_N         = {0x3570,10,350,220,500,305,350,220};
//9  机床1-允许启动  Y9
static PageCut O_PageCut_M1_AlawStart_Y    = {0x3580,11,500,220,645,305,500,220};	 
static PageCut O_PageCut_M1_AlawStart_N    = {0x3580,10,500,220,645,305,500,220};
//10  机床1-开始加工  
static PageCut O_PageCut_M1_StartProcess_Y = {0x3590,11,660,220,785,305,660,220};	 
static PageCut O_PageCut_M1_StartProcess_N = {0x3590,10,660,220,785,305,660,220};
//11  机床1-顶门开
static PageCut O_PageCut_M1_DoorOpen_Y     = {0x35A0,11,70 ,328,195,415,70 ,328};	 
static PageCut O_PageCut_M1_DoorOpen_N     = {0x35A0,10,70 ,328,195,415,70 ,328};
//12  机床1-顶门关	  Y12
static PageCut O_PageCut_M1_DoorClose_Y    = {0x35B0,11,210,328,350,415,210,328};	 
static PageCut O_PageCut_M1_DoorClose_N    = {0x35B0,10,210,328,350,415,210,328};
//13  机床1-预留1
static PageCut O_PageCut_M1_Reserve1_Y     = {0x35C0,11,365,328,490,415,365,328};	 
static PageCut O_PageCut_M1_Reserve1_N     = {0x35C0,10,365,328,490,415,365,328};
//14  机床1-预留2
static PageCut O_PageCut_M1_Reserve2_Y     = {0x35D0,11,515,328,640,415,515,328};	 
static PageCut O_PageCut_M1_Reserve2_N     = {0x35D0,10,515,328,640,415,515,328};
//15  机床1-预留3
static PageCut O_PageCut_M1_Reserve3_Y     = {0x35E0,11,660,328,785,415,660,328};	 
static PageCut O_PageCut_M1_Reserve3_N     = {0x35E0,10,660,328,785,415,660,328};

/*------输出点2--------*/
//机床2-卡盘A开
static PageCut O_PageCut_M2_KaPanAOpen_Y    = {0x3600,13,90 ,105,175,172,90 ,105};	 
static PageCut O_PageCut_M2_KaPanAOpen_N    = {0x3600,12,90 ,105,175,172,90 ,105};
//机床2-卡盘A关
static PageCut O_PageCut_M2_KaPanAClose_Y   = {0x3610,13,240,105,325,172,240,105};	 
static PageCut O_PageCut_M2_KaPanAClose_N   = {0x3610,12,240,105,325,172,240,105};
//机床2-吹气
static PageCut O_PageCut_M2_Blow_Y          = {0x3620,13,385,105,470,172,385,105};	 
static PageCut O_PageCut_M2_Blow_N          = {0x3620,12,385,105,470,172,385,105};
//机床2-允许启动
static PageCut O_PageCut_M2_AlawStart_Y     = {0x3630,13,535,105,620,172,535,105};	 
static PageCut O_PageCut_M2_AlawStart_N     = {0x3630,12,535,105,620,172,535,105};
//机床2-开始加工
static PageCut O_PageCut_M2_StartProcess_Y  = {0x3640,13,680,105,765,172,680,105};	 
static PageCut O_PageCut_M2_StartProcess_N  = {0x3640,12,680,105,765,172,680,105};
//机床2-顶门开
static PageCut O_PageCut_M2_DoorOpen_Y      = {0x3650,13,90 ,220,175,284,90 ,220};	 
static PageCut O_PageCut_M2_DoorOpen_N      = {0x3650,12,90 ,220,175,284,90 ,220};
//机床2-顶门关
static PageCut O_PageCut_M2_DoorClose_Y     = {0x3660,13,240,220,325,284,240,220};	 
static PageCut O_PageCut_M2_DoorClose_N     = {0x3660,12,240,220,325,284,240,220};
//机床2-预留1
static PageCut O_PageCut_M2_Reserve1_Y      = {0x3670,13,385,220,470,284,385,220};	 
static PageCut O_PageCut_M2_Reserve1_N      = {0x3670,12,385,220,470,284,385,220};
//机床2-预留2
static PageCut O_PageCut_M2_Reserve2_Y      = {0x3680,13,535,220,620,284,535,220};	 
static PageCut O_PageCut_M2_Reserve2_N      = {0x3680,12,535,220,620,284,535,220};
//机床2-预留3
static PageCut O_PageCut_M2_Reserve3_Y      = {0x3690,13,680,220,765,284,680,220};	 
static PageCut O_PageCut_M2_Reserve3_N      = {0x3690,12,680,220,765,284,680,220};	//
//翻转台-正摆  
static PageCut O_PageCut_FZT_Front_Y        = {0x36A0,13,90 ,328,175,390,90 ,328};	 
static PageCut O_PageCut_FZT_Front_N        = {0x36A0,12,90 ,328,175,390,90 ,328};
//翻转台-反摆  
static PageCut O_PageCut_FZT_Back_Y         = {0x36B0,13,240,328,325,390,240,328};	  
static PageCut O_PageCut_FZT_Back_N         = {0x36B0,12,240,328,325,390,240,328};
//翻转台-松开  
static PageCut O_PageCut_FZT_Release_Y      = {0x36C0,13,385,328,470,390,385,328};	 
static PageCut O_PageCut_FZT_Release_N      = {0x36C0,12,385,328,470,390,385,328};
//翻转台-夹紧  
static PageCut O_PageCut_FZT_Tight_Y        = {0x36D0,13,535,328,620,390,535,328};	 
static PageCut O_PageCut_FZT_Tight_N        = {0x36D0,12,535,328,620,390,535,328};
//翻转台-预留1  
static PageCut O_PageCut_FZT_Reserve1_Y     = {0x36E0,13,680,328,765,390,680,328};	 
static PageCut O_PageCut_FZT_Reserve1_N     = {0x36E0,12,680,328,765,390,680,328};

/*------输出点3--------*/
//料爪-上料爪松
static PageCut O_PageCut_CL_LoadOpen_Y       = {0x3700,32,90 ,105,175,172,90 ,105};	 
static PageCut O_PageCut_CL_LoadOpen_N       = {0x3700,31,90 ,105,175,172,90 ,105};
//料爪-上料爪紧
static PageCut O_PageCut_CL_LoadClose_Y      = {0x3710,32,240,105,325,172,240,105};	 
static PageCut O_PageCut_CL_LoadClose_N      = {0x3710,31,240,105,325,172,240,105};
//料爪-卸料爪松
static PageCut O_PageCut_CL_UnloadOpen_Y     = {0x3720,32,385,105,470,172,385,105};	 
static PageCut O_PageCut_CL_UnloadOpen_N     = {0x3720,31,385,105,470,172,385,105};
//料爪-卸料爪紧
static PageCut O_PageCut_CL_UnloadClose_Y    = {0x3730,32,535,105,620,172,535,105};	 
static PageCut O_PageCut_CL_UnloadClose_N    = {0x3730,31,535,105,620,172,535,105};
//料爪-摆缸正摆
static PageCut O_PageCut_CL_BG_Front_Y       = {0x3740,32,680,105,765,172,680,105};	 	 
static PageCut O_PageCut_CL_BG_Front_N       = {0x3740,31,680,105,765,172,680,105};
//料爪-摆缸复位
static PageCut O_PageCut_CL_BG_Reset_Y       = {0x3750,32,90 ,220,175,284,90 ,220};	 
static PageCut O_PageCut_CL_BG_Reset_N       = {0x3750,31,90 ,220,175,284,90 ,220};
//料爪-Y向气缸伸出
static PageCut O_PageCut_CL_YQG_Extend_Y     = {0x3760,32,240,220,325,284,240,220};	 
static PageCut O_PageCut_CL_YQG_Extend_N     = {0x3760,31,240,220,325,284,240,220};
//料爪-Y向气缸缩回
static PageCut O_PageCut_CL_YQG_Retraction_Y = {0x3770,32,385,220,470,284,385,220};	 
static PageCut O_PageCut_CL_YQG_Retraction_N = {0x3770,31,385,220,470,284,385,220};
//料爪-手抓吹气
static PageCut O_PageCut_CL_Blow_Y           = {0x3780,32,535,220,620,284,535,220};	 
static PageCut O_PageCut_CL_Blow_N           = {0x3780,31,535,220,620,284,535,220};
//料爪-预留1
static PageCut O_PageCut_CL_Reserve1_Y       = {0x3790,32,680,220,765,284,680,220};	 
static PageCut O_PageCut_CL_Reserve1_N       = {0x3790,31,680,220,765,284,680,220};
//其他-润滑使能
static PageCut O_PageCut_QT_OilingEnable_Y   = {0x37A0,32,90 ,328,175,390,90 ,328};	  //
static PageCut O_PageCut_QT_OilingEnable_N   = {0x37A0,31,90 ,328,175,390,90 ,328};
//其他-运行灯
static PageCut O_PageCut_QT_RunLight_Y       = {0x37B0,32,240,328,325,390,240,328};	 
static PageCut O_PageCut_QT_RunLight_N       = {0x37B0,31,240,328,325,390,240,328};
//其他-待机灯
static PageCut O_PageCut_QT_WaitLight_Y      = {0x37C0,32,385,328,470,390,385,328};	 
static PageCut O_PageCut_QT_WaitLight_N      = {0x37C0,31,385,328,470,390,385,328};
//其他-报警灯
static PageCut O_PageCut_QT_AlarmLight_Y     = {0x37D0,32,535,328,620,390,535,328};	 
static PageCut O_PageCut_QT_AlarmLight_N     = {0x37D0,31,535,328,620,390,535,328};
//其他-预留1
static PageCut O_PageCut_QT_Reserve1_Y       = {0x37E0,32,680,328,765,390,680,328};	 
static PageCut O_PageCut_QT_Reserve1_N       = {0x37E0,31,680,328,765,390,680,328};




static PageCut ChuJiao1_Y1 = {0x3750,7,98,181,180,245,98,181};
static PageCut ChuJiao1_N1 = {0x3750,12,98,181,180,245,98,181};

static PageCut ChuJiao2_Y1 = {0x3760,7,188,181,270,245,188,181};
static PageCut ChuJiao2_N1 = {0x3760,12,188,181,270,245,188,181};

static PageCut ChuJiao3_Y1 = {0x3770,7,278,181,360,245,278,181};
static PageCut ChuJiao3_N1 = {0x3770,12,278,181,360,245,278,181};

static PageCut JiaoTouXiaYi1_Y1 = {0x3780,7,368,184,455,245,368,181};
static PageCut JiaoTouXiaYi1_N1 = {0x3780,12,368,184,455,245,368,181};

static PageCut JiaoTouXiaYi2_Y1 = {0x3790,7,458,184,545,245,458,181};
static PageCut JiaoTouXiaYi2_N1 = {0x3790,12,458,184,545,245,458,181};

static PageCut JiaoTouXiaYi3_Y1 = {0x37A0,7,548,184,620,245,548,181};
static PageCut JiaoTouXiaYi3_N1 = {0x37A0,12,548,184,620,245,548,181};


static PageCut XiZuan1_Y1 = {0x37B0,7,630,181,705,245,630,181};
static PageCut XiZuan1_N1 = {0x37B0,12,630,181,705,245,630,181};

static PageCut XiZuan2_Y1 = {0x37C0,7,705,181,780,245,705,181};
static PageCut XiZuan2_N1 = {0x37C0,12,705,181,780,245,705,181};

static PageCut XiZuan3_Y1 = {0x37D0,7,101,266,181,327,101,266};
static PageCut XiZuan3_N1 = {0x37D0,6,101,266,181,327,101,266};

static PageCut XiZuan4_Y1 = {0x37E0,7,181,266,281,327,181,266};
static PageCut XiZuan4_N1 = {0x37E0,12,181,266,281,327,181,266};

static PageCut XiZuan5_Y1 = {0x37F0,7,281,266,371,327,279,262};
static PageCut XiZuan5_N1 = {0x37F0,12,281,266,371,327,279,262};

static PageCut XiZuan6_Y1 = {0x3800,7,371,266,458,327,364,266};
static PageCut XiZuan6_N1 = {0x3800,12,371,266,458,327,364,266};

static PageCut XiZuan7_Y1 = {0x3810,7,456,266,533,324,453,266};
static PageCut XiZuan7_N1 = {0x3810,12,456,266,533,324,453,266};

static PageCut XiZuan8_Y1 = {0x3820,7,541,266,621,327,545,270};
static PageCut XiZuan8_N1 = {0x3820,12,541,266,621,327,545,270};

static PageCut ZuanTouXiaYi1_Y1 = {0x3830,7,621,266,701,327,628,271};
static PageCut ZuanTouXiaYi1_N1 = {0x3830,12,621,266,701,327,628,271};

static PageCut ZuanTouXiaYi2_Y1 = {0x3840,7,701,268,780,327,701,268};
static PageCut ZuanTouXiaYi2_N1 = {0x3840,12,701,268,780,327,701,268};

static PageCut ZuanTouXiaYi3_Y1 = {0x3850,7,101,348,181,410,101,348};
static PageCut ZuanTouXiaYi3_N1 = {0x3850,12,101,348,181,410,101,348};

static PageCut ZuanTouXiaYi4_Y1 = {0x3860,7,181,348,281,410,181,348};
static PageCut ZuanTouXiaYi4_N1 = {0x3860,12,181,348,281,410,181,348};

static PageCut ZuanTouXiaYi5_Y1 = {0x3870,7,281,348,371,410,281,348};
static PageCut ZuanTouXiaYi5_N1 = {0x3870,12,281,348,371,410,281,348};

static PageCut ZuanTouXiaYi6_Y1 = {0x3880,7,371,348,440,455,374,350};
static PageCut ZuanTouXiaYi6_N1 = {0x3880,12,371,348,440,455,374,350};

static PageCut ZuanTouXiaYi7_Y1 = {0x3890,7,455,348,540,545,460,350};
static PageCut ZuanTouXiaYi7_N1 = {0x3890,12,455,348,540,545,460,350};

static PageCut ZuanTouXiaYi8_Y1 = {0x38A0,7,545,348,625,410,545,348};
static PageCut ZuanTouXiaYi8_N1 = {0x38A0,12,545,348,625,410,545,348};

extern u8 ChuJiao1,ChuJiao2,ChuJiao3,JiaoTouXiaYi1,JiaoTouXiaYi2,JiaoTouXiaYi3,
XiZuan1,XiZuan2,XiZuan3,XiZuan4,XiZuan5,XiZuan6,XiZuan7,XiZuan8,
ZuanTouXiaYi1,ZuanTouXiaYi2,ZuanTouXiaYi3,ZuanTouXiaYi4,ZuanTouXiaYi5,ZuanTouXiaYi6,ZuanTouXiaYi7,ZuanTouXiaYi8;
extern u32 IO_Choose;//高八位控制胶头或者钻头下移  低八位控制出胶和吸放钻
//extern u32 IO_Choose1;//从低位到高位代表  出胶1 2 3 胶头下移 1 2 3 吸钻1 2 3 4 5 6 7 8 钻头下移1 2 3 4 5 6 7 8
extern u8 LCD_Main_Page_Pre;
u8 IO_Parameter_Refresh=TRUE;
/*************************************************************************
**  函数名：  ManulOutputProcess()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
void ManulOutputProcess()
{
	switch(LCD_Var_Address)
	{
	/*------输出点界面1------*/
		case 0x3A00:						//Y0  
		     if(g_Output1_Status &0x0001)
			 {
			 	g_Output1_Status &= 0xfffe;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output1_Status |= 0x0001;		   
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_1;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;	
		case 0x3A01:					   //Y1
		     if(g_Output1_Status &0x0002)
			 {
			 	g_Output1_Status &= 0xfffd;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output1_Status |= 0x0002;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_2;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A02:						//Y2
		     if(g_Output1_Status &0x0004)
			 {
			 	g_Output1_Status &= 0xfffb;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output1_Status |= 0x0004;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_3;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A03:						//Y3
		     if(g_Output1_Status &0x0008)
			 {
			 	g_Output1_Status &= 0xfff7;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output1_Status |= 0x0008;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_4;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A04:					   //Y4
		     if(g_Output1_Status &0x0010)
			 {
			 	g_Output1_Status &= 0xffef;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output1_Status |= 0x0010;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_5;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A05:						 //Y5   
		     if(g_Output1_Status &0x0020)   
			 {
			 	g_Output1_Status &= 0xffdf;
				Order_Temp[1]=0;
			 }
			 else						   
			 {
				g_Output1_Status |= 0x0020;				
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_6;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A06:							   //Y6
		     if(g_Output1_Status &0x0040)  //关	  
			 {
			 	g_Output1_Status &= 0xffbf;
				Order_Temp[1]=0;
			 }
			 else						  //开
			 {
			    g_Output1_Status |= 0x0040;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_7;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A07:							 //Y7    
		     if(g_Output1_Status &0x0080)
			 {
			 	g_Output1_Status &= 0xff7f;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output1_Status |= 0x0080;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_8;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A08:							 //Y8   
		     if(g_Output1_Status &0x0100)
			 {
			 	g_Output1_Status &= 0xfeff;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output1_Status |= 0x0100;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_9;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A09:							//Y9
		     if(g_Output1_Status &0x0200)
			 {
			 	g_Output1_Status &= 0xfdff;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output1_Status |= 0x0200;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_10;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A0A:							//Y10
		     if(g_Output1_Status &0x0400)
			 {
			 	g_Output1_Status &= 0xfbff;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output1_Status |= 0x0400;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_11;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A0B:						   //预留1
		     break;
		case 0x3A0C:						//预留2
		     break;
		case 0x3A0D:					    //预留3
		     break;
		case 0x3A0E:						//预留4
		     break;

     /*------输出点界面2------*/
      //单阀无检测-------------------------------------
		case 0x3A10:						   //Y11
		     if(g_Output2_Status &0x0001)
			 {
			 	g_Output2_Status &= 0xfffe;
				Order_Temp[1]=0;
			 }
			 else
			 {
 			    g_Output2_Status |= 0x0001;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_12;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;	
		case 0x3A11:						  //Y12
		     if(g_Output2_Status &0x0002)
			 {
			 	g_Output2_Status &= 0xfffd;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output2_Status |= 0x0002;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_13;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A12:						 //Y13
		     if(g_Output2_Status &0x0004)
			 {
			 	g_Output2_Status &= 0xfffb;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output2_Status |= 0x0004;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_14;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A13:						  //Y14
		     if(g_Output2_Status &0x0008)
			 {
			 	g_Output2_Status &= 0xfff7;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output2_Status |= 0x0008;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT1_15;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
		     break;
		case 0x3A14:							  //Y15
		     if(g_Output2_Status &0x0010)
			 {
			 	g_Output2_Status &= 0xffef;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output2_Status |= 0x0010;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_1;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A15:							//Y16
		     if(g_Output2_Status &0x0020)
			 {
			 	g_Output2_Status &= 0xffdf;
				Order_Temp[1]=0;
			 }
			 else
			 {				   
			    g_Output2_Status |= 0x00020;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_2;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A16:						  //Y17
		     if(g_Output2_Status &0x0040)
			 {
			 	g_Output2_Status &= 0xffbf;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output2_Status |= 0x0040;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_3;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A17:						   //Y18
		     if(g_Output2_Status &0x0080)
			 {
			 	g_Output2_Status &= 0xff7f;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output2_Status |= 0x0080;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_4;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A18:					  //Y19
		     if(g_Output2_Status &0x0100)
			 {
			 	g_Output2_Status &= 0xfeff;
				Order_Temp[1]=0;
			 }
			 else
			 {
			    g_Output2_Status |= 0x0100;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_5;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A19:					  //预留1
		     break;
	  //双阀-------------------------------------------
		case 0x3A1A:					  //Y20-Y21
		     if(g_Output2_Status &0x0400)
			 {
			 	g_Output2_Status &= 0xfbff;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT2_7;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);			    
				g_Output2_Status &= 0xf7ff;	

			    g_Output2_Status |= 0x0400;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_6;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A1B:					  //Y21-Y20
		     if(g_Output2_Status &0x0800)
			 {
			 	g_Output2_Status &= 0xf7ff;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT2_6;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);			    
				g_Output2_Status &= 0xfbff;	

			    g_Output2_Status |= 0x0800;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_7;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A1C:							//Y22-Y23
		     if(g_Output2_Status &0x1000)
			 {
			 	g_Output2_Status &= 0xefff;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT2_9;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);			    
				g_Output2_Status &= 0xdfff;	

			    g_Output2_Status |= 0x1000;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_8;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A1D:							//Y23-Y22
		     if(g_Output2_Status &0x2000)
			 {
			 	g_Output2_Status &= 0xdfff;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT2_8;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);			    
				g_Output2_Status &= 0xefff;	

			    g_Output2_Status |= 0x2000;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_9;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A1E:			//预留1
		     break;

     /*------输出点3------*/
		case 0x3A20:						//Y24-Y25
		     if(g_Output3_Status &0x0001)
			 {
			 	g_Output3_Status &= 0xfffe;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT2_11;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);			    
				g_Output3_Status &= 0xfffd;	

			    g_Output3_Status |= 0x0001;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_10;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;	
		case 0x3A21:						//Y25-Y24
		     if(g_Output3_Status &0x0002)
			 {
			 	g_Output3_Status &= 0xfffd;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT2_10;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);			    
				g_Output3_Status &= 0xfffe;	

			    g_Output3_Status |= 0x0002;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_11;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A22:					   //Y26-Y27
		     if(g_Output3_Status &0x0004)
			 {
			 	g_Output3_Status &= 0xfffb;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT2_13;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);			    
				g_Output3_Status &= 0xfff7;

			    g_Output3_Status |= 0x0004;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_12;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A23:						  //Y27-Y26
		     if(g_Output3_Status &0x0008)
			 {
			 	g_Output3_Status &= 0xfff7;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT2_12;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);			    
				g_Output3_Status &= 0xfffb;

			    g_Output3_Status |= 0x0008;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_13;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A24:						   //Y28-Y29
		     if(g_Output3_Status &0x0010)
			 {
			 	g_Output3_Status &= 0xffef;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT2_15;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);			    
				g_Output3_Status &= 0xffdf;

			    g_Output3_Status |= 0x0010;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_14;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A25:							 //Y29-Y28
		     if(g_Output3_Status &0x0020)
			 {
			 	g_Output3_Status &= 0xffdf;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT2_14;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);			    
				g_Output3_Status &= 0xffef;

			    g_Output3_Status |= 0x00020;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT2_15;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT2,&Order_Temp[0]);
		     break;
		case 0x3A26:							 //Y30-Y31
		     if(g_Output3_Status &0x0040)
			 {
			 	g_Output3_Status &= 0xffbf;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT3_2;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);			    
				g_Output3_Status &= 0xff7f;

			    g_Output3_Status |= 0x0040;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT3_1;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);
		     break;
		case 0x3A27:							//Y31-Y30
		     if(g_Output3_Status &0x0080)
			 {
			 	g_Output3_Status &= 0xff7f;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT3_1;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);			    
				g_Output3_Status &= 0xffbf;

			    g_Output3_Status |= 0x0080;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT3_2;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);
		     break;
		case 0x3A28:						   //Y32-Y33
		     if(g_Output3_Status &0x0100)
			 {
			 	g_Output3_Status &= 0xfeff;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT3_4;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);			    
				g_Output3_Status &= 0xfdff;

			    g_Output3_Status |= 0x0100;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT3_3;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);
		     break;
		case 0x3A29:						//Y33-Y32
		     if(g_Output3_Status &0x0200)
			 {
			 	g_Output3_Status &= 0xfdff;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT3_3;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);			    
				g_Output3_Status &= 0xfeff;

			    g_Output3_Status |= 0x0200;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT3_4;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);
		     break;
		case 0x3A2A:						  //Y34-Y35
		     if(g_Output3_Status &0x0400)
			 {
			 	g_Output3_Status &= 0xfbff;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT3_6;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);			    
				g_Output3_Status &= 0xf7ff;

			    g_Output3_Status |= 0x0400;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT3_5;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);
		     break;
		case 0x3A2B:						  //Y35-Y34
		     if(g_Output3_Status &0x0800)
			 {
			 	g_Output3_Status &= 0xf7ff;
				Order_Temp[1]=0;
			 }
			 else
			 {
				Order_Temp[0]=IODEBUG_OUTPUT3_5;
				Order_Temp[1]=0;		 
				USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);			    
				g_Output3_Status &= 0xfbff;

			    g_Output3_Status |= 0x0800;
				Order_Temp[1]=1;
			 } 
		     Order_Temp[0]=IODEBUG_OUTPUT3_6;		 
			 USART1_SendData(2,IO_DEBUG_OUTPUT3,&Order_Temp[0]);
		     break;
		case 0x3A2C:						  //预留1
		     break;
		case 0x3A2D:						  //预留2
		     break;
		case 0x3A2E:						  //预留3
		     break;
	}
}

/*************************************************************************
**  函数名：  LCDIOControlProcess()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/5/28
**************************************************************************/
 void LCDIOControlProcess(void)
{
	if( (Robot_Origined == TRUE)&&(Robot_Status == ACTION_STOP) )	//机械手已回零，且处于停止状态
	{
		 //如果不是当前页面，则切换到当前界面
		if( (LCD_Var_Address == 0x3000) && (LCD_Main_Page != IO_CONTROL))
		{
			LCD_Main_Page_Pre = LCD_Main_Page;
			LCD_Main_Page = IO_CONTROL;
			LCD_Current_Page = 	LCD_Main_Page;					  
//			Order_Temp[0]=IO_MODE;
//			USART1_SendData(1,WORK_MODE,&Order_Temp[0]);
//			//查询输出IO状态
//			Order_Temp[0]=IO_DEBUG_OUTPUT1_LCD;
//			USART1_SendData(1,WATCH_COMMAND,&Order_Temp[0]);
			g_Robot_Auto_Reset = FALSE;
			//
		}
		switch(LCD_Var_Address)
		{
			case 0x3900:
				ChuJiao1=!ChuJiao1;
				if(ChuJiao1==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFFE)|0x00000001;//1
					Order_Temp[0]=1;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else
				{
					IO_Choose=(IO_Choose&0x00EFFFFE);
					Order_Temp[0]=1;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3901:
				ChuJiao2=!ChuJiao2;
				if(ChuJiao2==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFFD)|0x00000002;//2
					Order_Temp[0]=2;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else
				{
					IO_Choose=(IO_Choose&0x00EFFFFD);
					Order_Temp[0]=2;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);					
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3902:
				ChuJiao3=!ChuJiao3;
				if(ChuJiao3==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFFB)|0x00000004;//3
					Order_Temp[0]=3;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else
				{
					IO_Choose=(IO_Choose&0x00EFFFFB);
					Order_Temp[0]=3;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);					
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3903:
				JiaoTouXiaYi1=!JiaoTouXiaYi1;
				if(JiaoTouXiaYi1==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFF7)|0x00000008;//4
					Order_Temp[0]=4;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFFF7);
					Order_Temp[0]=4;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3904:
				JiaoTouXiaYi2=!JiaoTouXiaYi2;
				if(JiaoTouXiaYi2==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFEF)|0x00000010;//5
					Order_Temp[0]=5;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFFEF);//5
					Order_Temp[0]=5;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3905:
				JiaoTouXiaYi3=!JiaoTouXiaYi3;
				if(JiaoTouXiaYi3==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFDF)|0x00000020;//6
					Order_Temp[0]=6;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFFDF);//6
					Order_Temp[0]=6;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3906:
				XiZuan1=!XiZuan1;
				if(XiZuan1==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFBF)|0x00000040;//7
					Order_Temp[0]=7;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFFBF);//7
					Order_Temp[0]=7;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3907:
				XiZuan2=!XiZuan2;
				if(XiZuan2==1)
				{
					IO_Choose=(IO_Choose&0x00EFFF7F)|0x00000080;//8
					Order_Temp[0]=8;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFF7F);//8
					Order_Temp[0]=8;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3908:
				XiZuan3=!XiZuan3;
				if(XiZuan3==1)
				{
					IO_Choose=(IO_Choose&0x00EFFEFF)|0x00000100;//9
					Order_Temp[0]=9;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFEFF);//9
					Order_Temp[0]=9;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3909:
				XiZuan4=!XiZuan4;
				if(XiZuan4==1)
				{
					IO_Choose=(IO_Choose&0x00EFFDFF)|0x00000200;//10
					Order_Temp[0]=10;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFDFF);//10
					Order_Temp[0]=10;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x390A:
				XiZuan5=!XiZuan5;
				if(XiZuan5==1)
				{
					IO_Choose=(IO_Choose&0x00EFFBFF)|0x00000400;//11
					Order_Temp[0]=11;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFBFF);//11
					Order_Temp[0]=11;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x390B:
				XiZuan6=!XiZuan6;
				if(XiZuan6==1)
				{
					IO_Choose=(IO_Choose&0x00EFF7FF)|0x00000800;//13
					Order_Temp[0]=12;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFF7FF);//13
					Order_Temp[0]=12;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x390C:
				XiZuan7=!XiZuan7;
				if(XiZuan7==1)
				{
					IO_Choose=(IO_Choose&0x00EFEFFF)|0x00001000;//14
					Order_Temp[0]=13;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFEFFF);//14
					Order_Temp[0]=13;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x390D:
				XiZuan8=!XiZuan8;
				if(XiZuan8==1)
				{
					IO_Choose=(IO_Choose&0x00EFDFFF)|0x00002000;//15
					Order_Temp[0]=14;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFDFFF);//15
					Order_Temp[0]=14;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x390E:
				ZuanTouXiaYi1=!ZuanTouXiaYi1;
				if(ZuanTouXiaYi1==1)
				{
					IO_Choose=(IO_Choose&0x00EFBFFF)|0x00004000;//16
					Order_Temp[0]=15;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFBFFF);//16
					Order_Temp[0]=15;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x390F:
				ZuanTouXiaYi2=!ZuanTouXiaYi2;
				if(ZuanTouXiaYi2==1)
				{
					IO_Choose=(IO_Choose&0x00EF7FFF)|0x00008000;//17
					Order_Temp[0]=16;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EF7FFF);//17
					Order_Temp[0]=16;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3910:
				ZuanTouXiaYi3=!ZuanTouXiaYi3;
				if(ZuanTouXiaYi3==1)
				{
					IO_Choose=(IO_Choose&0x00EEFFFF)|0x00010000;//18
					Order_Temp[0]=17;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EEFFFF);//18
					Order_Temp[0]=17;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3911:
				ZuanTouXiaYi4=!ZuanTouXiaYi4;
				if(ZuanTouXiaYi4==1)
				{
					IO_Choose=(IO_Choose&0x00EDFFFF)|0x00020000;//19
					Order_Temp[0]=18;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EDFFFF);//19
					Order_Temp[0]=18;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3912:
				ZuanTouXiaYi5=!ZuanTouXiaYi5;
				if(ZuanTouXiaYi5==1)
				{
					IO_Choose=(IO_Choose&0x00EBFFFF)|0x00040000;//20
					Order_Temp[0]=19;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EBFFFF);//20
					Order_Temp[0]=19;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3913:
				ZuanTouXiaYi6=!ZuanTouXiaYi6;
				if(ZuanTouXiaYi6==1)
				{
					IO_Choose=(IO_Choose&0x00E7FFFF)|0x00080000;//21
					Order_Temp[0]=20;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00E7FFFF);//21
					Order_Temp[0]=20;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3914:
				ZuanTouXiaYi7=!ZuanTouXiaYi7;
				if(ZuanTouXiaYi7==1)
				{
					IO_Choose=(IO_Choose&0x00EFFFFF)|0x00100000;//22
					Order_Temp[0]=21;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00EFFFFF);//22
					Order_Temp[0]=21;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3915:
				ZuanTouXiaYi8=!ZuanTouXiaYi8;
				if(ZuanTouXiaYi8==1)
				{
					IO_Choose=(IO_Choose&0x00DFFFFF)|0x00200000;//23
					Order_Temp[0]=22;
					Order_Temp[1]=1;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				else 
				{
					IO_Choose=(IO_Choose&0x00DFFFFF);//23
					Order_Temp[0]=22;
					Order_Temp[1]=0;
					USART2_SendDataArray(3,IO_DEBUG_OUTPUT1,&Order_Temp[0]);
				}
				IO_Parameter_Refresh=TRUE;				
				break;
			case 0x3916:
				
				break;
			case 0x3917:
				
				break;
			case 0x3918:
				if(LCD_Main_Page_Pre==MANUL_DEBUG)
				{
					 LCD_Main_Page       = MANUL_DEBUG;                	                  
					 LCD_Current_Page    = 148;          			//LCD当前界面为手动调试						
				}
				else if(LCD_Main_Page_Pre==SYSTEM_SETTING)
				{
					 LCD_Main_Page       = SYSTEM_SETTING;                	                  
					 LCD_Current_Page    = LCD_Main_Page; 					
				}
				else if(LCD_Main_Page_Pre==FREE_PROGRAMME)
				{
					 LCD_Main_Page       = FREE_PROGRAMME;                	                  
					 LCD_Current_Page    = LCD_Main_Page; 					
				}
				else
				{
					 LCD_Main_Page       = START_UP;                	                  
					 LCD_Current_Page    = LCD_Main_Page;          			//LCD当前界面为手动调试		
				}
				break;
//			case 0x3000:  				         //本地IO
//				 g_Watch_IO_Control = IO_DEBUG_LOCAL;
//				 LCD_Current_Page = 4;
//				 break;
//			case 0x3001: 						 //本地IO
//				 g_Watch_IO_Control = IO_DEBUG_LOCAL;
//				 LCD_Current_Page = 4;
//				 break;	
//			case 0x3002: 				         //输入点1
//				 g_Watch_IO_Control = IO_DEBUG_INPUT1; 
//				 LCD_Current_Page = 6;
//				 break;	
//			case 0x3003: 				         //输出点1
//				 g_Watch_IO_Control = IO_DEBUG_OUTPUT1;					   
//				 LCD_Current_Page = 10;
//				 break;
//			case 0x3004: 				         //输入点1-下一页: 输入点2
//				 g_Watch_IO_Control = IO_DEBUG_INPUT2;
//				 LCD_Current_Page = 8;
//				 break;
//			case 0x3005: 						 //输入点2-上一页: 输入点1
//				 g_Watch_IO_Control = IO_DEBUG_INPUT1;
//				 LCD_Current_Page = 6;
//				 break;	
//			case 0x3006: 				         //输入点2-下一页: 输入点3
//				 g_Watch_IO_Control = IO_DEBUG_INPUT3; 
//				 LCD_Current_Page = 29;
//				 break;	
//			case 0x3007: 				         //输入点3-上一页: 输入点2
//				 g_Watch_IO_Control = IO_DEBUG_INPUT2;
//				 LCD_Current_Page = 8;
//				 break;
//			case 0x3008: 				         //输出点1-下一页：输出点2
//				 g_Watch_IO_Control = IO_DEBUG_OUTPUT2;
//				 LCD_Current_Page = 12;
//				 break;
//			case 0x3009: 				         //输出点2-上一页：输出点1
//				 g_Watch_IO_Control = IO_DEBUG_OUTPUT1; 
//				 LCD_Current_Page = 10;
//				 break;	
//			case 0x300A: 				         //输出点2-下一页：输出点3
//				 g_Watch_IO_Control = IO_DEBUG_OUTPUT3;
//				 LCD_Current_Page = 31;
//				 break;
//			case 0x300B: 				         //输出点3-上一页：输出点1
//				 g_Watch_IO_Control = IO_DEBUG_OUTPUT2;
//				 LCD_Current_Page = 12;
//				 break;
	 	    default:
			     break;		   	
		}
//	   /*----------输出按键按下-----------*/
//		if( (LCD_Var_Address & 0xff00) == 0x3A00)
//		{	
//			ManulOutputProcess();
//		}	
	}
	else
	{
		if(LCD_Current_Page == START_UP)	  //主页面
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_1_HOMEFIRST;	//弹出提示框-请先回零
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	//弹出提示框-请先停止机械手
			}
		}
		else if(LCD_Current_Page == 16)	  //报警第一页
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_6_HOMEFIRST1;	//弹出提示框-请先回零
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	//弹出提示框-请先停止机械手
			}
		}	
		else if(LCD_Current_Page == 34)	  //报警第二页
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_6_HOMEFIRST2;	//弹出提示框-请先回零
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	//弹出提示框-请先停止机械手
			}
		}	
		else if(LCD_Current_Page == SYSTEM_SETTING)	  //系统设置
		{
			if(Robot_Origined == FALSE)
			{
				LCD_Popup_Windows = PW_4_HOMEFIRST;	//弹出提示框-请先回零
			}
			else if(Robot_Status != ACTION_STOP)
			{
			    LCD_Popup_Windows = PW_1_STOPROBOT;	//弹出提示框-请先停止机械手
			}
		}
	}
}


/*************************************************************************
**  函数名：  LocalIOStatusDisplay()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void LocalIOStatusDisplay()
{
	//1 X37启动
	if((g_LocalIO_Status&0x0001) != (gs_LocalIO_Status&0x0001) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0001)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_1Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_1N);
		}
	}
	//2 X38停止
	if((g_LocalIO_Status&0x0002) != (gs_LocalIO_Status&0x0002) )	
	{
		if(g_LocalIO_Status&0x0002)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_2Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_2N);
		}
	}
	//3 X39复位			
	if((g_LocalIO_Status&0x0004) != (gs_LocalIO_Status&0x0004) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0004)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_3Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_3N);
		}
	}

	//4 X42--X原点			
	if((g_LocalIO_Status&0x0008) != (gs_LocalIO_Status&0x0008) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0008)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_4Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_4N);
		}
	}
	//5 XX  			
	if((g_LocalIO_Status&0x0010) != (gs_LocalIO_Status&0x0010) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0010)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_5Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_5N);
		}
	}
	//6 X43--Z原点		
	if((g_LocalIO_Status&0x0020) != (gs_LocalIO_Status&0x0020) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0020)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_6Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_6N);
		}
	}
	//7 XX			
	if((g_LocalIO_Status&0x0040) != (gs_LocalIO_Status&0x0040) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0040)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_7Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_7N);
		}
	}
	//8 X41--Y原点		
	if((g_LocalIO_Status&0x0080) != (gs_LocalIO_Status&0x0080) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0080)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_8Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_8N);
		}
	}
	//9 XX			
	if((g_LocalIO_Status&0x0100) != (gs_LocalIO_Status&0x0100) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0100)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_9Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_9N);
		}
	}
	//10 O轴原点			
	if((g_LocalIO_Status&0x0200) != (gs_LocalIO_Status&0x0200) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0200)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_10Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_10N);
		}
	}
	//11 XX			
	if((g_LocalIO_Status&0x0400) != (gs_LocalIO_Status&0x0400) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0400)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_11Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_11N);
		}
	}
	//12 XX			
	if((g_LocalIO_Status&0x0800) != (gs_LocalIO_Status&0x0800) )	//第一个按钮与之前状态不相同
	{
		if(g_LocalIO_Status&0x0800)	  //当前位为1，则显示效果
		{
			Display_PageCut(L_PageCut_Local_12Y);
		}
		else
		{
			Display_PageCut(L_PageCut_Local_12N);
		}
	}			
}

/*************************************************************************
**  函数名：  Input1StatusDisplay()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void Input1StatusDisplay()
{
	//X0
	if((g_Input1_Status&0x0001) != (gs_Input1_Status&0x0001) )	
	{
		if(g_Input1_Status&0x0001)	  //当前位为1，则显示效果
		{
			Display_PageCut(I_PageCut_Input1_1Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_1N);
		}
	}
	//X1 
	if((g_Input1_Status&0x0002) != (gs_Input1_Status&0x0002) )	
	{
		if(g_Input1_Status&0x0002)	  
		{
			Display_PageCut(I_PageCut_Input1_2Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_2N);
		}
	}

	//X2 
	if((g_Input1_Status&0x0004) != (gs_Input1_Status&0x0004) )	
	{
		if(g_Input1_Status&0x0004)	  
		{
			Display_PageCut(I_PageCut_Input1_3Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_3N);
		}
	}
	//X3 
	if((g_Input1_Status&0x0008) != (gs_Input1_Status&0x0008) )	
	{
		if(g_Input1_Status&0x0008)	  
		{
			Display_PageCut(I_PageCut_Input1_4Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_4N);
		}
	}
	//X4 
	if((g_Input1_Status&0x0010) != (gs_Input1_Status&0x0010) )	
	{
		if(g_Input1_Status&0x0010)	  
		{
			Display_PageCut(I_PageCut_Input1_5Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_5N);
		}
	}
	//X5 
	if((g_Input1_Status&0x0020) != (gs_Input1_Status&0x0020) )	
	{
		if(g_Input1_Status&0x0020)	  
		{
			Display_PageCut(I_PageCut_Input1_6Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_6N);
		}
	}
	//X6 
	if((g_Input1_Status&0x0040) != (gs_Input1_Status&0x0040) )	
	{
		if(g_Input1_Status&0x0040)	  
		{
			Display_PageCut(I_PageCut_Input1_7Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_7N);
		}
	}
	//X7 
	if((g_Input1_Status&0x0080) != (gs_Input1_Status&0x0080) )	
	{
		if(g_Input1_Status&0x0080)	  
		{
			Display_PageCut(I_PageCut_Input1_8Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_8N);
		}
	}		
	//X8 
	if((g_Input1_Status&0x0100) != (gs_Input1_Status&0x0100) )	
	{
		if(g_Input1_Status&0x0100)	  
		{
			Display_PageCut(I_PageCut_Input1_9Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_9N);
		}
	}
	//X9 
	if((g_Input1_Status&0x0200) != (gs_Input1_Status&0x0200) )	
	{
		if(g_Input1_Status&0x0200)	  
		{
			Display_PageCut(I_PageCut_Input1_10Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_10N);
		}
	}
	//X10 
	if((g_Input1_Status&0x0400) != (gs_Input1_Status&0x0400) )	
	{
		if(g_Input1_Status&0x0400)	  
		{
			Display_PageCut(I_PageCut_Input1_11Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_11N);
		}
	}
	//X11 
	if((g_Input1_Status&0x0800) != (gs_Input1_Status&0x0800) )	
	{
		if(g_Input1_Status&0x0800)	  
		{
			Display_PageCut(I_PageCut_Input1_12Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_12N);
		}
	}
	//X12
	if((g_Input1_Status&0x1000) != (gs_Input1_Status&0x1000) )	
	{
		if(g_Input1_Status&0x1000)	  
		{
			Display_PageCut(I_PageCut_Input1_13Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_13N);
		}
	}
	//X13
	if((g_Input1_Status&0x2000) != (gs_Input1_Status&0x2000) )	
	{
		if(g_Input1_Status&0x2000)	  
		{
			Display_PageCut(I_PageCut_Input1_14Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_14N);
		}
	}
	//X14
	if((g_Input1_Status&0x4000) != (gs_Input1_Status&0x4000) )	
	{
		if(g_Input1_Status&0x4000)	  
		{
			Display_PageCut(I_PageCut_Input1_15Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input1_15N);
		}
	}
}

/*************************************************************************
**  函数名：  Input2StatusDisplay()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void Input2StatusDisplay()
{
	//X15
	if((g_Input2_Status&0x0001) != (gs_Input2_Status&0x0001) )	
	{
		if(g_Input2_Status&0x0001)	  //当前位为1，则显示效果
		{
			Display_PageCut(I_PageCut_Input2_1Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_1N);
		}
	}
	// 
	if((g_Input2_Status&0x0002) != (gs_Input2_Status&0x0002) )	
	{
		if(g_Input2_Status&0x0002)	  
		{
			Display_PageCut(I_PageCut_Input2_2Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_2N);
		}
	}

	// 
	if((g_Input2_Status&0x0004) != (gs_Input2_Status&0x0004) )	
	{
		if(g_Input2_Status&0x0004)	  
		{
			Display_PageCut(I_PageCut_Input2_3Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_3N);
		}
	}
	// 
	if((g_Input2_Status&0x0008) != (gs_Input2_Status&0x0008) )	
	{
		if(g_Input2_Status&0x0008)	  
		{
			Display_PageCut(I_PageCut_Input2_4Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_4N);
		}
	}
	// 
	if((g_Input2_Status&0x0010) != (gs_Input2_Status&0x0010) )	
	{
		if(g_Input2_Status&0x0010)	  
		{
			Display_PageCut(I_PageCut_Input2_5Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_5N);
		}
	}
	//X16
	if((g_Input2_Status&0x0020) != (gs_Input2_Status&0x0020) )	
	{
		if(g_Input2_Status&0x0020)	  
		{
			Display_PageCut(I_PageCut_Input2_6Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_6N);
		}
	}
	//X17
	if((g_Input2_Status&0x0040) != (gs_Input2_Status&0x0040) )	
	{
		if(g_Input2_Status&0x0040)	  
		{
			Display_PageCut(I_PageCut_Input2_7Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_7N);
		}
	}
	//X18
	if((g_Input2_Status&0x0080) != (gs_Input2_Status&0x0080) )	
	{
		if(g_Input2_Status&0x0080)	  
		{
			Display_PageCut(I_PageCut_Input2_8Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_8N);
		}
	}		
	//X19
	if((g_Input2_Status&0x0100) != (gs_Input2_Status&0x0100) )	
	{
		if(g_Input2_Status&0x0100)	  
		{
			Display_PageCut(I_PageCut_Input2_9Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_9N);
		}
	}
	//X20
	if((g_Input2_Status&0x0200) != (gs_Input2_Status&0x0200) )	
	{
		if(g_Input2_Status&0x0200)	  
		{
			Display_PageCut(I_PageCut_Input2_10Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_10N);
		}
	}
	//X21
	if((g_Input2_Status&0x0400) != (gs_Input2_Status&0x0400) )	
	{
		if(g_Input2_Status&0x0400)	  
		{
			Display_PageCut(I_PageCut_Input2_11Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_11N);
		}
	}
	//X22
	if((g_Input2_Status&0x0800) != (gs_Input2_Status&0x0800) )	
	{
		if(g_Input2_Status&0x0800)	  
		{
			Display_PageCut(I_PageCut_Input2_12Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_12N);
		}
	}
	//X23
	if((g_Input2_Status&0x1000) != (gs_Input2_Status&0x1000) )	
	{
		if(g_Input2_Status&0x1000)	  
		{
			Display_PageCut(I_PageCut_Input2_13Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_13N);
		}
	}
	//X24
	if((g_Input2_Status&0x2000) != (gs_Input2_Status&0x2000) )	
	{
		if(g_Input2_Status&0x2000)	  
		{
			Display_PageCut(I_PageCut_Input2_14Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_14N);
		}
	}
	//X25
	if((g_Input2_Status&0x4000) != (gs_Input2_Status&0x4000) )	
	{
		if(g_Input2_Status&0x4000)	  
		{
			Display_PageCut(I_PageCut_Input2_15Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input2_15N);
		}
	}
}

/*************************************************************************
**  函数名：  Input3StatusDisplay()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void Input3StatusDisplay()
{
	//X26
	if((g_Input3_Status&0x0001) != (gs_Input3_Status&0x0001) )	
	{
		if(g_Input3_Status&0x0001)	  //当前位为1，则显示效果
		{
			Display_PageCut(I_PageCut_Input3_1Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_1N);
		}
	}
	//X27
	if((g_Input3_Status&0x0002) != (gs_Input3_Status&0x0002) )	
	{
		if(g_Input3_Status&0x0002)	  
		{
			Display_PageCut(I_PageCut_Input3_2Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_2N);
		}
	}
    //X28
	if((g_Input3_Status&0x0004) != (gs_Input3_Status&0x0004) )	
	{
		if(g_Input3_Status&0x0004)	  
		{
			Display_PageCut(I_PageCut_Input3_3Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_3N);
		}
	}
	//X29
	if((g_Input3_Status&0x0008) != (gs_Input3_Status&0x0008) )	
	{
		if(g_Input3_Status&0x0008)	  
		{
			Display_PageCut(I_PageCut_Input3_4Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_4N);
		}
	}
	//X30
	if((g_Input3_Status&0x0010) != (gs_Input3_Status&0x0010) )	
	{
		if(g_Input3_Status&0x0010)	  
		{
			Display_PageCut(I_PageCut_Input3_5Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_5N);
		}
	}
	//X31
	if((g_Input3_Status&0x0020) != (gs_Input3_Status&0x0020) )	
	{
		if(g_Input3_Status&0x0020)	  
		{
			Display_PageCut(I_PageCut_Input3_6Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_6N);
		}
	}
	//  
	if((g_Input3_Status&0x0040) != (gs_Input3_Status&0x0040) )	
	{
		if(g_Input3_Status&0x0040)	  
		{
			Display_PageCut(I_PageCut_Input3_7Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_7N);
		}
	}
	// 
	if((g_Input3_Status&0x0080) != (gs_Input3_Status&0x0080) )	
	{
		if(g_Input3_Status&0x0080)	  
		{
			Display_PageCut(I_PageCut_Input3_8Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_8N);
		}
	}		
	// 
	if((g_Input3_Status&0x0100) != (gs_Input3_Status&0x0100) )	
	{
		if(g_Input3_Status&0x0100)	  
		{
			Display_PageCut(I_PageCut_Input3_9Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_9N);
		}
	}
	// 
	if((g_Input3_Status&0x0200) != (gs_Input3_Status&0x0200) )	
	{
		if(g_Input3_Status&0x0200)	  
		{
			Display_PageCut(I_PageCut_Input3_10Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_10N);
		}
	}
	//X32
	if((g_Input3_Status&0x0400) != (gs_Input3_Status&0x0400) )	
	{
		if(g_Input3_Status&0x0400)	  
		{
			Display_PageCut(I_PageCut_Input3_11Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_11N);
		}
	}
	//X33
	if((g_Input3_Status&0x0800) != (gs_Input3_Status&0x0800) )	
	{
		if(g_Input3_Status&0x0800)	  
		{
			Display_PageCut(I_PageCut_Input3_12Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_12N);
		}
	}
	//X34
	if((g_Input3_Status&0x1000) != (gs_Input3_Status&0x1000) )	
	{
		if(g_Input3_Status&0x1000)	  
		{
			Display_PageCut(I_PageCut_Input3_13Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_13N);
		}
	}
	//X35
	if((g_Input3_Status&0x2000) != (gs_Input3_Status&0x2000) )	
	{
		if(g_Input3_Status&0x2000)	  
		{
			Display_PageCut(I_PageCut_Input3_14Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_14N);
		}
	}
	// 
	if((g_Input3_Status&0x4000) != (gs_Input3_Status&0x4000) )	
	{
		if(g_Input3_Status&0x4000)	  
		{
			Display_PageCut(I_PageCut_Input3_15Y);
		}
		else
		{
			Display_PageCut(I_PageCut_Input3_15N);
		}
	}
}

/*************************************************************************
**  函数名：  Output1StatusDisplay()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void Output1StatusDisplay()
{
	//1 
	if((g_Output1_Status&0x0001) != (gs_Output1_Status&0x0001) )	
	{
		if(g_Output1_Status&0x0001)	  //当前位为1，则显示效果
		{
			Display_PageCut(O_PageCut_LC_LoadFinish_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_LC_LoadFinish_N);
		}
	}
	//2
	if((g_Output1_Status&0x0002) != (gs_Output1_Status&0x0002) )	
	{
		if(g_Output1_Status&0x0002)	  
		{
			Display_PageCut(O_PageCut_LC_UnloadFinish_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_LC_UnloadFinish_N);
		}
	}
	//3
	if((g_Output1_Status&0x0004) != (gs_Output1_Status&0x0004) )	
	{
		if(g_Output1_Status&0x0004)	  
		{
			Display_PageCut(O_PageCut_LC_LoadAlarm_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_LC_LoadAlarm_N);
		}
	}
	//4
	if((g_Output1_Status&0x0008) != (gs_Output1_Status&0x0008) )	
	{
		if(g_Output1_Status&0x0008)	  
		{
			Display_PageCut(O_PageCut_LC_UnloadAlarm_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_LC_UnloadAlarm_N);
		}
	}
	//5
	if((g_Output1_Status&0x0010) != (gs_Output1_Status&0x0010) )	
	{
		if(g_Output1_Status&0x0010)	  
		{
			Display_PageCut(O_PageCut_LC_Reserve1_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_LC_Reserve1_N);
		}
	}
	//6
	if((g_Output1_Status&0x0020) != (gs_Output1_Status&0x0020) )	
	{
		if(g_Output1_Status&0x0020)	  
		{
			Display_PageCut(O_PageCut_M1_KaPanAOpen_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M1_KaPanAOpen_N);
		}
	}
	//7
	if((g_Output1_Status&0x0040) != (gs_Output1_Status&0x0040) )	
	{
		if(g_Output1_Status&0x0040)	  
		{
			Display_PageCut(O_PageCut_M1_KaPanAClose_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M1_KaPanAClose_N);
		}
	}
	//8
	if((g_Output1_Status&0x0080) != (gs_Output1_Status&0x0080) )	
	{
		if(g_Output1_Status&0x0080)	  
		{
			Display_PageCut(O_PageCut_M1_Blow_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M1_Blow_N);
		}
	}		
	//9 
	if((g_Output1_Status&0x0100) != (gs_Output1_Status&0x0100) )	
	{
		if(g_Output1_Status&0x0100)	  
		{
			Display_PageCut(O_PageCut_M1_AlawStart_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M1_AlawStart_N);
		}
	}
	//10 
	if((g_Output1_Status&0x0200) != (gs_Output1_Status&0x0200) )	
	{
		if(g_Output1_Status&0x0200)	  
		{
			Display_PageCut(O_PageCut_M1_StartProcess_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M1_StartProcess_N);
		}
	}
	//11
	if((g_Output1_Status&0x0400) != (gs_Output1_Status&0x0400) )	
	{
		if(g_Output1_Status&0x0400)	  
		{
			Display_PageCut(O_PageCut_M1_DoorOpen_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M1_DoorOpen_N);
		}
	}
	//12
	if((g_Output1_Status&0x0800) != (gs_Output1_Status&0x0800) )	
	{
		if(g_Output1_Status&0x0800)	  
		{
			Display_PageCut(O_PageCut_M1_DoorClose_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M1_DoorClose_N);
		}
	}
	//13
	if((g_Output1_Status&0x1000) != (gs_Output1_Status&0x1000) )	
	{
		if(g_Output1_Status&0x1000)	  
		{
			Display_PageCut(O_PageCut_M1_Reserve1_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M1_Reserve1_N);
		}
	}
	//14
	if((g_Output1_Status&0x2000) != (gs_Output1_Status&0x2000) )	
	{
		if(g_Output1_Status&0x2000)	  
		{
			Display_PageCut(O_PageCut_M1_Reserve2_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M1_Reserve2_N);
		}
	}
	//15
	if((g_Output1_Status&0x4000) != (gs_Output1_Status&0x4000) )	
	{
		if(g_Output1_Status&0x4000)	  
		{
			Display_PageCut(O_PageCut_M1_Reserve3_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M1_Reserve3_N);
		}
	}
}

/*************************************************************************
**  函数名：  Output2StatusDisplay()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void Output2StatusDisplay()
{
	//机床2-卡盘A开
	if((g_Output2_Status&0x0001) != (gs_Output2_Status&0x0001) )	
	{		   
		if(g_Output2_Status&0x0001)	  //当前位为1，则显示效果
		{
			Display_PageCut(O_PageCut_M2_KaPanAOpen_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M2_KaPanAOpen_N);
		}
	}
	//机床2-卡盘A关
	if((g_Output2_Status&0x0002) != (gs_Output2_Status&0x0002) )	
	{
		if(g_Output2_Status&0x0002)	  
		{
			Display_PageCut(O_PageCut_M2_KaPanAClose_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M2_KaPanAClose_N);
		}
	}
	//机床2-吹气
	if((g_Output2_Status&0x0004) != (gs_Output2_Status&0x0004) )	
	{
		if(g_Output2_Status&0x0004)	  
		{
			Display_PageCut(O_PageCut_M2_Blow_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M2_Blow_N);
		}
	}
	//机床2-允许启动
	if((g_Output2_Status&0x0008) != (gs_Output2_Status&0x0008) )	
	{
		if(g_Output2_Status&0x0008)	  
		{
			Display_PageCut(O_PageCut_M2_AlawStart_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M2_AlawStart_N);
		}
	}
	//机床2-开始加工
	if((g_Output2_Status&0x0010) != (gs_Output2_Status&0x0010) )	
	{
		if(g_Output2_Status&0x0010)	  
		{
			Display_PageCut(O_PageCut_M2_StartProcess_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M2_StartProcess_N);
		}
	}
	//机床2-顶门开
	if((g_Output2_Status&0x0020) != (gs_Output2_Status&0x0020) )	
	{
		if(g_Output2_Status&0x0020)	  
		{
			Display_PageCut(O_PageCut_M2_DoorOpen_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M2_DoorOpen_N);
		}
	}
	//机床2-顶门关
	if((g_Output2_Status&0x0040) != (gs_Output2_Status&0x0040) )	
	{
		if(g_Output2_Status&0x0040)	  
		{
			Display_PageCut(O_PageCut_M2_DoorClose_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M2_DoorClose_N);
		}
	}
	//机床2-预留1
	if((g_Output2_Status&0x0080) != (gs_Output2_Status&0x0080) )	
	{
		if(g_Output2_Status&0x0080)	  
		{
			Display_PageCut(O_PageCut_M2_Reserve1_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M2_Reserve1_N);
		}
	}		
	//机床2-预留2
	if((g_Output2_Status&0x0100) != (gs_Output2_Status&0x0100) )	
	{
		if(g_Output2_Status&0x0100)	  
		{
			Display_PageCut(O_PageCut_M2_Reserve2_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M2_Reserve2_N);
		}
	}
	//机床2-预留3
	if((g_Output2_Status&0x0200) != (gs_Output2_Status&0x0200) )	
	{
		if(g_Output2_Status&0x0200)	  
		{
			Display_PageCut(O_PageCut_M2_Reserve3_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_M2_Reserve3_N);
		}
	}
	//翻转台-正摆
	if((g_Output2_Status&0x0400) != (gs_Output2_Status&0x0400) )	
	{
		if(g_Output2_Status&0x0400)	  
		{
			Display_PageCut(O_PageCut_FZT_Front_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_FZT_Front_N);
		}
	}
	//翻转台-反摆
	if((g_Output2_Status&0x0800) != (gs_Output2_Status&0x0800) )	
	{
		if(g_Output2_Status&0x0800)	  
		{
			Display_PageCut(O_PageCut_FZT_Back_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_FZT_Back_N);
		}
	}
	//翻转台-松开
	if((g_Output2_Status&0x1000) != (gs_Output2_Status&0x1000) )	
	{
		if(g_Output2_Status&0x1000)	  
		{
			Display_PageCut(O_PageCut_FZT_Release_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_FZT_Release_N);
		}
	}
	//翻转台-夹紧
	if((g_Output2_Status&0x2000) != (gs_Output2_Status&0x2000) )	
	{
		if(g_Output2_Status&0x2000)	  
		{
			Display_PageCut(O_PageCut_FZT_Tight_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_FZT_Tight_N);
		}
	}
	//翻转台-预留1
	if((g_Output2_Status&0x4000) != (gs_Output2_Status&0x4000) )	
	{
		if(g_Output2_Status&0x4000)	  
		{
			Display_PageCut(O_PageCut_FZT_Reserve1_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_FZT_Reserve1_N);
		}
	}
}

/*************************************************************************
**  函数名：  Output3StatusDisplay()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void Output3StatusDisplay()
{
	//料爪-上料爪松
	if((g_Output3_Status&0x0001) != (gs_Output3_Status&0x0001) )	
	{		   
		if(g_Output3_Status&0x0001)	  //当前位为1，则显示效果
		{
			Display_PageCut(O_PageCut_CL_LoadOpen_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_CL_LoadOpen_N);
		}
	}
	//料爪-上料爪紧
	if((g_Output3_Status&0x0002) != (gs_Output3_Status&0x0002) )	
	{
		if(g_Output3_Status&0x0002)	  
		{
			Display_PageCut(O_PageCut_CL_LoadClose_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_CL_LoadClose_N);
		}
	}
	//料爪-卸料爪松
	if((g_Output3_Status&0x0004) != (gs_Output3_Status&0x0004) )	
	{
		if(g_Output3_Status&0x0004)	  
		{
			Display_PageCut(O_PageCut_CL_UnloadOpen_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_CL_UnloadOpen_N);
		}
	}
	//料爪-卸料爪紧
	if((g_Output3_Status&0x0008) != (gs_Output3_Status&0x0008) )	
	{
		if(g_Output3_Status&0x0008)	  
		{
			Display_PageCut(O_PageCut_CL_UnloadClose_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_CL_UnloadClose_N);
		}
	}
	//料爪-摆缸正摆
	if((g_Output3_Status&0x0010) != (gs_Output3_Status&0x0010) )	
	{
		if(g_Output3_Status&0x0010)	  
		{
			Display_PageCut(O_PageCut_CL_BG_Front_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_CL_BG_Front_N);
		}
	}
	//料爪-摆缸复位
	if((g_Output3_Status&0x0020) != (gs_Output3_Status&0x0020) )	
	{
		if(g_Output3_Status&0x0020)	  
		{
			Display_PageCut(O_PageCut_CL_BG_Reset_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_CL_BG_Reset_N);
		}
	}
	//料爪-Y向气缸伸出
	if((g_Output3_Status&0x0040) != (gs_Output3_Status&0x0040) )	
	{
		if(g_Output3_Status&0x0040)	  
		{
			Display_PageCut(O_PageCut_CL_YQG_Extend_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_CL_YQG_Extend_N);
		}
	}
	//料爪-Y向气缸缩回
	if((g_Output3_Status&0x0080) != (gs_Output3_Status&0x0080) )	
	{
		if(g_Output3_Status&0x0080)	  
		{
			Display_PageCut(O_PageCut_CL_YQG_Retraction_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_CL_YQG_Retraction_N);
		}
	}		
	//料爪-手抓吹气
	if((g_Output3_Status&0x0100) != (gs_Output3_Status&0x0100) )	
	{
		if(g_Output3_Status&0x0100)	  
		{
			Display_PageCut(O_PageCut_CL_Blow_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_CL_Blow_N);
		}
	}
	//料爪-预留1
	if((g_Output3_Status&0x0200) != (gs_Output3_Status&0x0200) )	
	{
		if(g_Output3_Status&0x0200)	  
		{
			Display_PageCut(O_PageCut_CL_Reserve1_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_CL_Reserve1_N);
		}
	}
	//其他-润滑使能
	if((g_Output3_Status&0x0400) != (gs_Output3_Status&0x0400) )	
	{
		if(g_Output3_Status&0x0400)	  
		{
			Display_PageCut(O_PageCut_QT_OilingEnable_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_QT_OilingEnable_N);
		}
	}
	//其他-运行灯
	if((g_Output3_Status&0x0800) != (gs_Output3_Status&0x0800) )	
	{
		if(g_Output3_Status&0x0800)	  
		{
			Display_PageCut(O_PageCut_QT_RunLight_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_QT_RunLight_N);
		}
	}
	//其他-待机灯
	if((g_Output3_Status&0x1000) != (gs_Output3_Status&0x1000) )	
	{
		if(g_Output3_Status&0x1000)	  
		{
			Display_PageCut(O_PageCut_QT_WaitLight_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_QT_WaitLight_N);
		}
	}
	//其他-报警灯
	if((g_Output3_Status&0x2000) != (gs_Output3_Status&0x2000) )	
	{
		if(g_Output3_Status&0x2000)	  
		{
			Display_PageCut(O_PageCut_QT_AlarmLight_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_QT_AlarmLight_N);
		}
	}
	//其他-预留1
	if((g_Output3_Status&0x4000) != (gs_Output3_Status&0x4000) )	
	{
		if(g_Output3_Status&0x4000)	  
		{
			Display_PageCut(O_PageCut_QT_Reserve1_Y);
		}
		else
		{
			Display_PageCut(O_PageCut_QT_Reserve1_N);
		}
	}
}

/*************************************************************************
**  函数名：  LCDIOControlDisply()
**	输入参数：
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    wx   
**  开发日期：2015/6/26
**************************************************************************/
void LCDIOControlDisply()
{
	//--界面切换--//
	if(LCD_Pre_Page != LCD_Current_Page )	   
	{
		LCD_Pre_Page = LCD_Current_Page;
		PageChange(LCD_Pre_Page);
	}
	if(IO_Parameter_Refresh)
	{
			//IO组选择
			if(ChuJiao1)
			Display_PageCut(ChuJiao1_Y1);
			else
			Display_PageCut(ChuJiao1_N1);
			if(ChuJiao2)
			Display_PageCut(ChuJiao2_Y1);
			else
			Display_PageCut(ChuJiao2_N1);
			if(ChuJiao3)
			Display_PageCut(ChuJiao3_Y1);
			else
			Display_PageCut(ChuJiao3_N1);
			if(JiaoTouXiaYi1)
			Display_PageCut(JiaoTouXiaYi1_Y1);
			else
			Display_PageCut(JiaoTouXiaYi1_N1);
			if(JiaoTouXiaYi2)
			Display_PageCut(JiaoTouXiaYi2_Y1);
			else
			Display_PageCut(JiaoTouXiaYi2_N1);
			if(JiaoTouXiaYi3)
			Display_PageCut(JiaoTouXiaYi3_Y1);
			else
			Display_PageCut(JiaoTouXiaYi3_N1);
			
			
			if(XiZuan1)
			Display_PageCut(XiZuan1_Y1);
			else
			Display_PageCut(XiZuan1_N1);
			if(XiZuan2)
			Display_PageCut(XiZuan2_Y1);
			else
			Display_PageCut(XiZuan2_N1);
			if(XiZuan3)
			Display_PageCut(XiZuan3_Y1);
			else
			Display_PageCut(XiZuan3_N1);
			if(XiZuan4)
			Display_PageCut(XiZuan4_Y1);
			else
			Display_PageCut(XiZuan4_N1);
			if(XiZuan5)
			Display_PageCut(XiZuan5_Y1);
			else
			Display_PageCut(XiZuan5_N1);
			if(XiZuan6)
			Display_PageCut(XiZuan6_Y1);
			else
			Display_PageCut(XiZuan6_N1);
			if(XiZuan7)
			Display_PageCut(XiZuan7_Y1);
			else
			Display_PageCut(XiZuan7_N1);
			if(XiZuan8)
			Display_PageCut(XiZuan8_Y1);
			else
			Display_PageCut(XiZuan8_N1);
			
			
			if(ZuanTouXiaYi1)
			Display_PageCut(ZuanTouXiaYi1_Y1);
			else
			Display_PageCut(ZuanTouXiaYi1_N1);
			if(ZuanTouXiaYi2)
			Display_PageCut(ZuanTouXiaYi2_Y1);
			else
			Display_PageCut(ZuanTouXiaYi2_N1);
			if(ZuanTouXiaYi3)
			Display_PageCut(ZuanTouXiaYi3_Y1);
			else
			Display_PageCut(ZuanTouXiaYi3_N1);
			if(ZuanTouXiaYi4)
			Display_PageCut(ZuanTouXiaYi4_Y1);
			else
			Display_PageCut(ZuanTouXiaYi4_N1);
			if(ZuanTouXiaYi5)
			Display_PageCut(ZuanTouXiaYi5_Y1);
			else
			Display_PageCut(ZuanTouXiaYi5_N1);
			if(ZuanTouXiaYi6)
			Display_PageCut(ZuanTouXiaYi6_Y1);
			else
			Display_PageCut(ZuanTouXiaYi6_N1);
			if(ZuanTouXiaYi7)
			Display_PageCut(ZuanTouXiaYi7_Y1);
			else
			Display_PageCut(ZuanTouXiaYi7_N1);
			if(ZuanTouXiaYi8)
			Display_PageCut(ZuanTouXiaYi8_Y1);
			else
			Display_PageCut(ZuanTouXiaYi8_N1);
			IO_Parameter_Refresh=FALSE;
		}
}
//   /*---本地IO---*/ 
//    if(g_LocalIO_Status != gs_LocalIO_Status)	 //本地IO发生变化
//	{
//		LocalIOStatusDisplay();
//		gs_LocalIO_Status = g_LocalIO_Status;
//	}
//    /*---输入1---*/   
//    if(g_Input1_Status != gs_Input1_Status)	 //输入1发生变化
//	{
//		Input1StatusDisplay();
//		gs_Input1_Status = g_Input1_Status;
//	}	
//   /*---输入2---*/ 
//    if(g_Input2_Status != gs_Input2_Status)	 //输入2发生变化
//	{
//		Input2StatusDisplay();
// 		gs_Input2_Status = g_Input2_Status;
//	}	
//    /*---输入3---*/ 
//    if(g_Input3_Status != gs_Input3_Status)	 //输入3发生变化
//	{
//		Input3StatusDisplay();
//		gs_Input3_Status = g_Input3_Status;
//	}	
//    /*---输出1---*/   
//    if((g_Output1_Status != gs_Output1_Status) && (g_Output1Ack_Flag == TRUE))	 //输入1发生变化
//	{
//		Output1StatusDisplay();
//		gs_Output1_Status = g_Output1_Status;
//		g_Output1Ack_Flag = FALSE;
//	}	
//    /*---输出2---*/ 
//    if((g_Output2_Status != gs_Output2_Status) && (g_Output1Ack_Flag == TRUE))	 //输入2发生变化
//	{
//		Output2StatusDisplay();
// 		gs_Output2_Status = g_Output2_Status;
//		g_Output2Ack_Flag = FALSE;
//	}	
//    /*---输出3---*/ 
//    if((g_Output3_Status != gs_Output3_Status) && (g_Output1Ack_Flag == TRUE))	 //输入3发生变化
//	{
//		Output3StatusDisplay();
//		gs_Output3_Status = g_Output3_Status;
//		g_Output3Ack_Flag = FALSE;
//	}	




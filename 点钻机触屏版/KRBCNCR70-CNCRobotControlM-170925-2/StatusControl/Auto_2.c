/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ************************
* File Name          : Auto_2.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 21/10/2015
* Description        : 自动运行控制文件
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Auto_2.h"
#include "Parameter.h"
#include "ActionOperate.h"
#include "StatusControl.h"
#include "Auto.h"
#include "in.h"
#include "out.h"
#include "PDO.h"
#include "w25qxx.h"
#include "Usart.h" 
#include "SpeedControl.h" 

#include "Delay.h"
#include "SpeedControl_CAN.h"
/***************************公共参数，供上位机查询与访问**************************************/
u8  g_AutoStatue = AUTO_WAITE;		//自动运行状态
u16  g_Auto_PresentLine = 0;			//自动运行行号
u32 g_Auto_LastCircleTime = 0;
u32 g_Auto_Circle_Timer = 0;		//自动运行周期时间
u32 g_Auto_ProduceCounter = 0;
u8  g_LC_Program_Step_Run=FALSE;    //是否运行料仓程序，在主程序中置位
u8  g_LC_Step_Control=FALSE;		//标识是否是lc动作，与自动动作区分
u8  g_LC_Load_Finish=FALSE;			//取料完成
u8  g_LC_Load_Permit=FALSE;			//允许取料
u8  g_LC_UnLoad_Finish=FALSE;		//卸料完成
u8  g_LC_UnLoad_Permit=FALSE;	    //允许卸料
u8  g_Read_LC_Load_Program=FALSE;
u8  g_Read_LC_UnLoad_Program=FALSE;
u8  g_LC_PresentLine=0;				//料仓当前行
u8  g_Start_Action_PresentLine=0;	//启动单次动作当前行
u8  g_Start_Step_Control=FALSE;		//标识是否是启动动作，与自动动作区分
u8  g_Reset_Action_PresentLine=0;	//复位动作当前行
u8  g_Reset_Step_Control=FALSE;		//标识是否是复位动作，与自动动作区分

u8  g_Auto_Order_Start = FALSE;		//自动运行启动指令
u8  g_Auto_Order_Pause = FALSE;		//自动运行暂停指令
u8  g_Auto_Order_Stop  = FALSE;		//自动运行停止指令
u8  g_Auto_Order_StackReset = FALSE;//码垛复位指令

/***************************自行运行内部参数**************************************************/
u32 g_Auto_ActionRun_Timer = 0;
u8  g_Auto_ActionRun_Step = 0;
u32	g_Auto_ActionTimeOut_Time = 1000; //10ms为单位		 20s没检测到确认信号，则认为超时
u8 	g_Auto_ActionTimeOut_Flag = FALSE;//动作检测超时
u8 	g_Auto_ActionError_Flag = FALSE;  //自动运行进入到错误状态AUTO_ERROR
u8 	g_Auto_WorkFinished_Flag = FALSE; //当前工作任务已完成
u8  g_Material_Load_Disk_Finished    = FALSE; //当前料盘加工完成
u8  g_Material_UnLoad_Disk_Finished  = FALSE; //当前料盘加工完成

ActionStepList Action_Step_List[LISTNUM]={0}; //运行指令
u8 	Action_Step_List_Num=0;					  //单次运行指令数
u8 	Action_Step_Run_Num=0;					  //已经运行的指令行数
u8 	Action_Step_Confirm_Num=0;				  //确认的指令行数
u8  g_Material_Disk_Mode = FALSE;	          //料盘模式
u8  g_Read_Load_LC_Position = FALSE;		  //计算料仓位置标志位
u8  g_Read_UnLoad_LC_Position = FALSE;		  //计算料仓位置标志位
u8  g_LC_ActionRun_Step = 0;				  //料仓动作步骤
u32 g_LC_ActionRun_Timer = 0;
u32	g_LC_ActionTimeOut_Time = 1000; //10ms为单位		 20s没检测到确认信号，则认为超时
//u8 	g_LC_ActionTimeOut_Flag = FALSE;//动作检测超时
u8  g_Start_ActionRun_Step = 0;				  //启动单次动作步骤
u8  g_Reset_ActionRun_Step = 0;				  //启动单次动作步骤

u16 g_Load_LC_Key=0;				  //是否有上料指令
u32 g_Load_LC_Present_Position_X=0;	  //保存料仓初始位置X
u32 g_Load_LC_Present_Position_L=0;	  //保存料仓初始位置L
u8  g_Load_LC_Position_XKey=0;		  //LC位置指令行数X
u8  g_Load_LC_Position_LKey=0;	      //LC位置指令行数L

u16 g_UnLoad_LC_Key=0;				  //是否有卸料指令
u32 g_UnLoad_LC_Present_Position_X=0; //保存料仓初始位置X
u32 g_UnLoad_LC_Present_Position_L=0; //保存料仓初始位置L
u8  g_UnLoad_LC_Position_XKey=0;	  //LC位置指令行数X
u8  g_UnLoad_LC_Position_LKey=0;	  //LC位置指令行数L

void ActionStepControl(void);
void ActionOverOperate(void);
void ErrorJudgeWhile(void);
//u8 ActionAllowJudge(SaveProgram,u8);
void AutoPauseOperate(void);
void AutoStopOperate(void);
//void AutoRunning2(void);
void StartActionControl(void);




//-------------------------------------------//
u32 HT_Height=0;//回抬高度
u32 LC_Height=0;//料仓高度
u32 Glue_Time=0;//出胶量
u32 Delay_Time=0;//取放料延时
u8 HT_Flag=0,LC_Empty_Flag=0;//Z轴是否需要回抬
u8 LC_Count_X1[10],LC_Count_Y1[10];
u8 LC_Pause_Correct=0;//暂停料仓点修正
u8 DJ_Over=0,DZ_Over=0;
u16 Confirm_Order_Num=0;//点胶还是点钻模式
u16 X_Speed =0,Z_Speed=0,Y_Speed=0,O_Speed=0,A_Speed=0,B_Speed=0;


extern u8 Order_Read_Mode;
extern u16 ShuZu_Count;
extern u8 LC_ID;
extern u16 Read_Point_Num;
extern u32 g_DianJiao_Delay_Timer;
extern u8  g_DianJiao_Delay_Step;
extern u8 Action_Flag;
extern u8  g_LCDelay_Step;
extern u32 g_LCDelay_Timer;

/**************************************************************************************************
**  函数名：  AutoModeControl()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式控制函数
**	备注：	  控制设备自动运行
**  作者：    周海波    
**  开发日期：2015/10/21
***************************************************************************************************/
void AutoModeControl(void)
{
	switch(g_AutoStatue)
	{
		case AUTO_WAITE:{//等待状态检测启动命令
			 if(g_Auto_Order_Start == TRUE)		//接收到启动自动运行指令
			 {
				g_Auto_Order_Start = FALSE;		//复位标志位
				g_AutoStatue = AUTO_RUNNING;	//进入自动运行状态
				g_Auto_Circle_Timer = 0;
			 }
//			if(g_Auto_Order_StackReset == TRUE)	//重新复位码垛参数指令
//			{
//				g_Auto_Order_StackReset = FALSE;//复位标志位，具体码垛参数带后续添加
//			}
		 }break; 
		case AUTO_RUNNING:{//自动运行相关操作
			 if(g_Auto_Order_Pause == TRUE)		//重新启动自动运行指令
			 {
				g_Auto_Order_Pause = FALSE;		//复位标志位
				g_AutoStatue = AUTO_PAUSE;		//回到自动运行状态
				AutoPauseOperate();				//暂停运行相关操作			
			 }
			 if(g_Auto_Order_Stop == TRUE)		//自动运行停止指令
			 {
				g_Auto_Order_Stop = FALSE;		//复位标志位
				g_AutoStatue = AUTO_WAITE;		//回到自动等待状态	
				g_Auto_PresentLine=0;
				Action_Step_Run_Num=0;
				AutoPauseOperate();				//暂停运行相关操作
				AutoStopOperate();				//停止运行相关操作
				Read_Point_Num=0;
				Order_Read_Mode=0;
				ShuZu_Count=0;				//运行到的指令数组位置计数
				Order_Package();	
			 }
			 if(g_AutoStatue == AUTO_RUNNING)
			 {
				 g_LC_Step_Control=FALSE;
				 g_Start_Step_Control=FALSE;
				 g_Reset_Step_Control=FALSE;
				 ActionStepControl();//单行动作步骤控制（对Free_Program_Operate1结构体进行控制）
				 //启动单次动作
				 if(g_Program_Start_Action==TRUE && g_Program_Start_Action_Done==FALSE)//启动单次动作
				 {
				   g_Start_Step_Control=TRUE;
				 }
				 //堆叠复位动作
				 if(g_Program_Reset_Action==TRUE && g_Program_Reset_Action_Done==FALSE)//堆叠复位动作
				 {
				     g_Reset_Step_Control=TRUE;
				 }
				 //料仓动作
				 if(g_LC_Program_Step_Run==TRUE)  //料仓程序
				 {
					 g_LC_Step_Control=TRUE;
				 }
			 }
		 }break;
		case AUTO_PAUSE:{ //暂停转状态检测是否启动或停止
			 if(g_Auto_Order_Start == TRUE)		//重新启动自动运行指令
			 {
					g_Auto_Order_Start = FALSE;		//复位标志位
					g_AutoStatue = AUTO_RUNNING;	//回到自动运行状态
			 }
			 if(g_Auto_Order_Stop == TRUE)		//自动运行停止指令
			 {
					g_Auto_Order_Stop = FALSE;		//复位标志位
					g_AutoStatue = AUTO_WAITE;		//回到自动等待状态
					AutoStopOperate();				//停止运行相关操作
			 }
		 }break;
		case AUTO_ERROR:{//错误检测处理
			 g_Auto_ActionError_Flag=TRUE;
			 g_Auto_Order_Stop = TRUE;
			 if(g_Auto_Order_Stop == TRUE)		//自动运行停止指令
			 {
					//g_Auto_ActionError_Flag=FALSE;
					g_Auto_Order_Stop = FALSE;		//复位标志位
					g_AutoStatue = AUTO_WAITE;		//回到自动等待状态
					AutoPauseOperate();
					AutoStopOperate();				//停止运行相关操作
			 }
		 }break;
		default:{
		}break;
	}		
}



/**************************************************************************************************
**  函数名：  Speed_Compensate()
**	输入参数：无
**	输出参数：无
**	函数功能：通过时间实现各轴同步，指令预处理
**	备注：	  
**  作者：    White_L    
**  开发日期：2018/7/1
***************************************************************************************************/
void Speed_Compensate(u16 AutoPresentLine)
{
	u32 Axis_Distance[6]={0},Max_distance=0;
	u8 i=0;
	float Time1_Dis=0.0;

	if(AutoPresentLine<(Free_Program_Operate1.Num))
	{
		LC_ID      = Free_Program_Operate1.Program[ShuZu_Count].LC_ID;
		HT_Height  = LC_Parameter_Load[LC_ID-1].HT_Hight;
		LC_Height  = (LC_Parameter_Load[LC_ID-1].LC_Hight&0x0fffffff)+MINROBOTPOSITION;
		Glue_Time  = LC_Parameter_Load[LC_ID-1].Glue_Time;
		Delay_Time = LC_Parameter_Load[LC_ID-1].Diamonds_Time;	
		/*****************料仓检测，任意料盘料空空移到固定位置提示换料********************/
		if(Free_Program_Operate1.Program[ShuZu_Count].Key==3&&LC_Pause_Correct==0)
		{
			Free_Program_Operate1.Program[ShuZu_Count].Value1 = (LC_Load_Table[LC_ID-1].LC_Load_Position[LC_Count_X1[LC_ID-1]][LC_Count_Y1[LC_ID-1]].XPoint& 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[ShuZu_Count].Value2 = (LC_Load_Table[LC_ID-1].LC_Load_Position[LC_Count_X1[LC_ID-1]][LC_Count_Y1[LC_ID-1]].YPoint& 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[ShuZu_Count].Value3 = LC_Height;
			Free_Program_Operate1.Program[ShuZu_Count].Value4 = MINROBOTPOSITION;	
			Free_Program_Operate1.Program[ShuZu_Count].Value5 = MINROBOTPOSITION;
			 if(LC_Count_Y1[LC_ID-1]<LC_Parameter_Load[LC_ID-1].V_Num-1)
				 LC_Count_Y1[LC_ID-1]++;
			 else if(LC_Count_Y1[LC_ID-1]>=(LC_Parameter_Load[LC_ID-1].V_Num-1)&&LC_Count_X1[LC_ID-1]<=LC_Parameter_Load[LC_ID-1].H_Num-1)
			 {
				 LC_Count_X1[LC_ID-1]++;
				 LC_Count_Y1[LC_ID-1]=0;
			 }
			 else if(LC_Count_X1[LC_ID-1]>=LC_Parameter_Load[LC_ID-1].H_Num-1)//料钻取完进入暂停状态
			 {
				 LC_Empty_Flag=1;
				 LC_Count_X1[LC_ID-1]=0;
				 LC_Count_Y1[LC_ID-1]=0;					 
			 }
		}
		if(Free_Program_Operate1.Program[ShuZu_Count].Value1>Pulse_TotalCounter[X_Axsis])
			Axis_Distance[X_Axsis]=Free_Program_Operate1.Program[ShuZu_Count].Value1-Pulse_TotalCounter[X_Axsis];
		else
			Axis_Distance[X_Axsis]=Pulse_TotalCounter[X_Axsis]-Free_Program_Operate1.Program[ShuZu_Count].Value1;
		
		if(Free_Program_Operate1.Program[ShuZu_Count].Value2>Pulse_TotalCounter[L_Axsis])
			Axis_Distance[L_Axsis]=Free_Program_Operate1.Program[ShuZu_Count].Value2-Pulse_TotalCounter[L_Axsis];
		else
			Axis_Distance[L_Axsis]=Pulse_TotalCounter[L_Axsis]-Free_Program_Operate1.Program[ShuZu_Count].Value2;
		///------------------------------只要Z轴有回抬空间且回抬后比目标位置高就回抬操作-------------------------------//
		if((Pulse_TotalCounter[Z_Axsis]-MINROBOTPOSITION>HT_Height)&&(Pulse_TotalCounter[Z_Axsis]-HT_Height)<Free_Program_Operate1.Program[ShuZu_Count].Value3)
		{
			HT_Flag=1;
			Axis_Distance[Z_Axsis] = Pulse_TotalCounter[Z_Axsis]-HT_Height;
			if(Axis_Distance[Z_Axsis]>Free_Program_Operate1.Program[ShuZu_Count].Value3)
				Axis_Distance[Z_Axsis] = Axis_Distance[Z_Axsis]-Free_Program_Operate1.Program[ShuZu_Count].Value3+HT_Height;
			else
				Axis_Distance[Z_Axsis]=Free_Program_Operate1.Program[ShuZu_Count].Value3-Pulse_TotalCounter[Z_Axsis]+HT_Height;//取料每个轴每次动作各自需要走的路程
		}
		else
		{
			HT_Flag=0;
			if(Free_Program_Operate1.Program[ShuZu_Count].Value3>Pulse_TotalCounter[Z_Axsis])
				Axis_Distance[Z_Axsis] = Free_Program_Operate1.Program[ShuZu_Count].Value3-Pulse_TotalCounter[Z_Axsis];
			else
				Axis_Distance[Z_Axsis]=Pulse_TotalCounter[Z_Axsis]-Free_Program_Operate1.Program[ShuZu_Count].Value3;
		}

//		if(Axis_Distance[X_Axsis]>=Axis_Distance[L_Axsis]&&Axis_Distance[X_Axsis]>=Axis_Distance[Z_Axsis])
//		{
//			X_Speed = JXS_Parameter.AutoSpeed_H[0]*100;
//			Time1_Dis = Axis_Distance[X_Axsis]*1.0/JXS_Parameter.AutoSpeed_H[0]*1.0;
////			Y_Speed = (Axis_Distance[L_Axsis]*1.0/Time1_Dis*1.0)*100;
//			Z_Speed = (Axis_Distance[Z_Axsis]*1.0/Time1_Dis*1.0)*100;
//		}
//		else if(Axis_Distance[L_Axsis]>=Axis_Distance[X_Axsis]&&Axis_Distance[L_Axsis]>=Axis_Distance[Z_Axsis])
//		{
//			Y_Speed = JXS_Parameter.AutoSpeed_H[0]*100;
//			Time1_Dis = Axis_Distance[L_Axsis]*1.0/JXS_Parameter.AutoSpeed_H[0]*1.0;
//			X_Speed = (Axis_Distance[X_Axsis]*1.0/Time1_Dis*1.0)*100;
//			Z_Speed = (Axis_Distance[Z_Axsis]*1.0/Time1_Dis*1.0)*100;		
//		}
//		else if(Axis_Distance[Z_Axsis]>=Axis_Distance[X_Axsis]&&Axis_Distance[Z_Axsis]>=Axis_Distance[L_Axsis])
//		{
//			Z_Speed = JXS_Parameter.AutoSpeed_H[0]*100;
//			Time1_Dis =  Axis_Distance[Z_Axsis]*1.0/JXS_Parameter.AutoSpeed_H[0]*1.0;
//			X_Speed =  (Axis_Distance[X_Axsis]*1.0/Time1_Dis*1.0)*100;
////			Y_Speed =  (Axis_Distance[L_Axsis]*1.0/Time1_Dis*1.0)*100;
//		}
		Z_Speed = JXS_Parameter.AutoSpeed_H[0]*100;
		X_Speed = JXS_Parameter.AutoSpeed_H[0]*100;
		Y_Speed = JXS_Parameter.AutoSpeed_H[0];
		A_Speed = JXS_Parameter.AutoSpeed_H[1]*100;
		B_Speed = JXS_Parameter.AutoSpeed_H[2]*100;
   }
}


/**************************************************************************************************
**  函数名：  ActionStepControl()
**	输入参数：无
**	输出参数：无
**	函数功能：单行动作步骤控制函数
**	备注：	  控制每行程序单独动作
**  作者：    周海波    
**  开发日期：2015/10/21
***************************************************************************************************/
void ActionStepControl(void)
{
	u8 temp1[10];//当前需要运行行号
	u8 ActionSetResult = 0;//动作执行状态寄存变量
	u16 AutoPresentLine=0;
	u8 i=0;
	switch(g_Auto_ActionRun_Step)//根据Step确定对应的执行环节
	{
		case 0:{		//动作输出环节
			//执行每个动作，函数将根据动作执行情况返回，正常执行则进入动作确认环节，
			//执行函数需要将g_Auto_ActionTimeOut_Time参数设置对应值，不可为0，用于动作执行超时判断
			//统计当前一次执行指令行
			if(Auto_Mode == SINGLE_MODE)
			{
			    Action_Step_List_Num=1;//单次运行指令=1
			}
			else if(Action_Step_List_Num==0)// && Free_Program_Operate1.Program[0].Key==K_PROGRAMSTART)//非单步模式且没有关联行数信息的时候
			{
//				Action_Step_List_Num=AutoActionStepList(g_Auto_PresentLine);	  //获取关联行的行数
				Action_Step_List_Num=1;
				Action_Step_Run_Num=0;											  //已经运行行数置0
			}
			if(g_Auto_PresentLine==Free_Program_Operate1.Num)
				g_Auto_PresentLine=0;
			AutoPresentLine = g_Auto_PresentLine+Action_Step_Run_Num;						//当前需要运行行号=自动运行的行号+已运行行数
			Speed_Compensate(AutoPresentLine);
			ActionSetResult = AutoActionOutControl(AutoPresentLine);	//获取执行情况
			switch(ActionSetResult)//针对不同情况进行处理
			{				
				case 0:		//动作正常被执行
					 g_Auto_ActionRun_Step = 1;
					 Action_Step_List_Num=0;
					 Action_Step_Confirm_Num=0;
					 break;
				case 9:		//动作循环结束	  
					 ActionOverOperate();	 //g_Auto_ActionRun_Step = 3;
					 break;
				case 10:	//当前行号长度异常
					 g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报c警标志位
					 break;
				case 11:	//主要指令类型异常
					 g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					 break;
				case 12:	//基本指令类型异常
					 g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					 break;
				case 13:	//轴控指令类型异常
					 g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					 break;
				case 14:	//IO控制指令类型异常
					 g_AutoStatue = AUTO_ERROR;
					//插入对应异常报警处理函数与报警标志位
					 break;
				case 15:
				     break;
				default:
					 break;
			}
		}break;
		case 1:{		//动作确认环节			
			if(AutoActionOutConfirm(Free_Program_Operate1,ShuZu_Count) == TRUE)//判断动作确认情况
			{
				if(LC_Empty_Flag == 0)
				{
					Confirm_Order_Num++;//记录运动指令数目				
					Action_Step_Confirm_Num++;
					if(Action_Step_Confirm_Num==Action_Step_Run_Num)
					{
						g_Auto_ActionRun_Step = 2;
						g_Auto_ActionRun_Timer=0;
						Action_Step_Confirm_Num=0;
					}
				}
			}
			else//动作确认不成功
			{
				if(g_Auto_ActionRun_Timer > g_Auto_ActionTimeOut_Time)	//动作执行超时判断
				{
					if(Free_Program_Operate.Program[g_Auto_PresentLine+Action_Step_Confirm_Num].Order<=OR_AXISORDER)
					{
					}
					else
					{
						g_Auto_ActionTimeOut_Flag=TRUE;
						g_Auto_Order_Pause = TRUE;
						if(g_Program_Is_Debuging)
						{
							g_Auto_Order_Stop = TRUE;
						}
					}
					g_Auto_ActionRun_Timer=0;
				}
			}
		}break;
		case 2:{		//动作延时环节
			 if(TRUE)//AutoActionOutDelay(Free_Program_Operate1,g_Auto_PresentLine+(Action_Step_Run_Num-1)) == TRUE
			 {
					Order_Package();					
					
					if(Order_Read_Mode==0)
					{
						ShuZu_Count++;	
						if(ShuZu_Count==Free_Program_Operate1.Num)
							ShuZu_Count=0;
					}
				 //点胶指令
					if(Free_Program_Operate1.Program[ShuZu_Count].Key==1)
					{
						 for(i=13;i<21;i++)		//指示灯不复位
						 {
							SetOutput(i);
						 }	
					}
				 //点钻指令
					if(Free_Program_Operate1.Program[ShuZu_Count].Key==2||Free_Program_Operate1.Program[ShuZu_Count].Key==3)
					{
						 for(i=4;i<7;i++)		//指示灯不复位
						 {
							SetOutput(i);
						 }	
					}

					
					if(LC_Empty_Flag == 0)
					{
						if(Confirm_Order_Num==Free_Program_Operate1.Num) 
						{
							ActionOverOperate();//一套指令动作结束
						}
						else
							g_Auto_ActionRun_Step = 0;
						Action_Step_List_Num=0;
						g_Auto_PresentLine=g_Auto_PresentLine+Action_Step_Run_Num;
						if(Auto_Mode == SINGLE_MODE) //单步模式-进入暂停状态
						{
							g_Auto_Order_Pause = TRUE;	
						}
						if(g_Program_Is_Debuging)
						{
							g_Auto_Order_Stop = TRUE;
						}
					}
					else
					{
						g_Auto_Order_Pause = TRUE;
						temp1[0] =0xCC;
						USART1_SendData(1,0xCC,&temp1[0]);//暂停机器换料
						LC_Empty_Flag=0;
					}
		  }
		 }break;
		case 3:{		
			/***一套动作结束****/
			 Confirm_Order_Num=0;
//			 Order_Package();		
			 g_Auto_ActionRun_Step=0;
			 Action_Step_List_Num=0;
			LC_Pause_Correct=0;
			 if(Auto_Mode == SINGLE_MODE||(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)) //单步模式-进入暂停状态
			 {
				g_Auto_Order_Pause = TRUE;	
				 for(i=0;i<35;i++)		//指示灯不复位
				 {
					SetOutput(i);
				 }	
				DJ_Over=0;
				DZ_Over=0;
			 }
			 if(g_Program_Is_Debuging)
			 {
				g_Auto_Order_Stop = TRUE;
				//g_Program_Is_Debuging=FALSE;
			 }
			 if(SC_Parameter.SC_Num==SC_Parameter.RW_Num)
			 {
			 	g_Auto_Order_Pause = TRUE;
				g_Auto_WorkFinished_Flag=TRUE;
			 }
		 }break;
		default:
			break;
	}
}


/**************************************************************************************************
**  函数名：  ActionOverLCCalculate()
**	输入参数：
**	输出参数：
**	函数功能：
**	备注：	  
**  作者：    wx   
**  开发日期：2015/10/21
***************************************************************************************************/
void ActionOverLCCalculate(void)
{
//	if(g_Load_LC_Key)
//	{
//		if(SC_Parameter.SC_Num%(LC_Parameter_Load.Layer*LC_Parameter_Load.H_Num*LC_Parameter_Load.V_Num)==0)	 //一个料盘加工完成 进入报警（暂停）状态
//		{
//			g_Material_Load_Disk_Finished=TRUE;
////			Free_Program_Operate.Program[g_Load_LC_Position_XKey].Value1=g_Load_LC_Present_Position_X;
////			Free_Program_Operate.Program[g_Load_LC_Position_LKey].Value1=g_Load_LC_Present_Position_L;
//			//g_Read_Load_LC_Position=FALSE;
//		}
//		else
//		{
//			g_Material_Load_Disk_Finished=FALSE;
//		}	
//	}
//	if(g_UnLoad_LC_Key)
//	{
//		if(SC_Parameter.SC_Num%(LC_Parameter_UnLoad.Layer*LC_Parameter_UnLoad.H_Num*LC_Parameter_UnLoad.V_Num)==0)	 //一个料盘加工完成 进入报警（暂停）状态
//		{
//			g_Material_UnLoad_Disk_Finished=TRUE;
////			Free_Program_Operate.Program[g_UnLoad_LC_Position_XKey].Value1=g_UnLoad_LC_Present_Position_X;
////			Free_Program_Operate.Program[g_UnLoad_LC_Position_LKey].Value1=g_UnLoad_LC_Present_Position_L;
//			//g_Read_UnLoad_LC_Position=FALSE;
//		}
//		else
//		{
//			g_Material_UnLoad_Disk_Finished=FALSE;
//		}	
//	}
}

/**************************************************************************************************
**  函数名：  ActionOverLCCalculate()
**	输入参数：无
**	输出参数：无
**	函数功能：循环结束操作处理函数
**	备注：	  一个周期结束后，数据恢复处理
**  作者：    周海波    
**  开发日期：2015/10/21
***************************************************************************************************/
void ActionOverOperate(void)
{
	u8 Temp_data[8]={0};
	g_Auto_PresentLine = 0;
	Action_Step_List_Num=0;
	Action_Step_Run_Num=0;
	Action_Step_Confirm_Num=0;
	g_Auto_LastCircleTime = g_Auto_Circle_Timer;
	g_Auto_Circle_Timer=0;
//	g_Auto_ProduceCounter ++;
	//写IIC,累计产量以及生产产量
	if((g_Program_Is_Debuging==FALSE) && (g_Run_Program_Num<=8))
	{
		SC_Parameter.SC_Num++;
		SC_Parameter.LJ_Num++;	
		if(SC_Parameter.LJ_Num>MINROBOTPOSITION)
		{
			SC_Parameter.LJ_Num=0;
		}
		Temp_data[0] = SC_Parameter.SC_Num;
		Temp_data[1] = SC_Parameter.SC_Num>>8;
		Temp_data[2] = SC_Parameter.SC_Num>>16;
		Temp_data[3] = SC_Parameter.SC_Num>>24;
		Temp_data[4] = SC_Parameter.LJ_Num;
		Temp_data[5] = SC_Parameter.LJ_Num>>8;
		Temp_data[6] = SC_Parameter.LJ_Num>>16;
		Temp_data[7] = SC_Parameter.LJ_Num>>24;
		W25QXX_Write(Temp_data,0x1090+0x0C,8);
		//进入结束
		//料仓参数计算处理
		//ActionOverLCCalculate();
	}
	g_Auto_ActionRun_Step = 3;
}

/**************************************************************************************************
**  函数名：  ErrorJudgeWhile()
**	输入参数：无
**	输出参数：无
**	函数功能：全程异常检查函数
**	备注：	  在程序运行过程中，实时监测异常情况，
**            错误发身后，立刻报警
**  作者：    周海波    
**  开发日期：2015/10/21
***************************************************************************************************/
void ErrorJudgeWhile(void)
{
} 

/**************************************************************************************************
**  函数名：  AutoPauseOperate()
**	输入参数：无
**	输出参数：无
**	函数功能：自动暂停操作
**	备注：	  暂停正在运行的全自动动作，设置相关标志位
**  作者：    周海波    
**  开发日期：2015/10/21
***************************************************************************************************/
void AutoPauseOperate(void)
{	
//    switch(g_Auto_ActionRun_Step)
//	{

//		case 0:		//动作输出环节
//			 Action_Step_List_Num=0;		 
//			 break;
//		case 1:		//动作确认环节	
			 if(Timer_Enable_X==ENABLE)		//如果正在发送脉冲
			 {
				ServoStop_CAN(X_Axsis);
				  CAN_Servo_Stop_Action[X_Axsis]=FALSE;
			 }					 
			 if(Timer_Enable_Z==ENABLE)		//如果正在发送脉冲
			 {
			  	ServoStop_CAN(Z_Axsis);
				  CAN_Servo_Stop_Action[Z_Axsis]=FALSE;
			 }
			 if(Timer_Enable_L==ENABLE)		//如果正在发送脉冲
			 {
			  	ServoStop_CAN(L_Axsis);
				  CAN_Servo_Stop_Action[L_Axsis]=FALSE;
			 }
			 if(Timer_Enable_O==ENABLE)		//如果正在发送脉冲
			 {
			  	ServoStop_CAN(O_Axsis);
				  CAN_Servo_Stop_Action[O_Axsis]=FALSE;
			 }	 
			 if(Timer_Enable_A==ENABLE)		//如果正在发送脉冲
			 {
			  	ServoStop_CAN(A_Axsis);
				  CAN_Servo_Stop_Action[A_Axsis]=FALSE;
			 }
			 if(Timer_Enable_B==ENABLE)		//如果正在发送脉冲
			 {
			  	ServoStop_CAN(B_Axsis);
				  CAN_Servo_Stop_Action[B_Axsis]=FALSE;
			 }				 
			 Action_Step_List_Num=0;
			 Action_Step_Run_Num=0;
			 Action_Step_Confirm_Num=0;
			 g_Auto_ActionRun_Step = 0;
			 g_LC_ActionRun_Step = 0;
			 g_Reset_ActionRun_Step = 0;
			 
			 
			 Action_Flag=0;
			 g_LCDelay_Step=0;
			 g_LCDelay_Timer=0;
			 g_DianJiao_Delay_Step=0;
			 g_DianJiao_Delay_Timer=0;		 
//			 break;
//
//		case 2:		//动作延时环节
////			 g_Auto_ActionRun_Step = 0;
////			 g_Auto_PresentLine ++;
//			 break;
//		default:
//			break;
//	}
}

/**************************************************************************************************
**  函数名：  AutoStopOperate()
**	输入参数：无
**	输出参数：无
**	函数功能：动作执行前，合法性判断
**	备注：	  停止全自动运行动作，并根据需求复位标志位
**  作者：    周海波    
**  开发日期：2015/10/21
***************************************************************************************************/
void AutoStopOperate(void)
{
	if(g_Program_Is_Debuging)
	{
	  g_Program_Is_Debuging=FALSE;
		g_AutoStatue = AUTO_WAITE;
	}
	else
	{
		g_AutoStatue = AUTO_WAITE;
		g_Auto_Order_Start = FALSE;
		g_Auto_Order_Pause = FALSE;
		g_Auto_Order_Stop = FALSE;
		g_Auto_PresentLine = 0;
		g_Auto_LastCircleTime = 0;
		g_Auto_Circle_Timer = 0;
		Single_Mode_Enable = DISABLE;
		Loop_Mode_Enable = DISABLE;
		g_Auto_ActionTimeOut_Flag=FALSE;
		Puls_Delay_Num=0;
		Action_Step_List_Num=0;
		Action_Step_Run_Num=0;
		Action_Step_Confirm_Num=0;
		g_Read_LC_Load_Program=FALSE;
		g_Read_LC_UnLoad_Program=FALSE;
		g_LC_PresentLine = 0;
		g_LC_Step_Control=FALSE;
		g_LC_Program_Step_Run=FALSE;
		g_LC_Load_Permit=FALSE;
		g_LC_Load_Finish=FALSE;
		g_LC_UnLoad_Permit=FALSE;
		g_LC_UnLoad_Finish=FALSE;
		g_Program_Start_Action=FALSE;
		g_Start_Step_Control=FALSE;
		g_Program_Reset_Action=FALSE;
		g_Reset_Step_Control=FALSE;
	}
}

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/

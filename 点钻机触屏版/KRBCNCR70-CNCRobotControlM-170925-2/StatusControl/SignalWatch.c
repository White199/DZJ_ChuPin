/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
 
#include "SignalWatch.h"
#include "StatusControl.h"
#include "Manual.h"
#include "Auto.h"
#include "Auto_2.h"
#include "BackToOrigin.h"

#include "in.h"
#include "out.h"
#include "Error.h"
#include "Usart.h"
#include "Parameter.h"                  
#include "SpeedControl.h"

/*- 实时数据，每一位保存一种类型数据，1表示有效，0表示无效 -*/
/*      11            1      1         1           11      1    */
/*  机械手启动   机床完成  异常  料仓上料完成 机床安全门  转盘  */
/*  启动 ：00-停止   01-暂停   10-运行                         */
/*  安全门 ：00-关闭   01-关闭中   10-开启中  11-开启          */
u8 Watch_Data = 0;

//static u8 Send_Temp[6]={0x01,0x02,0x03,0x04,0x05,0x06};	//控制器回复手持器数据
//static u8 Data_Temp[4]={0};								//控制器回复手持器数据32位
static u8 Watch_Usart_Data[10]={0};
extern vu32 Pre_Real_Time_Position_1[6];                  //--2017.8实时目标位置
/**************************************************************************************************
**  函数名：  WatchCommand()
**	输入参数：无
**	输出参数：无
**	函数功能：监视命令
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/						 
void WatchCommand()
{
	 u8 i = 0;
	 for(i=0;i<12;i++)
	 {
	 	Watch_Usart_Data[i]=0;
	 }
	 switch(UsartReceiveData[1])
	 { 
	 	  case IO_DEBUG_LOCAL:								  //X37-X42	6位
//		  	 Watch_Usart_Data[0] =Input_Detect_Status[4];
//			   Watch_Usart_Data[1] =Input_Detect_Status[5];       
//			   USART1_SendData(2,0xE1,&Watch_Usart_Data[0]);									 //[0-2]
			   break;

	 	  case IO_DEBUG_INPUT1:								   //X0-X14 15位   
//		  	 Watch_Usart_Data[0] = Input_Detect_Status[0] ;  //
//			   Watch_Usart_Data[1] = Input_Detect_Status[1] ;  //
//			   Watch_Usart_Data[2] = Input_Detect_Status[2] ;  //
//			   Watch_Usart_Data[3] = Input_Detect_Status[3] ;  //
//			   Watch_Usart_Data[4] = Input_Detect_Status[4] ;  //
//			   USART1_SendData(5,0xE2,&Watch_Usart_Data[0]);
			   break;

	 	  case IO_DEBUG_INPUT2:								   //X12-X24 13位
//		  	 Watch_Usart_Data[0] = (Input_Detect_Status[2]<<4)|(Input_Detect_Status[1]>>4) ;  //1-[4-7]
//			   Watch_Usart_Data[1] = (Input_Detect_Status[3]<<4)|(Input_Detect_Status[2]>>4) ;  //2-[0-7]
//			   //Watch_Usart_Data[1] = Input_Detect_Status[3]	;                               //3-[0]
//			   USART1_SendData(2,0xE3,&Watch_Usart_Data[0]);
			   break;

	 	  case IO_DEBUG_INPUT3:								   //X25-X36 12位
//		  	 Watch_Usart_Data[0] = (Input_Detect_Status[4]<<7)|(Input_Detect_Status[3]>>1);  //3-[1-7]
//			   Watch_Usart_Data[1] = Input_Detect_Status[4]>>1 ;                               //4-[0-4]
//			   USART1_SendData(2,0xE4,&Watch_Usart_Data[0]);
			   break;

	 	  case IO_DEBUG_OUTPUT1_LCD:					 //X0-X11
//		  	 Watch_Usart_Data[0] = Output_Status[0] ;  //[0-7]
//			   Watch_Usart_Data[1] = Output_Status[1];   //[0-3]
//			   USART1_SendData(2,0xE5,&Watch_Usart_Data[0]);
			   Watch_Usart_Data[0] = Output_Status[0] ;
			   Watch_Usart_Data[1] = Output_Status[1] ;
			   Watch_Usart_Data[2] = Output_Status[2] ;
			   Watch_Usart_Data[3] = Output_Status[3] ;
			   Watch_Usart_Data[4] = Output_Status[4] ;
			   USART1_SendData(5,0xE5,&Watch_Usart_Data[0]);
			   break;

	 	  case IO_DEBUG_OUTPUT2_LCD:					 //X12-X23
//		  	 Watch_Usart_Data[0] = Output_Status[1] ;  //[4-7]
//			   Watch_Usart_Data[1] = Output_Status[2] ;  //[0-7]
//			   USART1_SendData(2,0xE6,&Watch_Usart_Data[0]);
			   break;

	 	  case IO_DEBUG_OUTPUT3_LCD:					 //X24-X36
//		  	 Watch_Usart_Data[0] = Output_Status[3] ;  //[0-7]
//			   Watch_Usart_Data[1] = Output_Status[4] ;  //[0-4]
//			   USART1_SendData(2,0xE7,&Watch_Usart_Data[0]);
			   break;

	 	  case ROBOT_ORIGINED:
		  	 Watch_Usart_Data[0] = Origin_Backed;
			   USART1_SendData(1,0xE8,&Watch_Usart_Data[0]);
			   break;

	 	  case ACTION_RESET_SCAN:	  //返回复位状态值
		  	 Watch_Usart_Data[0] = Robot_Auto_Reset;
			   USART1_SendData(1,0xE9,&Watch_Usart_Data[0]);
			   break;

	 	  case ALARM_CLEAR:
//			   if(Work_Status == MANUAL_WORK_MODE)
//			   {
//					Cancle_Genaral_Warning=TRUE;
//					Cancle_Genaral_Warning_Num=UsartReceiveData[1];
//			   }
			   Cancle_Genaral_Warning=TRUE;			   
			   USART1_SendData(1,0xEA,&Watch_Usart_Data[0]);
			   break;

	 	  case ROBOT_STATUS:
			   Watch_Usart_Data[0] = Work_Status;		   //机械手状态
			   Watch_Usart_Data[1] = g_AutoStatue;		   //机械手运行状态
			   Watch_Usart_Data[2] = Timer_Enable_X|Timer_Enable_Z|Timer_Enable_L|Timer_Enable_O|Timer_Enable_A|Timer_Enable_B;		   //是否在发脉冲
			   Watch_Usart_Data[3] = Robot_Error_Data[0];  //报警信息
			   Watch_Usart_Data[4] = Robot_Error_Data[1]; 
			   Watch_Usart_Data[5] = Robot_Error_Data[2]; 
			   Watch_Usart_Data[6] = Robot_Error_Data[3];
			   Watch_Usart_Data[7] = Pulse_ControlStep[X_Axsis];//当前轴所处的状态：加速 匀速 减速
			   Watch_Usart_Data[8] = Pulse_ControlStep[Z_Axsis]; 
			   Watch_Usart_Data[9] = Pulse_ControlStep[L_Axsis]; 
			   Watch_Usart_Data[10] = Pulse_ControlStep[O_Axsis]; 
			   Watch_Usart_Data[11] = Pulse_ControlStep[A_Axsis]; 
			   Watch_Usart_Data[12] = Pulse_ControlStep[B_Axsis]; 			
			   USART1_SendData(13,0xEB,&Watch_Usart_Data[0]);
			   break;

	 	  case AUTO_PARAMETER:
			   Watch_Usart_Data[0] = g_Auto_PresentLine;
			   Watch_Usart_Data[1] = g_Auto_PresentLine>>8;
			   Watch_Usart_Data[2] = SC_Parameter.SC_Num;
			   Watch_Usart_Data[3] = SC_Parameter.SC_Num>>8;
			   Watch_Usart_Data[4] = SC_Parameter.SC_Num>>16;
			   Watch_Usart_Data[5] = SC_Parameter.SC_Num>>24;  
			   Watch_Usart_Data[6] = SC_Parameter.LJ_Num;
			   Watch_Usart_Data[7] = SC_Parameter.LJ_Num>>8;
			   Watch_Usart_Data[8] = SC_Parameter.LJ_Num>>16;
			   Watch_Usart_Data[9] = SC_Parameter.LJ_Num>>24; 
			   USART1_SendData(10,0xEC,&Watch_Usart_Data[0]);
			   break;

	 	  case X_AXSIS_POSITION:
			  if(Real_Time_Position_1[X_Axsis]!=Pre_Real_Time_Position_1[X_Axsis])
			  {
				 Pre_Real_Time_Position_1[X_Axsis] =  Real_Time_Position_1[X_Axsis];
				 Watch_Usart_Data[5] = 0x01;		   //是否在发脉冲			
			  }
			  else
				 Watch_Usart_Data[5] = 0;		   //是否在发脉冲			
			  
			   Watch_Usart_Data[0] =  Pulse_TotalCounter[0];
			   Watch_Usart_Data[1] = (Pulse_TotalCounter[0])>>8;
			   Watch_Usart_Data[2] = (Pulse_TotalCounter[0])>>16;
			   Watch_Usart_Data[3] = (Pulse_TotalCounter[0])>>24;
			   Watch_Usart_Data[4] = Axsis_Move_Direction[X_Axsis];//当前运动方向			
			   USART1_SendData(6,0xED,&Watch_Usart_Data[0]);
			   break;

	 	  case Z_AXSIS_POSITION:
			  if(Real_Time_Position_1[Z_Axsis]!=Pre_Real_Time_Position_1[Z_Axsis])
			  {
				 Pre_Real_Time_Position_1[Z_Axsis] =  Real_Time_Position_1[Z_Axsis];
				 Watch_Usart_Data[5] = 0x02;		   //是否在发脉冲			
			  }
			  else
				 Watch_Usart_Data[5] = 0;		   //是否在发脉冲
			   Watch_Usart_Data[0] =  Pulse_TotalCounter[1];
			   Watch_Usart_Data[1] = (Pulse_TotalCounter[1])>>8;
			   Watch_Usart_Data[2] = (Pulse_TotalCounter[1])>>16;
			   Watch_Usart_Data[3] = (Pulse_TotalCounter[1])>>24;
			   Watch_Usart_Data[4] = Axsis_Move_Direction[Z_Axsis];//当前运动方向					
			   USART1_SendData(6,0xEE,&Watch_Usart_Data[0]);
			   break;

	 	  case ROBOT_PRE_STATUS:
		  	 Watch_Usart_Data[0] = Work_Status;
			   Watch_Usart_Data[1] = Timer_Enable_X|Timer_Enable_Z|Timer_Enable_L|Timer_Enable_O|Timer_Enable_A|Timer_Enable_B;
			   USART1_SendData(2,0xEF,&Watch_Usart_Data[0]);			   
			   break;

	 	  case ROBOT_DEBUG_STATUS:
		  	   Watch_Usart_Data[0] = g_Auto_PresentLine;
		  	   Watch_Usart_Data[1] = g_Auto_PresentLine>>8;
				USART1_SendData(2,0xF0,&Watch_Usart_Data[0]);			   
			   break;

	 	  case L_AXSIS_POSITION:
			  if(Real_Time_Position_1[L_Axsis]!=Pre_Real_Time_Position_1[L_Axsis])
			  {
				 Pre_Real_Time_Position_1[L_Axsis] =  Real_Time_Position_1[L_Axsis];
				 Watch_Usart_Data[5] = 0x04;		   //是否在发脉冲			
			  }
			  else
				 Watch_Usart_Data[5] = 0;		   //是否在发脉冲
			   Watch_Usart_Data[0] =  Pulse_TotalCounter[2];
			   Watch_Usart_Data[1] = (Pulse_TotalCounter[2])>>8;
			   Watch_Usart_Data[2] = (Pulse_TotalCounter[2])>>16;
			   Watch_Usart_Data[3] = (Pulse_TotalCounter[2])>>24;
			   Watch_Usart_Data[4] = Axsis_Move_Direction[L_Axsis];//当前运动方向				 
			   USART1_SendData(6,0xF1,&Watch_Usart_Data[0]);
			   break;

	 	  case O_AXSIS_POSITION:
			  if(Real_Time_Position_1[O_Axsis]!=Pre_Real_Time_Position_1[O_Axsis])
			  {
				 Pre_Real_Time_Position_1[O_Axsis] =  Real_Time_Position_1[O_Axsis];
				 Watch_Usart_Data[5] = 0x08;		   //是否在发脉冲			
			  }
			  else
				 Watch_Usart_Data[5] = 0;		   //是否在发脉冲
			   Watch_Usart_Data[0] =  Pulse_TotalCounter[3];
			   Watch_Usart_Data[1] = (Pulse_TotalCounter[3])>>8;
			   Watch_Usart_Data[2] = (Pulse_TotalCounter[3])>>16;
			   Watch_Usart_Data[3] = (Pulse_TotalCounter[3])>>24;
			   Watch_Usart_Data[4] = Axsis_Move_Direction[O_Axsis];//当前运动方向				 
			   USART1_SendData(6,0xF2,&Watch_Usart_Data[0]);
			   break;
	 	  case A_AXSIS_POSITION:
		     Watch_Usart_Data[0] =  Pulse_TotalCounter[4];
			   Watch_Usart_Data[1] = (Pulse_TotalCounter[4])>>8;
			   Watch_Usart_Data[2] = (Pulse_TotalCounter[4])>>16;
			   Watch_Usart_Data[3] = (Pulse_TotalCounter[4])>>24;
			   Watch_Usart_Data[4] = Axsis_Move_Direction[A_Axsis];//当前运动方向				 
			   Watch_Usart_Data[5] = Timer_Enable_X|Timer_Enable_Z<<1|Timer_Enable_L<<2|Timer_Enable_O<<3|Timer_Enable_A<<4|Timer_Enable_B<<5;		   //是否在发脉冲			
			   USART1_SendData(5,0xF3,&Watch_Usart_Data[0]);
			   break;
	 	  case B_AXSIS_POSITION:
		     Watch_Usart_Data[0] =  Pulse_TotalCounter[5];
			   Watch_Usart_Data[1] = (Pulse_TotalCounter[5])>>8;
			   Watch_Usart_Data[2] = (Pulse_TotalCounter[5])>>16;
			   Watch_Usart_Data[3] = (Pulse_TotalCounter[5])>>24;
			   Watch_Usart_Data[4] = Axsis_Move_Direction[B_Axsis];//当前运动方向				 
			   Watch_Usart_Data[5] = Timer_Enable_X|Timer_Enable_Z<<1|Timer_Enable_L<<2|Timer_Enable_O<<3|Timer_Enable_A<<4|Timer_Enable_B<<5;		   //是否在发脉冲			
			   USART1_SendData(6,0xF4,&Watch_Usart_Data[0]);
			   break;
	 	  default:
		     break; 
	 }
}

/**************************************************************************************************
**  函数名：  RealTimeWatch()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式下的各种模式选择
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void RealTimeWatch()
{
	 switch(UsartReceiveData[1])
	 {
	 	 case 0x01:				  //初始化是否完成
		    if(Initialize_Finished == TRUE)
			  {
			     UsartAcknowledge(INITIALIZE_FINISH);
			  }
		    break;
		 case 0x02:				  //开始是否已经回原点
		  	UsartAcknowledge(ORIGIN_BACK);			  
			  break; 
      }
}


/**************************************************************************************************
**  函数名：  WatchFunction()
**	输入参数：无
**	输出参数：无
**	函数功能：解码串口接收数据
**	备注：	  无
**  作者：         
**  开发日期： 
***************************************************************************************************/
void OrderDecoding()
{ 
	switch(UsartReceiveData[0])
	{
	  case P_ROBOT_ENABLE_A_ORIGIN:
		   RobotEnableOrigin();
		   break; 

	  case P_WORK_MODE:
	     WorkMode();	
		   break;            

	  case P_AUTO_RUN:
	     AutoRun();	
		   break;

	  case P_FREE_PROGRAM_SEND:
	     FreeProgramSend();	
		   break; 

	  case P_WATCH_COMMAND:	
		   WatchCommand();
		   break; 		   

	  case P_READ_IIC:
	     ReadIICData();	
		   break; 

	  case P_IO_DEBUG_OUTPUT1:
	     IODebugOutput1();
		   break; 		   		   		   		   		   		   		    		   		   		   	

	  case P_IO_DEBUG_OUTPUT2:
	     IODebugOutput2();
		   break; 

	  case P_IO_DEBUG_OUTPUT3:
	     IODebugOutput3();
		   break; 

	  case P_MANUL_DEBUG:
	     ManulDebug();
	     break;	

	  case P_PARAMETER_ORDER:
	     ParameterOrder();
	     break;	

	  default: 
	     break;					 						 	  
	}
		UsartAcknowledge(UsartReceiveData[0]);
}

/**************************************************************************************************
**  函数名：  WatchFunction()
**	输入参数：无
**	输出参数：无
**	函数功能：实现监视功能，并实时要求手控器变换显示的内容
**	备注：	  无
**  作者：         
**  开发日期： 
***************************************************************************************************/
void WatchFunction(void)
{
	u8 i=0;
	if( USART1ErrorFlag	== TRUE  )
	{
		 ReceiveDataCounter = 0;
		 NewOrder = FALSE;
	 	 for(i=0;i<60;i++)
		 {
		     ReceiveDataBuffer[i]=0;
		 }
	}
	else
	{
		 if(NewOrder == TRUE )
		 {
		    g_USART_Delay_Timer=0;
			  UsartDataDecode();			//串口接收数据处理
			  OrderDecoding();			//命令解析
				NewOrder=FALSE;
			  for(i=0;i<60;i++)			//清零处理后数据
			  {
			 	  UsartReceiveData[0]=0;
			  }		    
		 }
	}
}









/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

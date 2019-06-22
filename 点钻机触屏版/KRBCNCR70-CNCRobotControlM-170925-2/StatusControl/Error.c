/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Usart.h" 
#include "Error.h"
#include "in.h" 
#include "out.h"
#include "w25qxx.h" 
#include "Manual.h"
#include "Auto.h"
#include "Auto_2.h"
#include "SpeedControl.h"
#include "signalWatch.h"
#include "StatusControl.h"
#include "Parameter.h"

#include "PDO.h"
#include "CANopen.h"

/*----- 全局变量，错误发生标记位 -----*/
u8 Error_Enable_Flag = FALSE;       
u8 Error_Status = NO_ERROR;
u8 Machine_BreakDown_Flag=FALSE;	 //机器故障标志位
u8 EmergencyStop_Flag=FALSE;		 //急停标志位
u8 Full_Material_Flag=FALSE;		 //满料标志位
u8 Lack_Material_Flag=FALSE;		 //缺料标志位
u8 XAxsis_Error_Flag=FALSE;
u8 ZAxsis_Error_FLAG_FASLE;
u8 Limit_Error_Flag[4]={FALSE};		 //限位报警标志，用于操作动作
u8 Cancle_Limit_Warning=FALSE;		 //取消限位报警页面显示
u8 Cancle_Genaral_Warning=FALSE;   //取消当前报警编号
u8 Cancle_Genaral_Warning_Num=0;
u8 Limit_Error=FALSE;
/*old-JR105*/
/*- 报警数据，每一位保存一种类型报警，1表示报警，0表示无报警                -*/
/*   bit 15~8 																 */
/*   1     1        1      1        1        1           1            1      */
/* 急停 机床故障 X轴报警 Z轴报警 缺料报警 满料报警  加工完成报警   料仓急停  */
/*   bit 7~0																 */
/*   1       1        1      1          1         1           1            1 */
/* X左极限 X右极限 Z上极限 Z下极限  软极限错误 自动安全区                    */
u16 Error_Data = 0;



/*- 报警数据，每一位保存一种类型报警，1表示报警，0表示无报警                -*/
/* Data[0] 急停   轴   机床故障 软限位 安全区 气压异常 润滑报警   预留 */
/* [7:0]    1      1      1      1       1       1        1         1 */
u8 Robot_Error_Data[4] = {0};			
u8 ServoEmergencyStop_Flag=0;//---2017.9.28
u8 Emerg_Flag=0;



extern u8 Axis_Step;
/**************************************************************************************************
**  函数名：  EmergencyStopJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：急停判断
**	备注：	  急停按下，为高电平有效信号
**  作者：          
**  开发日期： 
***************************************************************************************************/
void EmergencyStopJudge(void)
{			
	if(ReadEmergencyStop())					          //读取急停I点,急停为按下状态。高电平有效
	{	  
		EmergencyStop_Flag = TRUE;
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x80;
		Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;
		Timer_Enable_O = DISABLE;
		Timer_Enable_A = DISABLE;
		Timer_Enable_B = DISABLE;		
		
		//Error_Status = ERROR_EMERG_HAPPEN;	
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		TIM_Cmd(TIM1, DISABLE);
//		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
		Axis_Step=0;
		ServoEmergencyStop_Flag=1;//---2017.9.28
	}
	else						                       	  //急停键为释放状态
	{
		EmergencyStop_Flag = FALSE;
		Robot_Error_Data[0] = Robot_Error_Data[0] & 0x7f;
	}
}


/**************************************************************************************************
**  函数名：  LimitSwitchJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：检测极限开关
**	备注：	  无
**  作者：          
**  开发日期： 
***************************************************************************************************/
void LimitSwitchJudge(void)
{			
	 /* X轴左极限检测 */
	 if(!ReadInput(X_LEFT_LIMIT))	 
	 {
		 if(Cancle_Limit_Warning==FALSE)
		 {
//		    Robot_Error_Data[0] = Robot_Error_Data[1] | 0x80;	
		 }
		 Limit_Error_Flag[0]=TRUE;
	 }
	 else
	 {
//	 	   Robot_Error_Data[0] = Robot_Error_Data[1] & 0x7f;
		   Limit_Error_Flag[0]=FALSE;
	 }
	 /* X轴右极限检测 */
	 if(!ReadInput(X_RIGHT_LIMIT))
	 {
		 if(Cancle_Limit_Warning==FALSE)
		 {
		    Error_Data = Error_Data | 0x0040;	
		 }
		 Limit_Error_Flag[1]=TRUE;
	 }
	 else
	 {
		 Error_Data = Error_Data & 0xffbf;
		 Limit_Error_Flag[1]=FALSE;		 
	 }

	 /* Z轴上极限检测 */
	 if(!ReadInput(Z_LEFT_LIMIT))	 
	 {
		 if(Cancle_Limit_Warning==FALSE)
		 {
		    Error_Data = Error_Data | 0x0020;	
		 }
		 Limit_Error_Flag[2]=TRUE;		 	
	 }
	 else
	 {
	 	 Error_Data = Error_Data & 0xffdf;
		 Limit_Error_Flag[2]=FALSE;	
	 }

	 /* Z轴下极限检测 */
	 if(!ReadInput(Z_RIGHT_LIMIT))
	 {
		 if(Cancle_Limit_Warning==FALSE)
		 {
		    Error_Data = Error_Data | 0x0010;	
		 }
		 Limit_Error_Flag[3]=TRUE;	
	 }
	 else
	 {
		 Error_Data = Error_Data & 0xffef;
		 Limit_Error_Flag[3]=FALSE;
	 }
	 //
	 if((Limit_Error_Flag[0]==FALSE) && (Limit_Error_Flag[1]==FALSE) 
	     && (Limit_Error_Flag[2]==FALSE)&& (Limit_Error_Flag[3]==FALSE))
	 {
	 	 Limit_Error=FALSE;
		 Cancle_Limit_Warning=FALSE;
	 }
	 else  //检测到极限开关
	 {
	 	 Origin_Backed=FALSE;		 //回原点标志清零，需要重新回原点
	 }

}


/**************************************************************************************************
**  函数名：  SoftLimitJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：软极限检测
**	备注：	  无
**  作者：          
**  开发日期： 
***************************************************************************************************/
void SoftLimitJudge()
{
	 if(Origin_Backed == TRUE)
	 {
	 	if( Pulse_TotalCounter[0]<X_Axsis_Minlength-1 | Pulse_TotalCounter[0]>X_Axsis_Maxlength |
		    Pulse_TotalCounter[1]<Z_Axsis_Minlength-1 | Pulse_TotalCounter[1]>Z_Axsis_Maxlength | 
			Pulse_TotalCounter[2]<L_Axsis_Minlength-1 | Pulse_TotalCounter[2]>L_Axsis_Maxlength |
			Pulse_TotalCounter[3]<O_Axsis_Minlength-1 | Pulse_TotalCounter[3]>O_Axsis_Maxlength  )
		{
			Robot_Error_Data[0] = Robot_Error_Data[0] | 0x10;
			Timer_Enable_X = DISABLE;
			Timer_Enable_Z = DISABLE;
			Timer_Enable_L = DISABLE;
			Timer_Enable_O = DISABLE;
			Timer_Enable_A = DISABLE;
			Timer_Enable_B = DISABLE;
			//---2017.9.22
			ServoEmergencyStop_CAN(X_Axsis);	
			ServoEmergencyStop_CAN(Z_Axsis);
			TIM_Cmd(TIM1, DISABLE);
//			ServoEmergencyStop_CAN(L_Axsis);
			ServoEmergencyStop_CAN(O_Axsis);
			ServoEmergencyStop_CAN(A_Axsis);
			ServoEmergencyStop_CAN(B_Axsis);			
		}
		else
		{
			Robot_Error_Data[0] = Robot_Error_Data[0] & 0xef;
		}
	 }
}


/**************************************************************************************************
**  函数名：  MachineBreakDown()
**	输入参数：无
**	输出参数：无
**	函数功能：机床故障
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2013/12/17
***************************************************************************************************/
void MachineBreakDown()
{
	 if((ReadInput(11) & ReadInput(19))==0)	  //11==0(低电平):机床1报警  19==0(低电平):机床2报警
	 {
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] | 0x20;
		Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;
		Timer_Enable_O = DISABLE;
		Timer_Enable_A = DISABLE;
		Timer_Enable_B = DISABLE;
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		TIM_Cmd(TIM1, DISABLE);
//		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
	 }
	 else
	 {
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] & 0xdf;
	 }
}

/**************************************************************************************************
**  函数名：  MaterialEmergencyStop()
**	输入参数：无
**	输出参数：无
**	函数功能：料仓急停 PE8
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2013/12/17
***************************************************************************************************/
void MaterialEmergencyStop()
{
	if(Input_Detect_Status[4]&0x03)//X32 -- X33
	{
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x01;
	}
	else
	{
		Robot_Error_Data[0] = Robot_Error_Data[0] & 0xfe;
	}
}

/**************************************************************************************************
**  函数名：  LackMaterialJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：缺料判断
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2013/12/17
***************************************************************************************************/
void LackMaterialJudge()
{
   
	if(g_Material_Load_Disk_Finished)					  //取料仓缺料
	{	  
		 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x40;
	}
	else						                       	  //未缺料
	{
		 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xbf;
	}
}

/**************************************************************************************************
**  函数名：  FullMaterialJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：满料判断
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2013/12/17
***************************************************************************************************/
void FullMaterialJudge()
{
	if(g_Material_UnLoad_Disk_Finished)					  //卸料仓满料状态
	{	  
		 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x80;
	}
	else						                       	  //未满料
	{
		 Robot_Error_Data[1] = Robot_Error_Data[1] & 0x7f;
	}
}

/**************************************************************************************************
**  函数名：  XAxsisError()
**	输入参数：无
**	输出参数：无
**	函数功能：X轴错误，高电平报警
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2013/12/17
***************************************************************************************************/
void  XAxsisError()
{
	if( ReadXAxsisError() )					          //读取X轴错误信号
	{	  
		Error_Data = Error_Data | 0x2000;
		TIM_Cmd(TIM2, DISABLE);
		TIM_Cmd(TIM3, DISABLE);
		TIM_Cmd(TIM1, DISABLE);
		TIM_Cmd(TIM4, DISABLE);
		Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;	
		Timer_Enable_O = DISABLE;
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		TIM_Cmd(TIM1, DISABLE);
//		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
		//Error_Status = ERROR_EMERG_HAPPEN;	
	}
	else						                       	  
	{
		Error_Data = Error_Data & 0xdfff;
	}		
}


/**************************************************************************************************
**  函数名：  ZAxsisError()
**	输入参数：无
**	输出参数：无
**	函数功能：Z轴错误，高电平报警
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2013/12/17
***************************************************************************************************/
void  ZAxsisError()
{
	if( ReadZAxsisError() )					          //读取Z轴错误信号
	{	  
		Error_Data = Error_Data | 0x1000;
	  Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;
		Timer_Enable_O = DISABLE;
		Timer_Enable_A = DISABLE;
		Timer_Enable_B = DISABLE;
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		TIM_Cmd(TIM1, DISABLE);
//		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
		//Error_Status = ERROR_EMERG_HAPPEN;
	}
	else						                       	  
	{
		Error_Data = Error_Data & 0xefff;
	}	


}

/**************************************************************************************************
**  函数名：  AxsisError()
**	输入参数：无
**	输出参数：无
**	函数功能：Z轴错误，高电平报警
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2013/12/17
***************************************************************************************************/
void  AxsisError()
{
	if( ReadAxsisError(ERROR_STATUS)==TRUE )					          //读取机械轴错误信号
	{	  
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x40;
	    Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;
		Timer_Enable_O = DISABLE;	
		Timer_Enable_A = DISABLE;
		Timer_Enable_B = DISABLE;
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		TIM_Cmd(TIM1, DISABLE);
//		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
	}
	else						                       	  
	{
		Robot_Error_Data[0] = Robot_Error_Data[0] & 0xbf;
	}	
}

/**************************************************************************************************
**  函数名：  HardLimitJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：软极限检测
**	备注：	  无
**  作者：          
**  开发日期： 
***************************************************************************************************/
void HardLimitJudge()
{
    if((Origin_Backed==TRUE)&&((Input_Detect_Status[4]&0xf0)!=0x00))
	{
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x08;
		Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;
		Timer_Enable_O = DISABLE;
		Timer_Enable_A = DISABLE;
		Timer_Enable_B = DISABLE;
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		TIM_Cmd(TIM1, DISABLE);
//		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
	}
}

/**************************************************************************************************
**  函数名：  AirAlarm()
**	输入参数：无
**	输出参数：无
**	函数功能：气泵报警
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2016/01/16
***************************************************************************************************/
void  AirAlarm()
{
	 if(ReadInput(41)==0)	  //41==0(低电平):气源报警
	 {
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] | 0x08;
		Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;
		Timer_Enable_O = DISABLE;
		Timer_Enable_A = DISABLE;
		Timer_Enable_B = DISABLE;
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		TIM_Cmd(TIM1, DISABLE);
//		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
	 }
	 else
	 {
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] & 0xf7;
	 }
	
}

/**************************************************************************************************
**  函数名：  OilingAlarm()
**	输入参数：无
**	输出参数：无
**	函数功能：润滑报警
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2016/01/16
***************************************************************************************************/
void  OilingAlarm()
{
	 if(ReadInput(40)==0)	  //40==0(低电平):润滑报警
	 {
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] | 0x04;
		Timer_Enable_X = DISABLE;
		Timer_Enable_Z = DISABLE;
		Timer_Enable_L = DISABLE;
		Timer_Enable_O = DISABLE;
		Timer_Enable_A = DISABLE;
		Timer_Enable_B = DISABLE;
		//---2017.9.22
		ServoEmergencyStop_CAN(X_Axsis);	
		ServoEmergencyStop_CAN(Z_Axsis);
		TIM_Cmd(TIM1, DISABLE);
//		ServoEmergencyStop_CAN(L_Axsis);
		ServoEmergencyStop_CAN(O_Axsis);
		ServoEmergencyStop_CAN(A_Axsis);
		ServoEmergencyStop_CAN(B_Axsis);
	 }
	 else
	 {
	 	 Robot_Error_Data[0] = Robot_Error_Data[0] & 0xfb;
	 }

}

/**************************************************************************************************
**  函数名：  ManulActionError()
**	输入参数：无
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2016/01/23
***************************************************************************************************/
void  ManulActionError()
{
	 if(Safe_Area_Flag)	  //
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x08;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xf7;
	 }
}

/**************************************************************************************************
**  函数名：  CurrentWorkFinished()
**	输入参数：无
**	输出参数：无
**	函数功能：动作超时报警
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2016/01/18
***************************************************************************************************/
void  CurrentWorkFinished()
{
	 if(g_Auto_WorkFinished_Flag)	  //
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x20;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xdf;
	 }
}

/**************************************************************************************************
**  函数名：  AutoActionError()
**	输入参数：无
**	输出参数：无
**	函数功能：
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2016/01/18
***************************************************************************************************/
void  AutoActionError()
{
	 if(g_Auto_ActionError_Flag)	  //
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x10;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xef;
	 }

}
/**************************************************************************************************
**  函数名：  AutoActionTimeOut()
**	输入参数：无
**	输出参数：无
**	函数功能：动作超时报警
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2016/01/18
***************************************************************************************************/
void  AutoActionTimeOut()
{
	 if(g_Auto_ActionTimeOut_Flag)	  //
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] | 0x04;
	 }
	 else
	 {
	 	 Robot_Error_Data[1] = Robot_Error_Data[1] & 0xfb;
	 }
}

/**************************************************************************************************
**  函数名：  USARTSyncTimeOut()
**	输入参数：无
**	输出参数：无
**	函数功能：串口通信超时报警
**	备注：	  无
**  作者：    吴祥   
**  开发日期：2016/01/18
***************************************************************************************************/
void  USARTSyncTimeOut()
{
    if(g_USART_Delay_Timer>300)//超过3000ms了，则报错
	{
	    if(g_AutoStatue==AUTO_RUNNING)	//自由编程-调试 状态也满足条件,将进入暂停状态
	    {
	   		g_Auto_Order_Pause = TRUE;
	    }
		else if(Work_Status==MANUAL_WORK_MODE)
		{
			if(Timer_Enable_X|Timer_Enable_Z|Timer_Enable_L|Timer_Enable_O)
			{
//                Servo_Stop(Axsis_Chosen);
				          ServoStop_CAN(Axsis_Chosen);
				          CAN_Servo_Stop_Action[Axsis_Chosen]=FALSE;
            }
		}
		g_USART_Delay_Timer=0;
	}
}

/**************************************************************************************************
**  函数名：  ErrorOperate()
**	输入参数：无
**	输出参数：无
**	函数功能：报警操作
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void ErrorOperate()
{
	 u8 i=0;
	 u8 Temp_data[4]={0};
	 EmergencyStopJudge(); //急停检测
//	 AxsisError();		   //机械轴报警
//	 #ifdef HARDLIMITJUDGE_INPUT
//	     HardLimitJudge();	   //硬限位报警
//	 #endif
	 //MachineBreakDown();	   //机床故障检测
//	 SoftLimitJudge();		   //软限位检测	 
	 //AirAlarm();		       //气泵报警
	 //OilingAlarm();		   //润滑报警
//	 SafeAreaJudge();	       //安全区检测	 --紧急
//	 ManulActionError();	   //手动操作错误-超出安全区,请反向操作
	 //USARTSyncTimeOut();	   //串口通信报警 
	 //MaterialEmergencyStop();  //料仓限位报警-
	 if(Work_Status == AUTO_WORK_MODE)
	 {		 		 
		 //LackMaterialJudge();	   //缺料监测-一个料盘加工完成
		 //FullMaterialJudge();	   //满料监测
		 CurrentWorkFinished();	   //加工完成		 
		 AutoActionError();	       //自动运行进入AUTO_ERROR
		 AutoActionTimeOut();	   //动作超时			  
	 }
	 
	 //取消当前报警
     if(Cancle_Genaral_Warning==TRUE)
	 {
		 //紧急急停报警不清零，其他报警直接清零
//		 if(Safe_Area_Alarm)
//		 {
//		 	Safe_Area_Alarm=FALSE;
//		 }
		 for(i=0;i<4;i++)
		 {
		 	Robot_Error_Data[i]=0;
		 }
//		 if(Robot_Error_Data[0] & 0x08)	 //限位报警
//		 {
//		 	Robot_Error_Data[0] = Robot_Error_Data[0] & 0xf7;
//		 }
		 g_Auto_ActionError_Flag=FALSE;
		 g_Auto_ActionTimeOut_Flag=FALSE;
		 Safe_Area_Flag=FALSE;
		 Cancle_Genaral_Warning=FALSE;
		 g_Material_Load_Disk_Finished=FALSE;
		 g_Material_UnLoad_Disk_Finished=FALSE;
		 if(g_Auto_WorkFinished_Flag)	//清除当前生产产量参数
		 {
			SC_Parameter.SC_Num=0;
			Temp_data[0] = SC_Parameter.SC_Num;
			Temp_data[1] = SC_Parameter.SC_Num>>8;
			Temp_data[2] = SC_Parameter.SC_Num>>16;
			Temp_data[3] = SC_Parameter.SC_Num>>24;
			W25QXX_Write(Temp_data,0x102C,4);
			g_Auto_WorkFinished_Flag=FALSE;		 
		 }	
    
	 }
	 for(i=0;i<4;i++)
	 {
	 	if(Robot_Error_Data[i] != 0 )
		{
			if(Robot_Error_Data[0]!=0)
			{
				Error_Status = ERROR_EMERG_HAPPEN;
				//---2017.9.28
				Emerg_Flag=1;
			}
			else
			{
				Error_Status = ERROR_HAPPEN;
			}
			break;
		}
		else
		{
			Error_Status = NO_ERROR;
		}
	 }
	 if(Error_Status==NO_ERROR)	       //未发生错误
	 {	                
			//---2017.9.28
			if(Emerg_Flag)
			{
			//按照状态转换图实现状态转换
			 Change_Servo_State_Machine(X_Axsis,SWITCH_ON);
			 Change_Servo_State_Machine(Z_Axsis,SWITCH_ON);
			 Change_Servo_State_Machine(L_Axsis,SWITCH_ON);
			 Change_Servo_State_Machine(O_Axsis,SWITCH_ON);
			 Change_Servo_State_Machine(A_Axsis,SWITCH_ON);
			 Change_Servo_State_Machine(B_Axsis,SWITCH_ON);
			 Delay(10);
			 Change_Servo_State_Machine(X_Axsis,ENABLE_OPERATION);
			 Change_Servo_State_Machine(Z_Axsis,ENABLE_OPERATION);
			 Change_Servo_State_Machine(L_Axsis,ENABLE_OPERATION);
			 Change_Servo_State_Machine(O_Axsis,ENABLE_OPERATION);
			 Change_Servo_State_Machine(A_Axsis,ENABLE_OPERATION);
			 Change_Servo_State_Machine(B_Axsis,ENABLE_OPERATION);
			 Emerg_Flag=0;
			 
			 ClosePulseReset(L_Axsis);
			}	
		   
	 }
   else					      //发生错误
	 {		 
		 if( Error_Status == ERROR_EMERG_HAPPEN ) //急停报警，直接停止机械手   //||(Robot_Error_Data[0]&0x40)||(Robot_Error_Data[0]&0x20)||(Robot_Error_Data[0]&0x01))	
		 { 
			   AutoReset();
				Timer_Enable_X = DISABLE;
				Timer_Enable_Z = DISABLE;
				Timer_Enable_L = DISABLE;
				Timer_Enable_O = DISABLE;
				Timer_Enable_A = DISABLE;
				Timer_Enable_B = DISABLE;
				//---2017.9.22
				ServoEmergencyStop_CAN(X_Axsis);	
				ServoEmergencyStop_CAN(Z_Axsis);
				TIM_Cmd(TIM1, DISABLE);
//				ServoEmergencyStop_CAN(L_Axsis);
				ServoEmergencyStop_CAN(O_Axsis);
				ServoEmergencyStop_CAN(A_Axsis);
				ServoEmergencyStop_CAN(B_Axsis);
			 
			     ServoEmergencyStop_Flag=1;//---2017.9.28
			 
		 }
		 else			//其他报警，按照停止处理
		 {			  
			   if(g_AutoStatue==AUTO_RUNNING)	//自由编程-调试 状态也满足条件,将进入暂停状态
			   {
			   		g_Auto_Order_Pause = TRUE;
			   }	   
			   Linked_Move_Enable=DISABLE;
			   Jog_Move_Enable=DISABLE;	
			   if(Back_Origin_Flag)
			   {
			   	   Back_Origin_Flag = FALSE;
			   }              			   		 
		 }
	 }
	 //---2017.9.28是否可以在这判断急停完后失能
	 if(ServoEmergencyStop_Flag)
	 {
		  //判断伺服器位置到达或停止
				if(Timer_Enable_X )
				{		     
					 if(Axsis_Stop_Flag[X_Axsis])//--判断X轴是否已经停下来
					 {
						Timer_Enable_X = DISABLE;					
						Axsis_Stop_Flag[X_Axsis]=0;
					 }
				}	 
				if(Timer_Enable_Z )
				{		     
					 if(Axsis_Stop_Flag[Z_Axsis])//--判断Z轴是否已经停下来
					 {								
							Timer_Enable_Z = DISABLE;
							Axsis_Stop_Flag[Z_Axsis]=0;
					  }
				 }
				if(Timer_Enable_L )
				{		     
					 if(Axsis_Stop_Flag[L_Axsis])//--判断L轴是否已经停下来
					 {								
							Timer_Enable_L = DISABLE;
							Axsis_Stop_Flag[L_Axsis]=0;
					 }
				 }
				if(Timer_Enable_O )
				{		     
					 if(Axsis_Stop_Flag[O_Axsis])//--判断O轴是否已经停下来
					 {
							Timer_Enable_O = DISABLE;	
							Axsis_Stop_Flag[O_Axsis]=0;
					 }
				}
				if(Timer_Enable_A )
				{		     
					 if(Axsis_Stop_Flag[A_Axsis])//--判断L轴是否已经停下来
					 {								
							Timer_Enable_A = DISABLE;
							Axsis_Stop_Flag[A_Axsis]=0;
					 }
				 }
				if(Timer_Enable_B )
				{		     
					 if(Axsis_Stop_Flag[B_Axsis])//--判断O轴是否已经停下来
					 {
							Timer_Enable_B = DISABLE;	
							Axsis_Stop_Flag[B_Axsis]=0;
					 }
				}
			if((Timer_Enable_X && Timer_Enable_Z && Timer_Enable_L && Timer_Enable_O&& Timer_Enable_A&& Timer_Enable_B)==DISABLE)
			{
				ServoEmergencyStop_Flag=0;
			}
	 }
	 else
	 {}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Usart.h" 
#include "StatusControl.h"
#include "SpeedControl.h"
#include "Auto.h"
#include "Manual.h"
#include "out.h"
#include "in.h"
#include "Error.h"
#include "Parameter.h"
#include "SignalWatch.h"
#include "Auto_2.h"
#include "PDO.h"

#include "SpeedControl_CAN.h"

//u8 Servo_Is_Run = FALSE;		   //伺服器是否正在运转标志位
u8 Synchronize = FALSE;			   //同步标志位
u8 Initialize_Finished = FALSE;	   //初始化完成标志位
u8 Origin_Backed = FALSE;		   //达到原点标志位
u8 X_Origin_Backed = FALSE;		   //X轴达到原点标志位
u8 Z_Origin_Backed = FALSE;		   //Z轴达到原点标志位
u8 L_Origin_Backed = FALSE;		   //L轴达到原点标志位
u8 O_Origin_Backed = FALSE;		   //O轴达到原点标志位
u8 Back_Origin_Flag = FALSE;	   //回原点标志位
u8 Origin_Speed_Down=FALSE;        //回原点检测到信号后，状态为减速状态标志位
u32 Back_Origin_Speed = 50;	       //回原点速度
u8 Robot_Enable_Flag = FALSE;	   //机械手使能
//u8 Origin_Type = Z_X_TYPE;		   //回原点方式

u8 Work_Status = AUTO_WORK_MODE;        //机械手工作模式 
u8 Axis_Manul_BackToOrigin=FALSE;  //手动回零
u8 Axsis_Chosen = X_Axsis;		   //运动轴选择
u8 Axsis_Move_Direction[6] = {POSITIVE,POSITIVE,POSITIVE,POSITIVE,POSITIVE,POSITIVE};//运动方向选择			  
u32 Input_Detect_Time = 0;		   //输入检测定时参数
u8 Action_Detect_Flag=FALSE;	   //动作监视
u8 Input_Detect_Enable = ENABLE;   //输入检测使能标志，第一次使能
u8 Jog_Pause_Enable=DISABLE;       //寸动操作暂停
u8 Position_Handle_Flag=FALSE;     //是否计算位置标志
u8 SafeArea_Is_On=FALSE;
u8 X_In_SafeArea_Flag=FALSE;
u8 Safe_Area_Flag=FALSE;		   //安全区域判断，禁止当前操作，给标志位，手控器跳出提醒界面
u8 Safe_Area_Alarm=FALSE;		   //安全区域报警，超出安全区
u8 g_Current_SafeArea_Num=0;	   //安全区编号
u8 Input_Count15 = 0;			   //输入15计数
u8 Input_Count16 = 0;			   //输入16计数
u8 Input15_16_Detect=DISABLE;	   //机器人启动停止判断标志位
u8 Servo_Stop_Done[6]={FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};		   //伺服器停止动作是否执行标志
u8 Test_ServoStop=0;
u8 g_Auto_Reset_Flag = FALSE;	   //自动复位标志位
u8 Robot_Auto_Reset = FALSE;
u8 g_Auto_LOrigin_Flag=FALSE;	   //L轴回零
u32 g_USART_Delay_Timer=0;		   //串口延时计数-方式通信失败
static u8 gs_AutoStatue=4;
static u8 gs_Error_Status = NO_ERROR;

//------------------------------------------//

/**************************************************************************************************
**  函数名：  WorkMode()
**	输入参数：无
**	输出参数：无
**	函数功能：工作模式选择
**	备注：	  无
**  作者：         
**  开发日期： 
***************************************************************************************************/
void WorkMode()
{
	switch(UsartReceiveData[1])
	{
		case P_AUTO_MODE:
			 Work_Status = AUTO_WORK_MODE;
			 if(g_Robot_Has_Debuged_Flag)
			 {
				 g_AutoStatue = AUTO_WAITE;
				 g_Auto_Order_Start = FALSE;
				 g_Auto_Order_Pause = FALSE;
				 g_Auto_Order_Stop = FALSE;
				 Single_Mode_Enable = DISABLE;
				 g_Auto_PresentLine = 0;	  //当前单步的行
				 g_Auto_ActionRun_Step=0;
				 Robot_Auto_Reset=FALSE;		
				 g_Robot_Has_Debuged_Flag=FALSE;	 
			 }
		   break;	

		case P_FREE_PROGRAM:
			 Work_Status = FREE_PROGRAM_MODE;     
			 break;	

		case P_IO_MODE:
			 Work_Status = IO_DEBUG_MODE;
		   break;	

		case P_MANUL_MODE:
			 Work_Status = MANUAL_WORK_MODE;
			 Axsis_Chosen = X_Axsis;
			 Axsis_Move_Direction[X_Axsis] = POSITIVE;
			 Linked_Mode_Enable = ENABLE;
			 Jog_Mode_Enable = DISABLE;
		   Jog_Pause_Enable=DISABLE;
		   break;

	    case WAIT_MODE:
			 Work_Status = WAIT_MODE;
	 		 break;

		default:
		   break;
	}
}

/**************************************************************************************************
**  函数名：  DirectionJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：机械手动作控制
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void DirectionJudge(u8 Axis)
{
   switch(Axis)
   {
       case X_Axsis:     	 
  	      if(Axsis_Move_Direction[X_Axsis] == POSITIVE)
          {
		        SetOutput(X_AXSIS_DIRECTION);
	        }
	        else if(Axsis_Move_Direction[X_Axsis] == NEGATIVE)
	        {
		        ResetOutput(X_AXSIS_DIRECTION);
	        }
			    break;
					
   	   /*- Z轴向下运动为正向，向上运动为反向 -*/
       case Z_Axsis:  
	   	    if(Axsis_Move_Direction[Z_Axsis] == POSITIVE)
	        {
			     ResetOutput(Z_AXSIS_DIRECTION);
		      }
		      else if(Axsis_Move_Direction[Z_Axsis] == NEGATIVE)
		      {			    
				   SetOutput(Z_AXSIS_DIRECTION);
		      }
			    break;

       case L_Axsis:     	 
  	      if(Axsis_Move_Direction[L_Axsis] == POSITIVE)
          {
		        ResetOutput(L_AXSIS_DIRECTION);
	        }
	        else if(Axsis_Move_Direction[L_Axsis] == NEGATIVE)
	        {
		        SetOutput(L_AXSIS_DIRECTION);
	        }
			    break;
					
       case O_Axsis:     	 
  	      if(Axsis_Move_Direction[O_Axsis] == POSITIVE)
          {
		        SetOutput(O_AXSIS_DIRECTION);
	        }
	        else if(Axsis_Move_Direction[O_Axsis] == NEGATIVE)
	        {
		        ResetOutput(O_AXSIS_DIRECTION);
	        }
			    break;
					
	     default: 
	        break;
   }
}

/**************************************************************************************************
**  函数名：  SafeAreaDetection()
**	输入参数：无
**	输出参数：无
**	函数功能：机械手运动安全区域检测
**	备注：	  
**			  
**  作者：    吴祥     
**  开发日期：2013/2/10 
***************************************************************************************************/
void SafeAreaDetection()
{
	u8 Num=0;
	if(SafeArea_Is_On)
	{
		//如果Z轴位于0点,且运行的是X轴,则认为绝对安全
		if((Pulse_TotalCounter[1]==MINROBOTPOSITION) && (Axsis_Chosen==X_Axsis))
		{
			Safe_Area_Flag=FALSE;
		}
		else
		{
			if(g_Current_SafeArea_Num)
			{
			 	Num=g_Current_SafeArea_Num-1;
				//X轴在安全区域Num之中时
				if((Pulse_TotalCounter[0]>=Robot_Safe_Area[Num].X_Left)&&(Pulse_TotalCounter[0]<=Robot_Safe_Area[Num].X_Right))  	   
			 	{
					   /*- 安全区域标志位判断 -*/
					   if((Pulse_TotalCounter[0]==Robot_Safe_Area[Num].X_Left)&&(Axsis_Chosen==X_Axsis)&&(Axsis_Move_Direction[X_Axsis]==NEGATIVE))
					   {
					   		Safe_Area_Flag=TRUE;
					   }
					   if((Pulse_TotalCounter[0]==Robot_Safe_Area[Num].X_Right)&&(Axsis_Chosen==X_Axsis)&&(Axsis_Move_Direction[X_Axsis]==POSITIVE))
					   {
					   		Safe_Area_Flag=TRUE;
					   }
					   if((Pulse_TotalCounter[1]==Robot_Safe_Area[Num].Z_Down)&&(Axsis_Chosen==Z_Axsis)&&(Axsis_Move_Direction[Z_Axsis]==POSITIVE))
					   {
					   		Safe_Area_Flag=TRUE;
					   }
					   /*- 安全区域标志位判断 -*/
					   if(Pulse_TotalCounter[1]<Robot_Safe_Area[Num].Z_Up)	 //Z轴小于料仓区域Z上安全位置
					   {
							 if((Linked_Mode_Enable==ENABLE) && (Axsis_Chosen==Z_Axsis) && (Axsis_Move_Direction[Z_Axsis]==POSITIVE))
							 {							  
								   Linked_Pulse=Robot_Safe_Area[Num].Z_Down-Pulse_TotalCounter[1];
							 }   
					   }
					   else if(Pulse_TotalCounter[1]>Robot_Safe_Area[Num].Z_Down) //Z轴大于料仓区域Z下安全位置
					   {
						   if(Axsis_Chosen==Z_Axsis)								   
						   {
							   if(Axsis_Move_Direction[Z_Axsis] == POSITIVE)	   //Z轴，且命令反向运动，则不动作
							   {
								   Safe_Area_Flag=TRUE;
							   }
						   }
						   else if(Axsis_Chosen==X_Axsis)			   //X轴，不动作
						   {
							   Safe_Area_Flag=TRUE;
						   }
						   else
						   {
						   }
					   }
					   else	 //Z在安全区域中  XZ都处于安全区域内,对行程进行限定
					   {
							if(Jog_Mode_Enable == ENABLE )	   //寸动模式
							{
								 if(Axsis_Chosen == X_Axsis)
								 {
									  //防止寸动超过左X安全坐标
										if(((Pulse_TotalCounter[0]-Jog_Pulse_Count)<=Robot_Safe_Area[Num].X_Left)&&(Axsis_Move_Direction[X_Axsis]==NEGATIVE))
										{
											 Jog_Pulse_Count=Pulse_TotalCounter[0]-Robot_Safe_Area[Num].X_Left;
										}
									  //防止寸动超过右X安全坐标
										if(((Pulse_TotalCounter[0]+Jog_Pulse_Count)>=Robot_Safe_Area[Num].X_Right)&&(Axsis_Move_Direction[X_Axsis]==POSITIVE))
										{
											 Jog_Pulse_Count=Robot_Safe_Area[Num].X_Right-Pulse_TotalCounter[0];
										}
								 }
								 else if(Axsis_Chosen == Z_Axsis)
								 {								
										if(((Pulse_TotalCounter[1]+Jog_Pulse_Count)>=Robot_Safe_Area[Num].Z_Down)&&(Axsis_Move_Direction[Z_Axsis]==POSITIVE))
										{
											 Jog_Pulse_Count=Robot_Safe_Area[Num].Z_Down-Pulse_TotalCounter[1];
										}
								 }
								 else
								 {
								 }						 	
							}
							else if(Linked_Mode_Enable == ENABLE )	//连动模式
							{
								 if(Axsis_Chosen == X_Axsis)
								 {							  
									   //防止连动超过X左安全坐标
									   if(Axsis_Move_Direction[X_Axsis] == NEGATIVE)
								       {
										     Linked_Pulse=Pulse_TotalCounter[0]-Robot_Safe_Area[Num].X_Left;
								       }
									   //防止连动超过X右安全坐标
									   else if(Axsis_Move_Direction[X_Axsis] == POSITIVE)
									   {
									   		 Linked_Pulse=Robot_Safe_Area[Num].X_Right-Pulse_TotalCounter[0];
									   }
									   else
									   {
									   }
								 }
								 else if(Axsis_Chosen == Z_Axsis)
								 {							  
									   if(Axsis_Move_Direction[Z_Axsis] == POSITIVE)
									   {
									   		Linked_Pulse=Robot_Safe_Area[Num].Z_Down-Pulse_TotalCounter[1];
									   }
								 }						 					
							}
							else
							{
							}
					   }//else	Z在安全区域中
				}//X轴在安全区域Num之中时 	 	
			}//if(g_Current_SafeArea_Num)
			//当X轴不在任何安全区内时,Z轴禁止上下移动 
			else
			{
			   if(Axsis_Chosen==Z_Axsis)
			   {
				   Safe_Area_Flag=TRUE;
			   }	
			}
		}	//if(Pulse_TotalCounter[1]==MINROBOTPOSITION)
	}//if(SafeArea_Is_On)
	/*--  超过软极限之后的方向处理 --*/
	if(Pulse_TotalCounter[0]<=X_Axsis_Minlength && Axsis_Chosen==X_Axsis && Axsis_Move_Direction[X_Axsis]==NEGATIVE)
	{
		Safe_Area_Flag=TRUE;
	}
	if(Pulse_TotalCounter[0]>=X_Axsis_Maxlength && Axsis_Chosen==X_Axsis && Axsis_Move_Direction[X_Axsis]==POSITIVE)
	{
		Safe_Area_Flag=TRUE;
	}
	if(Pulse_TotalCounter[1]<=Z_Axsis_Minlength && Axsis_Chosen==Z_Axsis && Axsis_Move_Direction[Z_Axsis]==NEGATIVE)
	{
		Safe_Area_Flag=TRUE;
	}
	if(Pulse_TotalCounter[1]>=Z_Axsis_Maxlength && Axsis_Chosen==Z_Axsis && Axsis_Move_Direction[Z_Axsis]==POSITIVE)
	{		
		Safe_Area_Flag=TRUE;
	}
	/*--  在边界禁止操作,发送报警 --*/
	if(Safe_Area_Flag)
	{
		Axsis_Move_Direction[X_Axsis]=NONE;
		Axsis_Move_Direction[Z_Axsis]=NONE;
		Axsis_Move_Direction[L_Axsis]=NONE;
		Axsis_Move_Direction[O_Axsis]=NONE;
		Axsis_Move_Direction[A_Axsis]=NONE;
		Axsis_Move_Direction[B_Axsis]=NONE;
		Linked_Move_Enable=DISABLE;
		//Jog_Move_Enable=DISABLE;
	}
}
void Halt_To_Reset(void)
{
//判断伺服器位置到达或停止
	if(Timer_Enable_X )
	{		     
		 if(Axsis_Stop_Flag[X_Axsis])//--判断X轴是否已经停下来
		 {
				if(Set_Halt_Flag[X_Axsis])//是否是暂停
				{
						Reset_Halt(X_Axsis);
						Set_Halt_Flag[X_Axsis]=0;
//						Timer_Enable_X = DISABLE;
				}
				else if(Target_Rreached[X_Axsis])//是否是 目标到达
				{
//						Timer_Enable_X = DISABLE;
				}	
				Axsis_Stop_Flag[X_Axsis]=0;
		  }
	 }
if(Timer_Enable_Z )	//再次启动
	{		     
		 if(Axsis_Stop_Flag[Z_Axsis])//--判断Z轴是否已经停下来
		 {
				if(Set_Halt_Flag[Z_Axsis])//  是否是暂停
				{
						Reset_Halt(Z_Axsis);
						Set_Halt_Flag[Z_Axsis]=0;
//						Timer_Enable_Z = DISABLE;
				}
				else if(Target_Rreached[Z_Axsis])//是否是 目标到达
				{
						Timer_Enable_Z = DISABLE;
				}	
				Axsis_Stop_Flag[Z_Axsis]=0;
		  }
	 }
		if(Timer_Enable_L )
	{		     
			 if(Axsis_Stop_Flag[L_Axsis])//--判断L轴是否已经停下来
			 {
					if(Set_Halt_Flag[L_Axsis])//  是否是暂停
					{
							Reset_Halt(L_Axsis);
							Set_Halt_Flag[L_Axsis]=0;
//							Timer_Enable_L = DISABLE;
					}
					else if(Target_Rreached[L_Axsis])//是否是 目标到达
					{
							Timer_Enable_L = DISABLE;
					}	
					Axsis_Stop_Flag[L_Axsis]=0;
			 }
		 }
			if(Timer_Enable_O )
			{		     
				 if(Axsis_Stop_Flag[O_Axsis])//--判断O轴是否已经停下来
				 {
						if(Set_Halt_Flag[O_Axsis])//  是否是暂停
						{
								Reset_Halt(O_Axsis);
								Set_Halt_Flag[O_Axsis]=0;
//								Timer_Enable_O = DISABLE;
						}
						else if(Target_Rreached[O_Axsis])//是否是 目标到达
						{
								Timer_Enable_O = DISABLE;
						}	
						Axsis_Stop_Flag[O_Axsis]=0;
				 }
		  }
		if(Timer_Enable_A )
	{		     
			 if(Axsis_Stop_Flag[A_Axsis])//--判断L轴是否已经停下来
			 {
					if(Set_Halt_Flag[A_Axsis])//  是否是暂停
					{
							Reset_Halt(A_Axsis);
							Set_Halt_Flag[A_Axsis]=0;
//							Timer_Enable_A = DISABLE;
					}
					else if(Target_Rreached[A_Axsis])//是否是 目标到达
					{
							Timer_Enable_A = DISABLE;
					}	
					Axsis_Stop_Flag[A_Axsis]=0;
			 }
		 }
			if(Timer_Enable_B )
			{		     
				 if(Axsis_Stop_Flag[B_Axsis])//--判断O轴是否已经停下来
				 {
						if(Set_Halt_Flag[B_Axsis])//  是否是暂停
						{
							Reset_Halt(B_Axsis);
							Set_Halt_Flag[B_Axsis]=0;
//							Timer_Enable_B = DISABLE;
						}
						else if(Target_Rreached[B_Axsis])//是否是 目标到达
						{
								Timer_Enable_B = DISABLE;
						}	
						Axsis_Stop_Flag[B_Axsis]=0;
				 }
		  }
}
/**************************************************************************************************
**  函数名：  ActionControl()
**	输入参数：无
**	输出参数：无
**	函数功能：机械手动作控制
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void ActionControl()
{
	if(Error_Status != ERROR_EMERG_HAPPEN)
	{
//		Halt_To_Reset();
		/**--------------------自动模式-------------------------**/ 
		 if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE && Origin_Backed==TRUE)
		 {
			  Servo_Control_Mode=AUTORUNNINGMODE;
			  if(g_Auto_Reset_Flag)
			  {
			  	 RobotAutoReset();				 				
			  }
			  if(Robot_Auto_Reset)
			  {
			  	 AutoRunning();	
			  }			
		 }
		/**--------------------手动模式-------------------------**/
		 else if(Work_Status == MANUAL_WORK_MODE && Back_Origin_Flag == FALSE && Origin_Backed==TRUE )
		 {	    
			   /*******连动模式*********/
			   if(Linked_Mode_Enable == ENABLE )
			  {
				  ManualLinkedRunning();
			  }	
		 }
		 else  //---不是自动和手动模式0925
		 {}			 
		}						 			 
}	
/**************************************************************************************************
**  函数名：  StatusControl()
**	输入参数：无
**	输出参数：无
**	函数功能：输入状态检测
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void StatusControl()
{
    u8 i=0,data_temp[5];
	if(Input_Detect_Enable == ENABLE)
	{
		for(i=1;i<22;i++)
		{		
		   ReadInput(i);
		}
		Input_Detect_Enable = DISABLE;
		
		/*- 自动运行模式下，机器人启动与停止按键检测 -*/    
		    if(Input_Detect_Status[1] & 0x40)	 //启动 X15
		    {
					Input_Detect_Status[1]&=0xbf;
				 if(Work_Status == AUTO_WORK_MODE && Origin_Backed)
				 {				 			
					  data_temp[0]=0x01;
						USART1_SendData(1,P_AUTO_RUN_LCD,data_temp);						
				 }
		    }
		    if(Input_Detect_Status[1] & 0x20)	 //暂停	X16
		    {
					Input_Detect_Status[1]&=0xdf;
					if(Work_Status == AUTO_WORK_MODE && Origin_Backed)
				 {
					  data_temp[0]=0x02;
						USART1_SendData(1,P_AUTO_RUN_LCD,data_temp);	
				 }
		    }
				if(Input_Detect_Status[1] & 0x10) 
				{
					 Input_Detect_Status[1]&=0xef;
					if(Origin_Backed&&(g_AutoStatue==AUTO_PAUSE))//已经回零点且机器停止允许复位
					{
					  data_temp[0]=0x03;
						USART1_SendData(1,P_AUTO_RUN_LCD,data_temp);//停止
					}	
					if(g_AutoStatue==AUTO_WAITE)
					{
						data_temp[0]=0x04;
						USART1_SendData(1,P_AUTO_RUN_LCD,data_temp);//复位						
					}
				}
		//三色灯控制
	   if(g_AutoStatue != gs_AutoStatue)
	   {
	       gs_AutoStatue=g_AutoStatue;
		   switch(g_AutoStatue)
		   {
		       case AUTO_RUNNING:
				      SetOutput(O_IODEBUG_OUTPUT_37);	   //黄灯
					    SetOutput(O_IODEBUG_OUTPUT_38);	   //红灯
					    SetOutput(O_IODEBUG_OUTPUT_39);	   //蜂鸣器
              ResetOutput(O_IODEBUG_OUTPUT_36);  //绿灯
			        break;

		       case AUTO_PAUSE:
				      SetOutput(O_IODEBUG_OUTPUT_36);
					    SetOutput(O_IODEBUG_OUTPUT_38);
					    SetOutput(O_IODEBUG_OUTPUT_39);	  //蜂鸣器
				      ResetOutput(O_IODEBUG_OUTPUT_37); //黄灯
			        break;

		       case AUTO_WAITE:
				      SetOutput(O_IODEBUG_OUTPUT_36);
					    SetOutput(O_IODEBUG_OUTPUT_38);
					    SetOutput(O_IODEBUG_OUTPUT_39);	  //蜂鸣器
				      ResetOutput(O_IODEBUG_OUTPUT_37); //黄灯
			        break;

		       case AUTO_ERROR:
				      SetOutput(O_IODEBUG_OUTPUT_36);
					    SetOutput(O_IODEBUG_OUTPUT_37);					
					    ResetOutput(O_IODEBUG_OUTPUT_38); //红灯
					    ResetOutput(O_IODEBUG_OUTPUT_39); //蜂鸣器
			        break;

		       default:
			       break;
		   }
	   }
	   if(Error_Status != gs_Error_Status)
	   {
	     gs_Error_Status=Error_Status;
		   if(Error_Status==ERROR_EMERG_HAPPEN)
		   {		   
			   SetOutput(O_IODEBUG_OUTPUT_36);
			   SetOutput(O_IODEBUG_OUTPUT_37);					
			   ResetOutput(O_IODEBUG_OUTPUT_38); //红灯
			   ResetOutput(O_IODEBUG_OUTPUT_39); //蜂鸣器	 
		   }
		   else 
		   {	     	   
			   SetOutput(O_IODEBUG_OUTPUT_36);
			   SetOutput(O_IODEBUG_OUTPUT_38);
			   SetOutput(O_IODEBUG_OUTPUT_39);	  //蜂鸣器
			   ResetOutput(O_IODEBUG_OUTPUT_37); //黄灯
		   }
	   }
	}
}
/**************************************************************************************************
**  函数名：  Servo_Stop(u8)
**	输入参数：无
**	输出参数：无
**	函数功能：伺服器停止控制函数
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2014/2/10 
***************************************************************************************************/
void Servo_Stop(u8 Axis)
{
	 if(Servo_Stop_Done[Axis]==FALSE && Pulse_Counter[Axis] != 0)
	 {     
		  if(Pulse_ControlStep[Axis] == SPEED_UP)
	      {
			  Servo_Pulse_Count[Axis] =Pulse_Counter[Axis] + Pulse_Counter[Axis] + Present_Pulse_SpeedControlStep_Counter[Axis];	
		    }
		  else if(Pulse_ControlStep[Axis] == SPEED_HOLD)  
		  {					 
			  if(Servo_Pulse_Count[Axis] - Pulse_Counter[Axis] - SpeedUp_Pulse_Counter[Axis] > 2 ) //如果2个脉冲之后仍不是Speed_Down
			  {
			       Servo_Pulse_Count[Axis] = Pulse_Counter[Axis]+SpeedUp_Pulse_Counter[Axis]+2;
			  }		  
		  }
		  else if(Pulse_ControlStep[Axis] == SPEED_DOWN) 				    //如果正好在脉冲发送减速段，不做操作
		  {
		  }
		  else
		  {
		  	   switch(Axis)
			   {
			   		case X_Axsis:
						 TIM_Cmd(TIM2, DISABLE);
						 Timer_Enable_X = DISABLE;
					     break;
			   		case Z_Axsis:
						 TIM_Cmd(TIM3, DISABLE);
						 Timer_Enable_Z = DISABLE;
					     break;
			   		case L_Axsis:
						 TIM_Cmd(TIM1, DISABLE);
						 Timer_Enable_L = DISABLE;
						 ClosePulseReset(L_Axsis);
					     break;
			   		case O_Axsis:
						 TIM_Cmd(TIM4, DISABLE);
						 Timer_Enable_O = DISABLE;
					     break;
			   		case A_Axsis:
						 TIM_Cmd(TIM8, DISABLE);
						 Timer_Enable_A = DISABLE;
					     break;
			   		case B_Axsis:
						 TIM_Cmd(TIM5, DISABLE);
						 Timer_Enable_B = DISABLE;
					     break;
					default:
					     break;
			   }			  	  
		  }
		  Servo_Stop_Done[Axis]=TRUE;
	}
}
/**************************************************************************************************
**  函数名：  Robot_Reset()
**	输入参数：无
**	输出参数：无
**	函数功能：机器相关参数复位到初始值
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void Robot_Reset()
{
	Error_Status = 	NO_ERROR;
	Error_Data = 0;
	Robot_Error_Data[0]=0;
	Pulse_TotalCounter[0] = MINROBOTPOSITION;	
	Pulse_TotalCounter[1] = MINROBOTPOSITION;
	Pulse_TotalCounter[2] = MINROBOTPOSITION;
	Pulse_TotalCounter[3] = MINROBOTPOSITION;
	Pulse_TotalCounter[4] = MINROBOTPOSITION;
	Pulse_TotalCounter[5] = MINROBOTPOSITION;
	Pulse_Maxspeed_HalfPeriod[Z_Axsis] = 180;
	Pulse_Maxspeed_HalfPeriod[X_Axsis] = 180;
	Pulse_Maxspeed_HalfPeriod[L_Axsis] = 180;
	Pulse_Maxspeed_HalfPeriod[O_Axsis] = 180;		
	Pulse_Maxspeed_HalfPeriod[A_Axsis] = 180;
	Pulse_Maxspeed_HalfPeriod[B_Axsis] = 180;
	Axsis_Move_Direction[X_Axsis] = POSITIVE;
	Axsis_Move_Direction[Z_Axsis] = POSITIVE;
	Axsis_Move_Direction[L_Axsis] = POSITIVE;
	Axsis_Move_Direction[O_Axsis] = POSITIVE;
	Axsis_Move_Direction[A_Axsis] = POSITIVE;
	Axsis_Move_Direction[B_Axsis] = POSITIVE;
	Axsis_Chosen = X_Axsis;
	Work_Status = AUTO_WORK_MODE;
	Servo_Control_Mode = LINKEDRUNNINGMODE;
	Pulse_PresentAcceleration[Z_Axsis]=20;
	Pulse_PresentAcceleration[X_Axsis]=20;
	Pulse_PresentAcceleration[L_Axsis]=20;
	Pulse_PresentAcceleration[O_Axsis]=20;
	Pulse_PresentAcceleration[A_Axsis]=20;
	Pulse_PresentAcceleration[B_Axsis]=20;
	Action_Delay_Flag=FALSE;
	Action_Done_Flag=FALSE;
	Pulse_ControlStep[Z_Axsis]=SPEED_UP;
	Pulse_ControlStep[X_Axsis]=SPEED_UP;
	Pulse_ControlStep[L_Axsis]=SPEED_UP;
	Pulse_ControlStep[O_Axsis]=SPEED_UP;
	Pulse_ControlStep[A_Axsis]=SPEED_UP;
	Pulse_ControlStep[B_Axsis]=SPEED_UP;
	DIR485_L;	   //485默认读状态
	SetOutput(O_IODEBUG_OUTPUT_36);
	SetOutput(O_IODEBUG_OUTPUT_38);
	SetOutput(O_IODEBUG_OUTPUT_39);	  //蜂鸣器
	ResetOutput(O_IODEBUG_OUTPUT_37); //黄灯
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

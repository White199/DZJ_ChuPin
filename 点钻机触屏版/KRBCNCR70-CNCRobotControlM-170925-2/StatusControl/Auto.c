/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Usart.h" 
#include "Auto.h"
#include "StatusControl.h"
#include "SpeedControl.h"
#include "in.h"
#include "out.h"
#include "Manual.h"
#include "Parameter.h"
#include "Error.h"
#include "SignalWatch.h"
#include "Delay.h"
#include "w25qxx.h" 
#include "Auto_2.h"
#include "ActionOperate.h"

#include "SpeedControl_CAN.h"
#include "PDO.h"

u32 Auto_Pulse_Count = 0;        //自动模式每个动作发送脉冲个数

/**-- 自动模式下的各模式标志位 --**/
u8 Auto_Mode = 	LOOP_MODE;	     //自动模式选择
u8 Once_Mode_Enable = DISABLE;	 //单次模式使能
u8 Single_Mode_Enable = DISABLE; //单节模式使能
u8 Loop_Mode_Enable = DISABLE;	 //循环模式使能

u8 Action_Complete = FALSE;      //105机械手一次循环动作完成
u8 Section_Action_Complete=FALSE;//单节动作是否完成标志位
u8 Auto_Run_Enable = DISABLE;    //自动模式使能标志

u32 Total_Product_Number = 0;	 //总计加工数量
u32 Product_Number = 0 ;	 	 //产品数量
u32 Check_Number = 0;			 //产品检测数量
u8  Unload_Material_Finish=FALSE; //卸料结束

u8 Stop_Status = RUN ;			 //停止按钮按下去之后的机械手状态
u8 Stop_Time = 0;				 //按下停止次数
u8 JHRobot_Run_Time = 0;	     //机械臂自动运行动作循环次数

u32 JHRobot_Section_Time=0;
u8 JHRobot_Section = 1 ;	     //机械臂运行一个完整动作的哪一节（暂时定为一次循环10节）
u8 Last_JHRobot_Section=0;
u8 JHRobot_Section_Delay=FALSE;
u8 JHRobot_Warning=FALSE;		 //动作全程监视报警
u8 Unload_Material_Step=1;       //放料点的动作步骤
u8 Section_Step = 1;
u8 Step_Done=FALSE;				 //步骤是否操作标志位，用于每一步里的X.Z轴中
u8 Delay_Done=FALSE;			 //延时是否操作标志位，用于伺服器跟随延时
u8 Manchine_Door_Flag=FALSE;     //移门是否动作标志位
u8 Robot_Move_Flag=FALSE;		 //机械手是否需要移动才能到目标位置标志位


u32	X_Axsis_Target = MINROBOTPOSITION;	 //X轴目的坐标
u32 Z_Axsis_Target = MINROBOTPOSITION;	 //Z轴目的坐标
u32 Present_Position = MINROBOTPOSITION;	 //当前位置值
u32 Max_Robot_Position=MAXROBOTPOSITION;	 //机械手最大的坐标轴
u32 Min_Robot_Position=MINROBOTPOSITION;	 //机械手最小的坐标轴
u32 Action_Delay_Time=0;         //机械手每次动作完成之后的延时，因为伺服器动作是跟随动作
u8 	Puls_Delay_Time[50]={0};		  //脉冲信号延时
u8  Puls_Delay_Enable[50]={DISABLE};  //脉冲信号标注
u8  Puls_Delay_Num=0;			  //脉冲信号标注
u8  Action_Done_Flag =FALSE;		  //伺服器动作完成标志
u8  Action_Delay_Flag=FALSE;		  //动作延时完成
u8  Send_UMF_Signle=FALSE;			  //发生卸料完成脉冲信号标志位

//输出脉冲信号延时参数
u16 Output_Delay_Time=0;		      //输出脉冲延时


u32 Test_Pulse_Xaxsis=0;		  //测试X轴丢脉冲
u8  Test_Pulse_Xaxsis_Flag = FALSE;
u8  Auto_Action_Delay=DISABLE;	  //卸料结束最后次动作延时
/*------------------------------------------------------------
// 说明:取料点X位置从position2开始，放料点位置从position1开始 //
//      依次间隔一定距离为下一坐标点，取料position2~6 ；放置  //
//	    成品坐标为position1~5。原始状态：机床中有料       	  //
//      在手控器中设置时，以position1的坐标为取料点X坐标	  //
  ------------------------------------------------------------*/
/* 产品取原料和放成品位置 */
u8  Load_Material_Row_Number=6;    //取料点一排原料个数，默认为6
u32 Product_Position[10]={0};
u32 Interval_Position=5000;			 //5000:5cm
/* 取料和放成品的位置坐标数0~5 */
u8  Load_Material_Number=1;
u8  Place_Product_Number=0;

static u8 j =0;
u8 Read_IIC_Auto_Program[16]={0}; 
u8 Auto_Reset_Step = 0;
u8 Auto_Reset_Step_Done = FALSE;

u8 g_Robot_Has_Debuged_Flag=FALSE;	 //机械手之前状态是否处于自由编程调试状态
u8 g_Program_Is_Debuging=FALSE;
u32 Target_Position_Can;

//--------------------------//
u8 LCD_Control_Flag=0;//控制板控制LCD
u16 LCD_Control_Time=0;//控制板控制LCD计时

extern u8 Product_Num;
extern u16 Confirm_Order_Num,g_Delay_Timer;
extern u8 g_Delay_Timer_Flag;
extern u8 Axis_Manul_Speed[6];
/**************************************************************************************************
**  函数名：  RobotAutoReset()
**	输入参数：无
**	输出参数：无
**	函数功能：自动下的标志位清零
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
u8 REST_Flag=0;
extern u8 Axis_Step;
void RobotAutoReset()
{
	u8 i=0;
		 for(i=0;i<35;i++)		//指示灯不复位
		 {
			SetOutput(i);
		 }
		if(Axis_Step<6)
		{
			switch(JXS_Parameter.Origin[Axis_Step])
			{
				case 0x00:
					Axis_Step++;
					REST_Flag=0;
					break;
				case 0x20://X
					 if(Pulse_TotalCounter[X_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(X_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[X_Axsis]*100);
					 }
					break;
				case 0x10://Y
					 if(Pulse_TotalCounter[L_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						 Pulse_Maxspeed_HalfPeriod[L_Axsis] = AUTO_PULSE_MINHALF_PERIOD*100/JXS_Parameter.OriginSpeed[L_Axsis];
						 if(Pulse_Maxspeed_HalfPeriod[L_Axsis]>18000)//2018.03.27 解决小速度回零速度问题  lin
						 {
							Pulse_Maxspeed_HalfPeriod[L_Axsis]=18000;
						 }
						 if(Pulse_Maxspeed_HalfPeriod[L_Axsis]<180)//2018.03.27 解决小速度回零速度问题  lin
						 {
							Pulse_Maxspeed_HalfPeriod[L_Axsis]=180;
						 }	
						 
						 AXisMove(L_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[L_Axsis]);	
					 }						 
					break;
				case 0x08://Z
					 if(Pulse_TotalCounter[Z_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(Z_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[Z_Axsis]*100);	
					 }					
					 break;
				case 0x04://O
					 if(Pulse_TotalCounter[O_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(O_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[O_Axsis]*100);	
					 }					
					 break;
				case 0x02://A
					 if(Pulse_TotalCounter[A_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(A_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[A_Axsis]*100);	
					 }					
					 break;
				case 0x01://B
					 if(Pulse_TotalCounter[B_Axsis] == MINROBOTPOSITION)
					 {
						Axis_Step++;
						REST_Flag=0;
					 }
					 else if(0==REST_Flag)
					 {
						 REST_Flag=1;
						AXisMove(B_Axsis,MINROBOTPOSITION,JXS_Parameter.OriginSpeed[B_Axsis]*100);	
					 }
					 break;
			}
		}
		else
		{
////////------------------------到达目标位置后不调用ServoStop_CAN则程序无法正常运行？-------------------------------//////////
			 ServoStop_CAN(X_Axsis);
			 CAN_Servo_Stop_Action[X_Axsis]=FALSE;	
			 ServoStop_CAN(Z_Axsis);
			 CAN_Servo_Stop_Action[Z_Axsis]=FALSE;
			 ServoStop_CAN(L_Axsis);
			 CAN_Servo_Stop_Action[L_Axsis]=FALSE;
			 ServoStop_CAN(O_Axsis);
			 CAN_Servo_Stop_Action[O_Axsis]=FALSE;
			 ServoStop_CAN(A_Axsis);
			 CAN_Servo_Stop_Action[A_Axsis]=FALSE;
			 ServoStop_CAN(B_Axsis);
			 CAN_Servo_Stop_Action[B_Axsis]=FALSE;
////////------------------------到达目标位置后不调用ServoStop_CAN则程序无法正常运行？-------------------------------//////////
			
			Axis_Step=0;
			REST_Flag=0;			
			Auto_Reset_Step_Done = TRUE;			
			 if(g_Auto_LOrigin_Flag)
			 {
			 	g_Auto_LOrigin_Flag=FALSE;
			 }
			 else
			 {
				 //ResetOutput(O_IODEBUG_OUTPUT_6); 
				 g_AutoStatue = AUTO_WAITE;
			 }
			 Auto_Reset_Step=0;
			 Robot_Auto_Reset = TRUE;
			 g_Auto_Reset_Flag = FALSE;
		}	 	

}

/**************************************************************************************************
**  函数名：  AutoReset()
**	输入参数：无
**	输出参数：无
**	函数功能：自动下的标志位清零
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void AutoReset()
{
	  Stop_Status = STOP;
	  Back_Origin_Flag = FALSE;
	  Origin_Backed = FALSE;
	  Robot_Auto_Reset = FALSE;
	  Auto_Run_Enable = DISABLE;
	  Once_Mode_Enable = DISABLE;
	  Single_Mode_Enable = DISABLE;			  
	  Loop_Mode_Enable = DISABLE;
	  Jog_Move_Enable = DISABLE;
	  Linked_Move_Enable = DISABLE;
	  g_AutoStatue = AUTO_WAITE;
	  Work_Status = WAIT_MODE;
	  g_Auto_ActionError_Flag=FALSE;
	  g_Auto_ActionTimeOut_Flag=FALSE;
	  g_Auto_ActionRun_Timer=0;
	  g_Auto_ActionRun_Step=0;
	  g_Auto_PresentLine = 0;			//自动运行行号
	  Action_Step_List_Num=0;
	  Action_Step_Run_Num=0;
	  Action_Step_Confirm_Num=0;
	  g_USART_Delay_Timer=0;
	  g_LC_ActionRun_Timer=0;
	  g_LC_ActionRun_Step=0;
	  g_LC_PresentLine = 0;			//自动运行料仓行号
	  g_Read_LC_Load_Program=FALSE;
	  g_Read_LC_UnLoad_Program=FALSE;
	  g_LC_Step_Control=FALSE;
	  g_Start_Step_Control=FALSE;
    g_Program_Start_Action=FALSE;
	  g_Program_Reset_Action=FALSE;
}

/**************************************************************************************************
**  函数名：  Auto_Start()
**	输入参数：无
**	输出参数：无
**	函数功能：启动键按下
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2014/2/13 
***************************************************************************************************/
void AutoStart()
{
	 Auto_Run_Enable = ENABLE;
	 Once_Mode_Enable = ENABLE;
	 Single_Mode_Enable = ENABLE;
	 Loop_Mode_Enable = ENABLE;
	 Section_Action_Complete = FALSE;	 //单节动作暂停			 
	 Stop_Time = 0;
	 Stop_Status = RUN;
	 Error_Data = Error_Data & 0xfdff;	 //清除加工完成报警信号	
	 Watch_Data = (Watch_Data&0xbf) | 0x80 ;		//1000
}

/**************************************************************************************************
**  函数名：  AutoPause()
**	输入参数：无
**	输出参数：无
**	函数功能：暂停键按下
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2015/8/10 
***************************************************************************************************/
void AutoPause()
{
	Step_Done = FALSE;
	Stop_Status = PAUSE;
	Watch_Data = (Watch_Data&0x7f) | 0x40 ;
}

/**************************************************************************************************
**  函数名：  AutoStop()
**	输入参数：无
**	输出参数：无
**	函数功能：停止键按下
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2014/2/13 
***************************************************************************************************/
void AutoStop()
{
//	 Stop_Time++;			 
//	 //暂停
//	 if(Stop_Time == 1)
//	 {
//		 Step_Done = FALSE;
//		 Stop_Status = PAUSE;
//         Watch_Data = (Watch_Data&0x7f) | 0x40 ;					 							 
//	 }
//	 //停止
//	 if(Stop_Time == 2)
//	 {
	 	 AutoReset();
		 Watch_Data = Watch_Data & 0x3f ;	 				 
//	 } 
}

/**************************************************************************************************
**  函数名：  SafeAreaJudge()
**	输入参数：无
**	输出参数：无
**	函数功能：自动运行时安全区域判断
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/2/17
***************************************************************************************************/
void SafeAreaJudge()
{
 	u8 i=0;	
	//判断安全区是否启用
	for(i=0;i<3;i++)
	{
		if(Robot_Safe_Area[i].SafeArea_Switch)
		{
			SafeArea_Is_On=TRUE;
			break;
		}
		else
		{
			SafeArea_Is_On=FALSE;
		}
	}

//	//如果Z轴位于0点,则认为绝对安全
//	if(Pulse_TotalCounter[1]==MINROBOTPOSITION)
//	{
//		Safe_Area_Alarm=FALSE;
//	}
//	else
//	{
		//如安全区功能启用,机械轴完成回原点,对X轴位于安全区号进行判断
		if(SafeArea_Is_On==TRUE && Origin_Backed==TRUE)
		{
			for(i=0;i<3;i++)
			{
				if(Robot_Safe_Area[i].SafeArea_Switch)
				{
					if((Pulse_TotalCounter[0]>=Robot_Safe_Area[i].X_Left) && (Pulse_TotalCounter[0]<=Robot_Safe_Area[i].X_Right))
					{
						g_Current_SafeArea_Num=i+1;  //1-2-3当前X轴处于安全区域编号
						break;
					}
				}
				else
				{
					g_Current_SafeArea_Num=0;		 //否则不在安全区域内部
				}
			}
			//X轴位于某个安全区中时
			if(g_Current_SafeArea_Num)
			{	
			   //Z轴 大于安全区Z轴下位置	或 小于安全区Z轴上位置
			   if((Pulse_TotalCounter[1]>Robot_Safe_Area[g_Current_SafeArea_Num-1].Z_Down)||(Pulse_TotalCounter[1]<Robot_Safe_Area[g_Current_SafeArea_Num-1].Z_Up)) 
			   {
					  Safe_Area_Alarm=TRUE;
			   }
			   else
			   {
			   		Safe_Area_Alarm=FALSE;
			   }					
			}
			//X轴不位于任何安全区中时
			else  
			{
			   //Z轴 坐标在下方
			   if(Pulse_TotalCounter[1]>MINROBOTPOSITION) 
			   {
					  Safe_Area_Alarm=TRUE;
			   }
			   else
			   {
			   		Safe_Area_Alarm=FALSE;
			   }
			}
		} //if(SafeArea_Is_On==TRUE && Origin_Backed==TRUE)
		else 
		{
			g_Current_SafeArea_Num=0;
		}
//	}
	//超出安全区
	if(Safe_Area_Alarm)
	{
		Robot_Error_Data[0] = Robot_Error_Data[0] | 0x02;
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
		Robot_Error_Data[0] = Robot_Error_Data[0] & 0xfd;
	}
}

/**************************************************************************************************
**  函数名：  MoveToTarget()
**	输入参数：无
**	输出参数：无
**	函数功能：运动到目标位置
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void MoveToTarget(u32 Target_Position,u8 Axis,u32 Axsis_Speed)
{
//	 if((Axis == X_Axsis)&&(Timer_Enable_X == ENABLE))
//	 {
//	 }
//	 else if((Axis == Z_Axsis)&&(Timer_Enable_Z == ENABLE))
//	 {
//	 }
//	 else if((Axis == L_Axsis)&&(Timer_Enable_L == ENABLE))
//	 {
//	 }
//	 else if((Axis == O_Axsis)&&(Timer_Enable_O == ENABLE))
//	 {
//	 }
//	 else if((Axis == A_Axsis)&&(Timer_Enable_A == ENABLE))
//	 {
//	 }
//	 else if((Axis == B_Axsis)&&(Timer_Enable_B == ENABLE))
//	 {
//	 }
//	 else
//	 {		 
     //---2017.8.21
	 if(Axis==L_Axsis)
	 {
		 Present_Position = Pulse_TotalCounter[Axis];
		 if(Min_Robot_Position <= Target_Position && Target_Position <= Max_Robot_Position)
		 {
		 	 if(Target_Position > Present_Position)
			 {
			 	   Axsis_Move_Direction[Axis] = POSITIVE;
				   Auto_Pulse_Count = Target_Position - Present_Position ;
				   //Servo_Is_Run = TRUE;
				   SendPulse(Axis);
			 }
		 	 else if(Target_Position < Present_Position)
			 {
			 	   Axsis_Move_Direction[Axis] = NEGATIVE;
				   Auto_Pulse_Count = Present_Position - Target_Position ;
				   //Servo_Is_Run = TRUE;
				   SendPulse(Axis);		 
			 }
			 else
			 {
			 }
		 }		 
	 }
	 else
	 {
		 Target_Position_Can=Target_Position-1000000;		 				 
		 MoveToTarget_CAN(Axis,Target_Position_Can,Axsis_Speed);
	 }
//   }
}

/**************************************************************************************************
**  函数名：  SetSingle()
**	输入参数：无
**	输出参数：无
**	函数功能：信号输出，并置位标志位
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/3/10 
***************************************************************************************************/
void SetSingle(u8 Reset_Output,u8 Set_Output, u32 Detect_Flag)
{
	 //信号输出：复位 , 置位 , 脉冲信号
	 if(Reset_Output ==60)
	 { //单信号-置位某IO
	 }
	 else
	 {
	 	SetOutput(Reset_Output);					   //复位  当前信号置为1 对应变量位 1
	 	//Puls_Delay_Enable[Reset_Output]=DISABLE;	   //清除标志位
	 }
	 if(Set_Output ==60)
	 { //单信号-复位某IO
	 }
	 else
	 {
		 ResetOutput(Set_Output);						   //置位	当前信号置为0 对应变量位0
		 if(Detect_Flag==15)
		 {
		 	Puls_Delay_Num++;
			Puls_Delay_Enable[Set_Output]=ENABLE;	   //置位标志位
		 }
	 }
}

/**************************************************************************************************
**  函数名：  Read_SaveProgram_IIC_Address()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式下的各种操作
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2016/01/10 
***************************************************************************************************/
void Read_SaveProgram_IIC_Address()
{
	 u8 i=0;
	 SaveProgram_IIC_Address=0;
	 SaveProgram_IIC_Num=0;
	 if(g_Run_Program_Num>0)
	 {
		 if(g_Run_Program_Num>SAVEPROGRAMNUM_2)
		 {
		     SaveProgram_IIC_Address=Program_IIC_Address[g_Run_Program_Num-1].Address;
			 SaveProgram_IIC_Num = Program_IIC_Address[g_Run_Program_Num-1].Num;
		 }
		 else
		 {
			 for(i=0;i<SAVEPROGRAMNUM_2;i++)
			 {
			 	//查询到编号与当前设定自动运行编号相同数组
				if(Program_IIC_Address[i].Code == g_Run_Program_Num)
				{
				  SaveProgram_IIC_Address=Program_IIC_Address[i].Address;
					SaveProgram_IIC_Num=Program_IIC_Address[i].Num;
					break;
				}
			 }
		 }
	 } 
}

/**************************************************************************************************
**  函数名：  AutoRun()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式下的各种操作
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2015/08/10 
***************************************************************************************************/
void AutoRun()
{
	u8 k=0;
	switch(UsartReceiveData[1])
	{
		case P_CYCLE_MODE:
		     Auto_Mode = LOOP_MODE;
			 Loop_Mode_Enable = ENABLE;
			 break;	

		case P_SINGLE_MODE:		  //单步模式
		     Auto_Mode = SINGLE_MODE;
			 Single_Mode_Enable = ENABLE;
			 break;	

		case P_ACTION_RUN:
			 if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
			 {				 			
				g_Auto_Order_Start = TRUE;
				//AutoStart();
				if(Auto_Mode == SINGLE_MODE)
				{
					Single_Mode_Enable = ENABLE;
					Loop_Mode_Enable = DISABLE;
				}
				else if(Auto_Mode == LOOP_MODE)
				{
					Loop_Mode_Enable = ENABLE;
					Single_Mode_Enable = DISABLE;
				}
			 }
		     break;	

		case P_ACTION_PAUSE:
		     if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
			 {
				//AutoPause();
				g_Auto_Order_Pause = TRUE;
			 }
		     break;

		case P_ACTION_STOP:
		     if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
			 {
				 g_LCDelay_Step=0;
				 g_LCDelay_Timer=0;
				 
				 g_Delay_Timer_Flag=0;
				 g_Delay_Timer=0;				 
				 Confirm_Order_Num=0;
				 g_Auto_PresentLine=0;
				 Action_Step_Run_Num=0;		
				 g_Auto_Order_Stop = TRUE;
				 if(LCD_Control_Flag==1)
					LCD_Control_Flag=2;
			 }
		     break;

		case P_ACTION_PROGRAM:
			 g_Run_Program_Num =  UsartReceiveData[2];
			 //写入IIC
			 W25QXX_Write(&g_Run_Program_Num,0x40E0,1);	
			 Read_SaveProgram_IIC_Address();
			 if(SaveProgram_IIC_Address>0)
			 {
				 W25QXX_Read(Read_IIC_Auto_Program,SaveProgram_IIC_Address,7);
			 	 Free_Program_Operate.Flag  = Read_IIC_Auto_Program[0];
			 	 Free_Program_Operate.Code  = Read_IIC_Auto_Program[1];
				 Free_Program_Operate.Name  = (u32)( ((u32)Read_IIC_Auto_Program[2])|((u32)Read_IIC_Auto_Program[3]<<8)|((u32)Read_IIC_Auto_Program[4]<<16)|((u32)Read_IIC_Auto_Program[5]<<24) );
				 Free_Program_Operate.Num   = Read_IIC_Auto_Program[6];
				 for(j=0;j<Free_Program_Operate.Num;j++)
				 {
				 	W25QXX_Read(Read_IIC_Auto_Program,SaveProgram_IIC_Address+0x07+0x10*j,16);
					Free_Program_Operate.Program[j].Flag   = Read_IIC_Auto_Program[0];
					Free_Program_Operate.Program[j].List   = Read_IIC_Auto_Program[1];
				 	Free_Program_Operate.Program[j].Order  = Read_IIC_Auto_Program[2];
					Free_Program_Operate.Program[j].Key    = Read_IIC_Auto_Program[3];
					Free_Program_Operate.Program[j].Value1 = (u32)( ((u32)Read_IIC_Auto_Program[4])|((u32)Read_IIC_Auto_Program[5]<<8)|((u32)Read_IIC_Auto_Program[6]<<16)|((u32)Read_IIC_Auto_Program[7]<<24) );
				  Free_Program_Operate.Program[j].Value2 = (u32)( ((u32)Read_IIC_Auto_Program[8])|((u32)Read_IIC_Auto_Program[9]<<8)|((u32)Read_IIC_Auto_Program[10]<<16)|((u32)Read_IIC_Auto_Program[11]<<24) );	
					Free_Program_Operate.Program[j].Value3 = (u32)( ((u32)Read_IIC_Auto_Program[12])|((u32)Read_IIC_Auto_Program[13]<<8)|((u32)Read_IIC_Auto_Program[14]<<16)|((u32)Read_IIC_Auto_Program[15]<<24) ); 					
					Free_Program_Operate.Program[j].Value1 = Free_Program_Operate.Program[j].Value1 & 0x0fffffff;
				  Free_Program_Operate.Program[j].Value2 = Free_Program_Operate.Program[j].Value2 & 0x0fffffff;	
					Free_Program_Operate.Program[j].Value3 = Free_Program_Operate.Program[j].Value3 & 0x0fffffff;
					if(K_ZAXISPOSITION<=Free_Program_Operate.Program[j].Key && Free_Program_Operate.Program[j].Key<=K_OAXIS)	//轴命令
					{
						Free_Program_Operate.Program[j].Value1 = Free_Program_Operate.Program[j].Value1+MINROBOTPOSITION;
					}
				 }
				 for(k=j;k<LARGESTPROGRAMNUM;k++)
				 {
					Free_Program_Operate.Program[k].Flag   = 0;
					Free_Program_Operate.Program[k].List   = 0;
				 	Free_Program_Operate.Program[k].Order  = 0;
					Free_Program_Operate.Program[k].Key    = 0;
					Free_Program_Operate.Program[k].Value1 = 0;
				  Free_Program_Operate.Program[k].Value2 = 0;	
					Free_Program_Operate.Program[k].Value3 = 0; 								 			 
				 }
				 //读取料仓指令行
				 Read_LC_List(g_Run_Program_Num);				    
			 }
	     break;

		case P_ACTION_RESET:  //机械手复位	
			 Axis_Step=0;
			 Work_Status = AUTO_WORK_MODE;
			 g_Auto_Reset_Flag = TRUE;
			 Robot_Auto_Reset = FALSE;
			 g_Auto_LOrigin_Flag = FALSE;
		   break;

		case P_ACTION_DEBUG:  //机械手调试			 
			 Work_Status = AUTO_WORK_MODE;			 
			 Auto_Mode = SINGLE_MODE;
			 Single_Mode_Enable = ENABLE;
			 g_Auto_PresentLine = UsartReceiveData[2];	  //当前单步的行
			 g_Auto_ActionRun_Step=0;
			 Robot_Auto_Reset=TRUE;
			 //g_Auto_Reset_Flag=FALSE;
			 g_Auto_Order_Start = TRUE;
			 g_Robot_Has_Debuged_Flag=TRUE;
			 g_Program_Is_Debuging=TRUE;
		   break;

		case P_ACTION_LORIGIN:  //机械手L回零			 
			 g_Auto_LOrigin_Flag = TRUE;
			 g_Auto_Reset_Flag = TRUE;
			 Robot_Auto_Reset = FALSE;
		   break;

		default:
		   break;
	}
}

/**************************************************************************************************
**  函数名：  AutoMode()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式下的各种模式选择
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void AutoMode()
{
	 switch(UsartReceiveData[1])
	 {
	    case ONCE_MODE:
		     Auto_Mode = ONCE_MODE;
			   UsartAcknowledge(AUTO_MODE);
         break;
		
			case SINGLE_MODE:
				 Auto_Mode = SINGLE_MODE;
				 UsartAcknowledge(AUTO_MODE);
				 break;
		
			case LOOP_MODE:
				 Auto_Mode = LOOP_MODE;
				 UsartAcknowledge(AUTO_MODE);
				 break;
			 		
			case AUTO_START:
				 if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
				 {				 
					AutoStart();
				 }	 
				 UsartAcknowledge(AUTO_MODE);
				 break;

			/*- 按一下是暂停，按两下是停止 -*/
			case AUTO_STOP:
				 if(Work_Status == AUTO_WORK_MODE && Back_Origin_Flag == FALSE)
				 {
					 AutoStop();
				 }
				 if(Work_Status == MANUAL_WORK_MODE)
				 {
					 Jog_Pause_Enable=ENABLE;
				 }
				 UsartAcknowledge(AUTO_MODE);
				 break;

 	   default:
		     break;	 
	 }
}

/**************************************************************************************************
**  函数名：  AutoRunning()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式下的各模式运行
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void AutoRunning()
{
	switch(Auto_Mode)
	{
		case ONCE_MODE:		//单次模式	              				 
//		   if(Once_Mode_Enable == ENABLE)
//			 {   
//				 //单节动作完成之后才能完成下一节动作  				 		 	
//			 }
			 break;
		
		case SINGLE_MODE:	//单步模式
		   if(Single_Mode_Enable == ENABLE)
			 {				
		 	   	AutoModeControl();				 
			 }
			 break;
		
		case LOOP_MODE:		//循环模式
		   if(Loop_Mode_Enable == ENABLE)
			 {
				  AutoModeControl(); 
			 }
			 break;

 		default:
		     break;	
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

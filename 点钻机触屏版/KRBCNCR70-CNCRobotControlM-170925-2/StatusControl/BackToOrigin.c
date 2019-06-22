#include "stm32f10x_lib.h"
#include "Usart.h" 
#include "BackToOrigin.h"
#include "StatusControl.h"
#include "SpeedControl.h"
#include "in.h"
#include "Auto.h"
#include "SignalWatch.h"
#include "Parameter.h"
#include "Homing.h"
#include "PDO.h"
#include "CANopen.h"
#include "Manual.h"
#include "SDO.h"


u8 Axis_Step=0;
u32 Origin_Pulse = 0;	        //过原点之后再次低速回原点的脉冲值
u8  Origin_Step = 0;	        //回原点步骤
u8  Origin_Time =FALSE;       //事件处理标志位
u8  Origin_Single_Flag=FALSE; //回原点信号标志位
u32 Origin_Speed[6] = {5,5,5,5,5,5};		      //回原点速度，默认10%
u8  Robot_Enable = FALSE;
u8  Homing_Flag_Can=FALSE;    //---2017.8.30


//------------------------------------//
extern u16 Read_Point_Num,ShuZu_Count;
extern u8 Order_Read_Mode,A_List[6];

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能： 
**	备注：	  无
**  作者：    吴祥     
**  开发日期： 
***************************************************************************************************/
void RobotEnable()
{
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能： 
**	备注：	  无
**  作者：    吴祥     
**  开发日期： 
***************************************************************************************************/
void RobotDisable()
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}

/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能： 
**	备注：	  无
**  作者：    吴祥     
**  开发日期： 
***************************************************************************************************/
void RobotEnableOrigin()
{
	switch(UsartReceiveData[1])
	{
		case P_ROBOT_ORIGIN:
		     //Robot_Enable = TRUE;	//debug
			 if(Robot_Enable)  //机械手已使能-允许回原点
			 {
			  	Back_Origin_Flag = TRUE; //机械手回原点标志位使能
				  X_Origin_Backed=FALSE;
				  Z_Origin_Backed=FALSE;
				  L_Origin_Backed=FALSE;
				  O_Origin_Backed=FALSE;
				  Origin_Backed=FALSE;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
				  Origin_Time = FALSE;
				  Origin_Step = 0;
				  //---2017.8.30
				  Homing_Flag_Can=TRUE;
				  //---2017.9.27
			 }
			 break;	

		case P_X_AXSIS_ORIGIN:
		   break;	

		case P_Z_AXSIS_ORIGIN:
		   break;	

		case P_ORIGIN_SPEED:
		   Origin_Speed[0] = (u32)( ((u32)UsartReceiveData[2]) | ((u32)UsartReceiveData[3]<<8) | ((u32)UsartReceiveData[4]<<16) | ((u32)UsartReceiveData[5]<<24) );
			 break;

		case P_ROBOT_ENABLE:
		   if(UsartReceiveData[2])
			 {
			 	RobotEnable();
				Robot_Enable = TRUE;
			 }
			 else
			 {
			 	RobotDisable();
				Robot_Enable = FALSE;
				Origin_Backed = FALSE;			   //清除已回原点标志			
			 }
			 break;

		default:
		   break;
	}
}
/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能： 
**	备注：	  无
**  作者：    吴祥     
**  开发日期： 
***************************************************************************************************/
void LAxis_BackToOrigin()
{
	switch(Origin_Step)
	{
		  /**************** X轴回原点*****************/		 
			case 0:	
				 /*---- 是否在原点位置，在原点位置则直接正向运动，不在则反向运动 ----*/
				 ReadInput(I_DETECT_L_ORIGIN);	 //X41
				 if(((Input_Detect_Status[2]&0x04)==0x04) && (Origin_Time == FALSE))	 //检测到零点开关信号
				 {	 
					 Origin_Step=Origin_Step+2;
					 Origin_Time = TRUE;
				 }
				 else
				 {
				 	 Servo_Control_Mode = BACKTOORIGIN;
					 if(JXS_Parameter.OriginDir[L_Axsis]==0)
						Axsis_Move_Direction[L_Axsis] = NEGATIVE;
					 else
						Axsis_Move_Direction[L_Axsis] = POSITIVE;
					 Axsis_Chosen=L_Axsis;
					 Pulse_Maxspeed_HalfPeriod[L_Axsis] = AUTO_PULSE_MINHALF_PERIOD*100/JXS_Parameter.OriginSpeed[L_Axsis];
					 if(Pulse_Maxspeed_HalfPeriod[L_Axsis]>18000)//2018.03.27 解决小速度回零速度问题  lin
					 {
						Pulse_Maxspeed_HalfPeriod[L_Axsis]=18000;
					 }
					 if(Pulse_Maxspeed_HalfPeriod[L_Axsis]<180)//2018.03.27 解决小速度回零速度问题  lin
					 {
						Pulse_Maxspeed_HalfPeriod[L_Axsis]=180;
					 }
					 SendPulse(Axsis_Chosen);								                  
					 Origin_Step++;
				 }
				 break;

			case 1:
			   /*---- 第一次检测到原点信号 ----*/
				 ReadInput(I_DETECT_L_ORIGIN);	 //X41
				 if(((Input_Detect_Status[2]&0x04)==0x04) && (Origin_Time == FALSE))	 //检测到零点开关信号
				 {
// 					  Present_SpeedUp_Jerk[L_Axsis]=SpeedUp_Jerk[2];
// 					  Present_SpeedUp_Max_Acceleration[L_Axsis]=SpeedUp_Max_Acceleration[2];
					  if(Pulse_ControlStep[L_Axsis] == SPEED_UP)
				    {
						   if(Pulse_Counter[L_Axsis]>=1000)
						   {
						  	  Servo_Pulse_Count[L_Axsis] =Pulse_Counter[L_Axsis]+1000;
						   }
						   else
						   {
						      Servo_Pulse_Count[L_Axsis] =Pulse_Counter[L_Axsis]+Pulse_Counter[L_Axsis];	
						   }
					  }
					  else  
					  {					 						  
						    if(SpeedUp_Pulse_Counter[L_Axsis]>=1000)
					 	    {
								   Servo_Pulse_Count[L_Axsis]=Pulse_Counter[L_Axsis]+1000;
						    }
							else
							{
								   Servo_Pulse_Count[L_Axsis]=Pulse_Counter[L_Axsis]+SpeedUp_Pulse_Counter[L_Axsis];
							}
					  }		
					  Pulse_ControlStep[L_Axsis]=SPEED_DOWN;		  						 
					  Origin_Time = TRUE;
				 }				
				 if(Timer_Enable_L == DISABLE)			 //检测到原点之后过冲并且停下来了
				 {
					  Origin_Step++;					 
				 }
				 break;

			case 2:
			 	/*---- 正向运动低速回原点 ----*/
				 Action_Done_Flag=TRUE;
				 if(Action_Delay_Flag==TRUE)
				 {
					  Action_Delay_Flag=FALSE;
					  Action_Done_Flag=FALSE;				 
					 if(JXS_Parameter.OriginDir[L_Axsis]==0)
						Axsis_Move_Direction[L_Axsis] = POSITIVE;
					 else
						Axsis_Move_Direction[L_Axsis] = NEGATIVE;
					 Axsis_Chosen=L_Axsis;
					  //Servo_Is_Run = TRUE;					 
					  Pulse_Maxspeed_HalfPeriod[L_Axsis] = 18000;//1% //以很低的速度回原点
					  SendPulse(Axsis_Chosen);
					  Origin_Time = FALSE;
					  Origin_Step++;
				 }
				 break;

            case 3:
				 /*---- 再次检测到原点信号----*/
				 ReadInput(I_DETECT_L_ORIGIN);	 //X41
				 if(Input_Detect_Status[2]&0x04)	 //检测到零点开关信号 X41
				 {
					  Origin_Step++;
				 }
				 break;
				 	
			case 4:
				 /*---- 原点信号消失时到达原点----*/			 
				 ReadInput(I_DETECT_L_ORIGIN);
				 if( ((Input_Detect_Status[2]| 0xfb)==0xfb) && (Origin_Time == FALSE) )	 //丢失零点开关信号 X42
				 {
				    if(Pulse_ControlStep[L_Axsis] == SPEED_UP)
				    {
					     Servo_Pulse_Count[L_Axsis] =Pulse_Counter[L_Axsis] + Pulse_Counter[L_Axsis] +5;	
				    }
				    else  
				    {					 
					     Servo_Pulse_Count[L_Axsis] = Pulse_Counter[L_Axsis]+SpeedUp_Pulse_Counter[L_Axsis]+5;
				    }								 
				    Origin_Time = TRUE;
				 }
				 if(Timer_Enable_L == DISABLE)			 //检测到原点之后过冲并且停下来了
				 {
					  Axis_Step++;
					  Origin_Step=0;
					  Origin_Time = FALSE;
					  L_Origin_Backed = TRUE ;	
				 }
				 break;		    			  
		}
}
/**************************************************************************************************
**  函数名：   
**	输入参数：无
**	输出参数：无
**	函数功能： 
**	备注：	  无
**  作者：     
**  开发日期： 
***************************************************************************************************/
void BackToOrigin()
{
	u8 i=0;
	if(Homing_Flag_Can==TRUE)
	{
		Homing_attained_status[X_Axsis]=0;
		Homing_attained_status[Z_Axsis]=0;
		Homing_attained_status[L_Axsis]=0;
		Homing_attained_status[O_Axsis]=0;
		Homing_attained_status[A_Axsis]=0;
		Homing_attained_status[B_Axsis]=0;
		Flag_ARM_Homing_x=0;			
		Flag_ARM_Homing_z=0;			
		Flag_ARM_Homing_l=0;			
		Flag_ARM_Homing_o=0;			
		Flag_ARM_Homing_a=0;			
		Flag_ARM_Homing_b=0;		
		Axis_Step=0;
		Homing_Mode_Init();
		Delay(100);
		Homing_Flag_Can=FALSE;
	}
	if(Back_Origin_Flag == TRUE)
	{
		if(Axis_Step<6)
		{
			switch(JXS_Parameter.Origin[Axis_Step])
			{
				case 0x00:
					Axis_Step++;
					break;
				case 0x10://Y
//					ARM_Homing(L_Axsis);
					LAxis_BackToOrigin();				
					break;
				case 0x20://X
					ARM_Homing(X_Axsis);
					break;
				case 0x08://Z
					ARM_Homing(Z_Axsis);
					break;
				case 0x04://O
					ARM_Homing(O_Axsis);
					break;
				case 0x02://A
					ARM_Homing(A_Axsis);
					break;
				case 0x01://B
					ARM_Homing(B_Axsis);
					break;
			}
		}
		else
		{
			Origin_Backed=TRUE;
		}
		if(Origin_Backed == TRUE)
		{
			PulseInit_Auto();					
			for(i=0;i<6;i++)
				A_List[i]=0;
			 Back_Origin_Flag = FALSE;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
			 Origin_Time = FALSE;        //事件处理标志位 
			 Origin_Step = 0;            //回原点步骤
			 Robot_Reset();
			 Read_Point_Num=0;
			 Order_Read_Mode=0;
			 ShuZu_Count=0;
			 Order_Package();
			 PP_Mode_Init();                                    //---2017.8.15回零完成后初始化成位置模式
		}	
    }	
}

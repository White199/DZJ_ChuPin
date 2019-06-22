/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Auto.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "Usart.h" 
#include "Manual.h"
#include "out.h"
#include "StatusControl.h"
#include "SpeedControl.h"
#include "Parameter.h"
#include "SignalWatch.h"
#include "w25qxx.h" 
#include "auto_2.h"

#include "SpeedControl_CAN.h"
//--
#include "PDO.h"
#include "ActionOperate.h"

u8 X_Axsis_Chosen = FALSE;			  //X轴选中标志位
u8 Z_Axsis_Chosen = FALSE;			  //Z轴选中标志位
u8 Manual_Mode_Enable = FALSE;	      //手动模式
u8 Jog_Mode_Enable = DISABLE;		  //寸动模式使能标志位
u8 Linked_Mode_Enable = DISABLE;	      //连动模式使能标志位

u8 Manual_Mode_Error_Flag = FALSE;	  //手动操作模式错误标志位
u8 Jog_Move_Enable = DISABLE;			  //寸动动作使能位
u8 Linked_Move_Enable = DISABLE	;	  //连动动作使能位
u8 g_Link_Move_Delay = FALSE;
u8 g_Link_Move_Count = 0;

u8 Send_Material_Finish_Flag = FALSE;     //送料完成标志位
u8 Load_Material_Finish_Flag = FALSE;	  //取料结束标志位
u8 Unload_Material_Finish_Flag = FALSE;	  //卸料结束标志位
u32 Send_Material_Finish_Time = 0;        //送料定时参数
u32 Load_Material_Finish_Time = 0;		  //取料定时参数
u32 Unload_Material_Finish_Time = 0;	  //卸料定时参数

u32 Jog_Pulse_Count = 50;			  //寸动模式的每个动作的脉冲数
u32 Jog_Pulse_Count_Init=50;		  //初始寸动距离
u32 Linked_Pulse_Count = MINROBOTPOSITION;	  //连动模式的每次发送脉冲数
u32 Linked_Pulse=MINROBOTPOSITION;
u8  XAxis_Manul_Speed = 10;			  //X轴手动速度5-100,默认50
u8  ZAxis_Manul_Speed = 10;			  //Z轴手动速度5-100,默认50
u8  LAxis_Manul_Speed = 10;			  //L轴手动速度5-100,默认50
u8  OAxis_Manul_Speed = 10;			  //O轴手动速度5-100,默认50
u8  AAxis_Manul_Speed = 10;			  //L轴手动速度5-100,默认50
u8  BAxis_Manul_Speed = 10;			  //O轴手动速度5-100,默认50
u8  g_Manul_Fine_Adjust=FALSE;		  //手动微调
//u8  Manual_Event_Flag = FALSE;
//----2017.8.1
u8  Axis_Manul_Speed[6] = {10,10,10,10,10,10};			  //X轴手动速度5-100,默认50

u8 Test_Key_Flag=0;
static u8 i=0;
static u8 j=0;
static u8 Temp_IIC_XZPoint[10]={0};		 //删除保存点的IIC存取缓存
static u8 Temp_IIC_LOPoint[10]={0};		 //删除保存点的IIC存取缓存
u32 Temp_IIC[6]={0};
u16 SavePoint_IIC_Address = 0; //记录当前点要保存的地址
//--
vu32 Real_Time_Position_1[6]={0};                  //--2017.8实时目标位置
u32 Real_Time_Position[6]={0};
u8 first_press=1;
u32 Axsis_Maxlength;//---各轴右移最大长度，为了区别发脉冲的形式。
u32 Axsis_Minlength;

//-----------------------------------//
u16 Read_Point_Num=0,ShuZu_Count=0;
u8 Order_Read_Mode=0;
u8  XAxis_Step_Distance = 10;		  //X轴寸动距离5-100,默认50mm
u8  ZAxis_Step_Distance = 30;		  //Z轴寸动距离5-100,默认50mm
u8  LAxis_Step_Distance = 10;		  //L轴寸动距离5-100,默认50mm
u8  OAxis_Step_Distance = 10;		  //O轴寸动距离5-100,默认50mm
u8  AAxis_Step_Distance = 10;		  //L轴寸动距离5-100,默认50mm
u8  BAxis_Step_Distance = 10;		  //O轴寸动距离5-100,默认50mm
u8 Speed_Change_Flag=FALSE;          //摇杆速度改变标志位
vu32 Pre_Real_Time_Position_1[6]={0};                  //--2017.8实时目标位置



extern u8 Read_IIC_Auto_Program[];
extern u8 LC_ID;
extern u8 Sheet_Position[6];
extern u8 Speed_Up_Flag[6],Speed_Down_Flag[6];


/**************************************************************************************************
**  函数名：  ManualJogRunnig()
**	输入参数：无
**	输出参数：无
**	函数功能：手动模式->寸动模式
**	备注：	  方向键按下一次，轴运动一次
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void ManualJogRunnig()
{	 
	  if(Jog_Move_Enable == ENABLE)
	  {	
			   if(Axsis_Chosen == X_Axsis)
			   {

				    Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = 18000; //1% //PULSE_MINHALF_PERIOD*100*10/XAxis_Manul_Speed;			   
					if(XAxis_Step_Distance == 0)
			     	{
					     Jog_Pulse_Count =  Jog_Pulse_Count_Init;
				    }
					else
					{
					     Jog_Pulse_Count = XAxis_Step_Distance;  //Jog_Pulse_Count_Init*XAxis_Manul_Speed/100;
					} 
				  //防止寸动超过0点坐标
					if(((Pulse_TotalCounter[0]-Jog_Pulse_Count)<=MINROBOTPOSITION)&&(Axsis_Move_Direction[X_Axsis]  == NEGATIVE))
					{
						 Jog_Pulse_Count=Pulse_TotalCounter[0]-MINROBOTPOSITION;
					}
					//防止寸动超过最大软限位
					if(((Pulse_TotalCounter[0]+Jog_Pulse_Count)>=X_Axsis_Maxlength)&&(Axsis_Move_Direction[X_Axsis] == POSITIVE))
					{
						 Jog_Pulse_Count=X_Axsis_Maxlength-Pulse_TotalCounter[0];
					}
			   }	  
		  	   else if(Axsis_Chosen == Z_Axsis)
		       {

								Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = 18000; //1% //PULSE_MINHALF_PERIOD*100*10/ZAxis_Manul_Speed;	//会超出65536		   
								if(ZAxis_Step_Distance ==0)
								{
										Jog_Pulse_Count =  Jog_Pulse_Count_Init;
								}
							else
							{
									 Jog_Pulse_Count = ZAxis_Step_Distance;  //Jog_Pulse_Count_Init*ZAxis_Manul_Speed/100;
							}
								//防止寸动超过0点坐标
							if(((Pulse_TotalCounter[1]-Jog_Pulse_Count)<=MINROBOTPOSITION)&&(Axsis_Move_Direction[Z_Axsis] == NEGATIVE))
							{
								 Jog_Pulse_Count=Pulse_TotalCounter[1]-MINROBOTPOSITION;
							}
							 //防止寸动超过最大软限位
							if(((Pulse_TotalCounter[1]+Jog_Pulse_Count)>=Z_Axsis_Maxlength)&&(Axsis_Move_Direction[Z_Axsis] == POSITIVE))
							{
								 Jog_Pulse_Count=Z_Axsis_Maxlength-Pulse_TotalCounter[1];
							}
		       }
		  	   else if(Axsis_Chosen == L_Axsis)
		       {

				    Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = 18000; //1% //PULSE_MINHALF_PERIOD*100*10/ZAxis_Manul_Speed;	//会超出65536		   
		            if(LAxis_Step_Distance ==0)
			     	{
					    Jog_Pulse_Count =  Jog_Pulse_Count_Init;
				    }
					else
					{
					    Jog_Pulse_Count = LAxis_Step_Distance;  //Jog_Pulse_Count_Init*ZAxis_Manul_Speed/100;
					}
				    //防止寸动超过0点坐标
					if(((Pulse_TotalCounter[2]-Jog_Pulse_Count)<=MINROBOTPOSITION)&&(Axsis_Move_Direction[L_Axsis] == NEGATIVE))
					{
						 Jog_Pulse_Count=Pulse_TotalCounter[2]-MINROBOTPOSITION;
					}
				   //防止寸动超过最大软限位
					if(((Pulse_TotalCounter[2]+Jog_Pulse_Count)>=L_Axsis_Maxlength)&&(Axsis_Move_Direction[L_Axsis] == POSITIVE))
					{
						 Jog_Pulse_Count=L_Axsis_Maxlength-Pulse_TotalCounter[2];
					}
		       }
		  	   else if(Axsis_Chosen == O_Axsis)
		       {
				    Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = 18000; //1% //PULSE_MINHALF_PERIOD*100*10/ZAxis_Manul_Speed;	//会超出65536		   
		            if(OAxis_Step_Distance ==0)
			     	{
					    Jog_Pulse_Count =  Jog_Pulse_Count_Init;
				    }
					else
					{
					    Jog_Pulse_Count = OAxis_Step_Distance;  //Jog_Pulse_Count_Init*ZAxis_Manul_Speed/100;
					}
				    //防止寸动超过0点坐标
					if(((Pulse_TotalCounter[3]-Jog_Pulse_Count)<=MINROBOTPOSITION)&&(Axsis_Move_Direction[O_Axsis] == NEGATIVE))
					{
						 Jog_Pulse_Count=Pulse_TotalCounter[3]-MINROBOTPOSITION;
					}
				   //防止寸动超过最大软限位
					if(((Pulse_TotalCounter[3]+Jog_Pulse_Count)>=O_Axsis_Maxlength)&&(Axsis_Move_Direction[O_Axsis] == POSITIVE))
					{
						 Jog_Pulse_Count=O_Axsis_Maxlength-Pulse_TotalCounter[3];
					}
		       }
//			   if(g_Manul_Fine_Adjust)	 //处于手动微调状态-速度相对固定
//			   {
//			   	   Pulse_Maxspeed_HalfPeriod=18000; //1%  PULSE_MINHALF_PERIOD*100*10;
//			   }
//		   }
		   /*-- 机械手安全区域检测 --*/
	       SafeAreaDetection();	
		   if(Safe_Area_Flag==FALSE)
		   {   		   	   	  
				    Real_Time_Position[Axsis_Chosen]=Real_Time_Position_1[Axsis_Chosen];//---读取中断发来的位置信息
				 
						if(Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
						{									
							 Real_Time_Position[Axsis_Chosen]=Real_Time_Position[Axsis_Chosen]+Jog_Pulse_Count;
							AXisMove(Axsis_Chosen,Real_Time_Position[Axsis_Chosen]+1000000,0x03e8);														 
						}
						else if(Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE) 
						{
							 Real_Time_Position[Axsis_Chosen]=Real_Time_Position[Axsis_Chosen]-Jog_Pulse_Count;
							 AXisMove(Axsis_Chosen,Real_Time_Position[Axsis_Chosen]+1000000,0x03e8);
						}		
		   }		
	      Jog_Move_Enable = DISABLE;	
	  }
}

/**************************************************************************************************
**  函数名：  ManualLinkedRunning()
**	输入参数：无
**	输出参数：无
**	函数功能：自动模式->连动模式
**	备注：	  方向键按下，进入连动模式，再次按下停止运动
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void ManualLinkedRunning()
{
	  //手动回零完成之后
	  if(Axis_Manul_BackToOrigin==TRUE && Axsis_Chosen == X_Axsis && Pulse_TotalCounter[0]==MINROBOTPOSITION)
	  {
			 Linked_Move_Enable = DISABLE;
			 Axis_Manul_BackToOrigin=FALSE;				  
	  }
	  else if(Axis_Manul_BackToOrigin==TRUE && Axsis_Chosen == Z_Axsis && Pulse_TotalCounter[1]==MINROBOTPOSITION)
	  {
			 Linked_Move_Enable = DISABLE;	
			 Axis_Manul_BackToOrigin=FALSE;			  
	  }
	  else if(Axis_Manul_BackToOrigin==TRUE && Axsis_Chosen == L_Axsis && Pulse_TotalCounter[2]==MINROBOTPOSITION)
	  {
			 Linked_Move_Enable = DISABLE;	
			 Axis_Manul_BackToOrigin=FALSE;			  
	  }
	  else if(Axis_Manul_BackToOrigin==TRUE && Axsis_Chosen == O_Axsis && Pulse_TotalCounter[3]==MINROBOTPOSITION)
	  {
			 Linked_Move_Enable = DISABLE;	
			 Axis_Manul_BackToOrigin=FALSE;			  
	  }
	  else if(Axis_Manul_BackToOrigin==TRUE && Axsis_Chosen == A_Axsis && Pulse_TotalCounter[4]==MINROBOTPOSITION)
	  {
			 Linked_Move_Enable = DISABLE;	
			 Axis_Manul_BackToOrigin=FALSE;			  
	  }
	  else if(Axis_Manul_BackToOrigin==TRUE && Axsis_Chosen == B_Axsis && Pulse_TotalCounter[5]==MINROBOTPOSITION)
	  {
			 Linked_Move_Enable = DISABLE;	
			 Axis_Manul_BackToOrigin=FALSE;			  
	  }
    if(Linked_Move_Enable == ENABLE )
	{		 
		if(Speed_Change_Flag)
		{			
			  Speed_Change_Flag = FALSE;
			  if((Axsis_Chosen == X_Axsis)&&(Axsis_Move_Direction[X_Axsis] == NEGATIVE)&&(Pulse_TotalCounter[0]==X_Axsis_Minlength))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else if((Axsis_Chosen == X_Axsis)&&(Axsis_Move_Direction[X_Axsis] == POSITIVE)&&(Pulse_TotalCounter[0]==MAXROBOTPOSITION))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else if((Axsis_Chosen == Z_Axsis)&&(Axsis_Move_Direction[Z_Axsis] == NEGATIVE)&&(Pulse_TotalCounter[1]==Z_Axsis_Minlength))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }  
			  else if((Axsis_Chosen == Z_Axsis)&&(Axsis_Move_Direction[Z_Axsis] == POSITIVE)&&(Pulse_TotalCounter[1]==MAXROBOTPOSITION))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else if((Axsis_Chosen == L_Axsis)&&(Axsis_Move_Direction[L_Axsis] == NEGATIVE)&&(Pulse_TotalCounter[2]==L_Axsis_Minlength))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else if((Axsis_Chosen == L_Axsis)&&(Axsis_Move_Direction[L_Axsis] == POSITIVE)&&(Pulse_TotalCounter[2]==MAXROBOTPOSITION))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else if((Axsis_Chosen == O_Axsis)&&(Axsis_Move_Direction[O_Axsis] == NEGATIVE)&&(Pulse_TotalCounter[3]==O_Axsis_Minlength))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else if((Axsis_Chosen == O_Axsis)&&(Axsis_Move_Direction[O_Axsis] == POSITIVE)&&(Pulse_TotalCounter[3]==MAXROBOTPOSITION))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else if((Axsis_Chosen == A_Axsis)&&(Axsis_Move_Direction[A_Axsis] == NEGATIVE)&&(Pulse_TotalCounter[4]==A_Axsis_Minlength))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else if((Axsis_Chosen == A_Axsis)&&(Axsis_Move_Direction[A_Axsis] == POSITIVE)&&(Pulse_TotalCounter[4]==MAXROBOTPOSITION))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else if((Axsis_Chosen == B_Axsis)&&(Axsis_Move_Direction[B_Axsis] == NEGATIVE)&&(Pulse_TotalCounter[5]==B_Axsis_Minlength))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else if((Axsis_Chosen == B_Axsis)&&(Axsis_Move_Direction[B_Axsis] == POSITIVE)&&(Pulse_TotalCounter[5]==MAXROBOTPOSITION))
			  {
					Linked_Move_Enable=DISABLE;
					Pulse_Counter[Axsis_Chosen]=0;			  
			  }
			  else
			  {
				    Linked_Pulse=MINROBOTPOSITION;
				    if(Axsis_Chosen == X_Axsis)
				    {			  	   
						 if(XAxis_Manul_Speed ==0 )
						 {
							Pulse_Maxspeed_HalfPeriod[Axsis_Chosen]=PULSE_MINHALF_PERIOD*100;
						 }
						 else
						 {
							Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100/XAxis_Manul_Speed;
						 }
						 //防止连动超过0点坐标
						 if(Axsis_Move_Direction[X_Axsis] == NEGATIVE)
						 {
								Linked_Pulse=Pulse_TotalCounter[0]-MINROBOTPOSITION;
						 }
						 //防止连动超过最大软件限位
						 else if(Axsis_Move_Direction[X_Axsis] == POSITIVE)
						 {
							  Linked_Pulse=MAXROBOTPOSITION-Pulse_TotalCounter[0];
						 }
						 else
						 {
						 }	
				   }
				   else if(Axsis_Chosen == Z_Axsis)
				   {
						 if(ZAxis_Manul_Speed == 0 )
						 {
								Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100;
						 }
						 else
						 {
								Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100/ZAxis_Manul_Speed;				   
						 }
						 //防止连动超过0点坐标
						 if(Axsis_Move_Direction[Z_Axsis] == NEGATIVE)
						 {
								Linked_Pulse=Pulse_TotalCounter[1]-MINROBOTPOSITION;
						 }
						 //防止连动超过最大软件限位
						 else if(Axsis_Move_Direction[Z_Axsis] == POSITIVE)
						 {
								Linked_Pulse=MAXROBOTPOSITION-Pulse_TotalCounter[1];
						 }
						 else
						 {
						 }				   	
				   }	
				   else if(Axsis_Chosen == L_Axsis)
				   {
						 if(LAxis_Manul_Speed == 0 )
						 {
							Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100;
						 }
						 else
						 {
							Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100/LAxis_Manul_Speed;				   
						 }
						 //防止连动超过0点坐标
						 if(Axsis_Move_Direction[L_Axsis] == NEGATIVE)
						 {
							 Linked_Pulse=Pulse_TotalCounter[2]-MINROBOTPOSITION;
						 }
						 //防止连动超过最大软件限位
						 else if(Axsis_Move_Direction[L_Axsis] == POSITIVE)
						 {
							 Linked_Pulse=MAXROBOTPOSITION-Pulse_TotalCounter[2];
						 }
						 else
						 {
						 }				   	
				   }
				   else if(Axsis_Chosen == O_Axsis)
				   {
						 if(OAxis_Manul_Speed == 0 )
						 {
							Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100;
						 }
						 else
						 {
							Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100/OAxis_Manul_Speed;				   
						 }
						 //防止连动超过0点坐标
						 if(Axsis_Move_Direction[O_Axsis] == NEGATIVE)
						 {
							 Linked_Pulse=Pulse_TotalCounter[3]-MINROBOTPOSITION;
						 }
						 //防止连动超过最大软件限位
						 else if(Axsis_Move_Direction[O_Axsis] == POSITIVE)
						 {
							 Linked_Pulse=MAXROBOTPOSITION-Pulse_TotalCounter[3];
						 }
						 else
						 {
						 }				   	
				   }
				   else if(Axsis_Chosen == A_Axsis)
				   {
						 if(AAxis_Manul_Speed == 0 )
						 {
							Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100;
						 }
						 else
						 {
							Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100/OAxis_Manul_Speed;				   
						 }
						 //防止连动超过0点坐标
						 if(Axsis_Move_Direction[A_Axsis] == NEGATIVE)
						 {
							 Linked_Pulse=Pulse_TotalCounter[4]-MINROBOTPOSITION;
						 }
						 //防止连动超过最大软件限位
						 else if(Axsis_Move_Direction[A_Axsis] == POSITIVE)
						 {
							 Linked_Pulse=MAXROBOTPOSITION-Pulse_TotalCounter[4];
						 }
						 else
						 {
						 }				   	
				   }
				   else if(Axsis_Chosen == B_Axsis)
				   {
						 if(AAxis_Manul_Speed == 0 )
						 {
							Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100;
						 }
						 else
						 {
							Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = PULSE_MINHALF_PERIOD*100/OAxis_Manul_Speed;				   
						 }
						 //防止连动超过0点坐标
						 if(Axsis_Move_Direction[B_Axsis] == NEGATIVE)
						 {
							 Linked_Pulse=Pulse_TotalCounter[5]-MINROBOTPOSITION;
						 }
						 //防止连动超过最大软件限位
						 else if(Axsis_Move_Direction[B_Axsis] == POSITIVE)
						 {
							 Linked_Pulse=MAXROBOTPOSITION-Pulse_TotalCounter[5];
						 }
						 else
						 {
						 }				   	
				   }
			     /*-- 机械手安全区域检测 --*/
		       SafeAreaDetection();
			   if(Safe_Area_Flag==FALSE)
			   {				 	   
					Linked_Pulse_Count=Linked_Pulse;
 					Servo_Control_Mode =  LINKEDRUNNINGMODE;;	 
					 //---2017.8.1
					 switch(Axsis_Chosen)
					 {
						 case X_Axsis:
//					      Axis_Manul_Speed[X_Axsis]=XAxis_Manul_Speed;
					      //---2017.8.21
						    Axsis_Maxlength=X_Axsis_Maxlength_Can;
						    Axsis_Minlength=X_Axsis_Minlength_Can;
							break;
						 
						 case Z_Axsis:
//					      Axis_Manul_Speed[Z_Axsis]=ZAxis_Manul_Speed;
                //---2017.8.21
						    Axsis_Maxlength=Z_Axsis_Maxlength_Can;
						 		Axsis_Minlength=Z_Axsis_Minlength_Can;
							break;
						 
						 case L_Axsis:
//                Axis_Manul_Speed[L_Axsis]=LAxis_Manul_Speed;
                //---2017.8.21
    						Axsis_Maxlength=L_Axsis_Maxlength_Can;
						 	  Axsis_Minlength=L_Axsis_Minlength_Can;
							break;
						 
						 case O_Axsis:
                //---2017.8.21
						    Axsis_Maxlength=O_Axsis_Maxlength_Can;
						 		Axsis_Minlength=O_Axsis_Minlength_Can;
							break;	
						 case A_Axsis:
//                Axis_Manul_Speed[A_Axsis]=AAxis_Manul_Speed;
                //---2017.8.21
    						Axsis_Maxlength=A_Axsis_Maxlength_Can;
						 	  Axsis_Minlength=A_Axsis_Minlength_Can;
							break;
						 
						 case B_Axsis:
//                Axis_Manul_Speed[B_Axsis]=BAxis_Manul_Speed;
                //---2017.8.21
						    Axsis_Maxlength=B_Axsis_Maxlength_Can;
						 		Axsis_Minlength=B_Axsis_Minlength_Can;
							break;						 
					}
					 
					 Pulse_Maxspeed_HalfPeriod[Axsis_Chosen] = AUTO_PULSE_MINHALF_PERIOD*100/Axis_Manul_Speed[Axsis_Chosen];
					 if(Pulse_Maxspeed_HalfPeriod[Axsis_Chosen]>18000)//2018.03.27 解决小速度回零速度问题  lin
					 {
						Pulse_Maxspeed_HalfPeriod[Axsis_Chosen]=18000;
					 }
					 if(Pulse_Maxspeed_HalfPeriod[Axsis_Chosen]<180)//2018.03.27 解决小速度回零速度问题  lin
					 {
						Pulse_Maxspeed_HalfPeriod[Axsis_Chosen]=180;
					 }	

					 
					 if(Axsis_Move_Direction[Axsis_Chosen] == POSITIVE)
					 {
						 AXisMove(Axsis_Chosen,Axsis_Maxlength+1000000,Axis_Manul_Speed[Axsis_Chosen]* 100);//
					  }
					 else if(Axsis_Move_Direction[Axsis_Chosen] == NEGATIVE)
					 {
						 AXisMove(Axsis_Chosen,Axsis_Minlength+1000000,Axis_Manul_Speed[Axsis_Chosen]* 100);//Axis_Manul_Speed[Axsis_Chosen]*
					 }					 
			   }
		 	}
		 }
		 else //--if(Timer_Enable == DISABLE)
		 {	  
		 }	  		  
	 }
	  else if((Timer_Enable_X|Timer_Enable_Z|Timer_Enable_L|Timer_Enable_O|Timer_Enable_A|Timer_Enable_B))//---Linked_Mode_Enable == DISABLE   0925
	 {
		 Linked_Move_Enable = DISABLE;
		 Servo_Stop(Axsis_Chosen);			
		 ServoStop_CAN(Axsis_Chosen);
		 CAN_Servo_Stop_Action[Axsis_Chosen]=FALSE;	
	  }
}

/**************************************************************************************************
**  函数名：  IODebugOutput1()
**	输入参数：无
**	输出参数：无
**	函数功能：IO调试
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2015/07/28 
***************************************************************************************************/
void IODebugOutput1()
{
	switch(UsartReceiveData[1])
	{
		case 1:			//Y0
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(1);	   //置0
			 }
			 else
			 {
			    SetOutput(1);
			 }
		     break;	

		case 2:		    //Y1
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(2);
			 }
			 else
			 {
			    SetOutput(2);
			 }
		     break;	

		case 3:	        //Y2
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(3);
			 }
			 else
			 {
			    SetOutput(3);
			 }
		     break;	

		case 4:	        //Y3
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(4);
			 }
			 else
			 {
			    SetOutput(4);
			 }
		     break;

		case 5:	    	//Y4
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(5);
			 }
			 else
			 {
			    SetOutput(5);
			 }
		     break;

		case 6:			//Y5
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(6);
			 }
			 else
			 {
			    SetOutput(6);
			 }
		     break;	

		case 7:			//Y6
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(7);
			 }
			 else
			 {
			    SetOutput(7);
			 }
		     break;	

		case 8:			//Y7
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(8);
			 }
			 else
			 {
			    SetOutput(8);
			 }
		     break;	

		case 9:		    //Y8
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(9);
			 }
			 else
			 {
			    SetOutput(9);
			 }
		     break;

		case 10:   	    //Y9
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(10);
			 }
			 else
			 {
			    SetOutput(10);
			 }
		     break;

		case 11:	       //Y10
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(11);
			 }
			 else
			 {
			    SetOutput(11);
			 }
		     break;	

		case 12:		   //Y11
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(12);
			 }
			 else
			 {
			    SetOutput(12);
			 }
		     break;	

		case 13:		   //Y12
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(13);
			 }
			 else
			 {
			    SetOutput(13);
			 }
		     break;	

		case 14:		   //Y13
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(14);
			 }
			 else
			 {
			    SetOutput(14);
			 }
		     break;

		case 15:	       //Y14
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(15);
			 }
			 else
			 {
			    SetOutput(15);
			 }
		     break;
		case 16:	       //Y14
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(16);
			 }
			 else
			 {
			    SetOutput(16);
			 }
		     break;
		case 17:	       //Y14
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(17);
			 }
			 else
			 {
			    SetOutput(17);
			 }
		     break;
		case 18:	       //Y14
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(18);
			 }
			 else
			 {
			    SetOutput(18);
			 }
		     break;
		case 19:	       //Y14
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(19);
			 }
			 else
			 {
			    SetOutput(19);
			 }
		     break;
		case 20:	       //Y14
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(20);
			 }
			 else
			 {
			    SetOutput(20);
			 }
		     break;
		case 21:	       //Y14
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(21);
			 }
			 else
			 {
			    SetOutput(21);
			 }
		     break;
		case 22:	       //Y14
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(22);
			 }
			 else
			 {
			    SetOutput(22);
			 }
		     break;
		case 23:	       //Y14
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(23);
			 }
			 else
			 {
			    SetOutput(23);
			 }
		     break;
		case 24:	       //Y14
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(24);
			 }
			 else
			 {
			    SetOutput(24);
			 }
		     break;
		default:
		     break;
	}
}

/**************************************************************************************************
**  函数名：  IODebugOutput2()
**	输入参数：无
**	输出参数：无
**	函数功能：IO调试
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2015/07/28 
***************************************************************************************************/
void IODebugOutput2()
{
	switch(UsartReceiveData[1])
	{
		case P_IODEBUG_OUTPUT2_1:			//Y15
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_15);	   //置0
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_15);
			 }
		     break;	

		case P_IODEBUG_OUTPUT2_2:		    //Y16
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_16);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_16);
			 }
		     break;	

		case P_IODEBUG_OUTPUT2_3:	        //Y17
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_17);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_17);
			 }
		     break;	

		case P_IODEBUG_OUTPUT2_4:	        //Y18
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_18);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_18);
			 }
		     break;

		case P_IODEBUG_OUTPUT2_5:	    	//Y19
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_19);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_19);
			 }
		     break;

		case P_IODEBUG_OUTPUT2_6:			//Y20
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_20);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_20);
			 }
		     break;	

		case P_IODEBUG_OUTPUT2_7:			//Y21
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_21);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_21);
			 }
		     break;	

		case P_IODEBUG_OUTPUT2_8:			//Y22
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_22);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_22);
			 }
		     break;	

		case P_IODEBUG_OUTPUT2_9:		    //Y23
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_23);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_23);
			 }
		     break;

		case P_IODEBUG_OUTPUT2_10:   	    //Y24
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_24);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_24);
			 }
		     break;

		case P_IODEBUG_OUTPUT2_11:	       //Y25
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_25);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_25);
			 }
		     break;	

		case P_IODEBUG_OUTPUT2_12:		   //Y26
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_26);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_26);
			 }
		     break;	

		case P_IODEBUG_OUTPUT2_13:		   //Y27
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_27);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_27);
			 }
		     break;	

		case P_IODEBUG_OUTPUT2_14:		   //Y28
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_28);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_28);
			 }
		     break;

		case P_IODEBUG_OUTPUT2_15:	       //Y29
		  	 if(UsartReceiveData[2])
			 {
			 	ResetOutput(O_IODEBUG_OUTPUT_29);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_29);
			 }
		     break;

		default:
		     break;
	}

}

/**************************************************************************************************
**  函数名：  IODebugOutput3()
**	输入参数：无
**	输出参数：无
**	函数功能：IO调试
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2015/07/28 
***************************************************************************************************/
void IODebugOutput3()
{
	switch(UsartReceiveData[1])
	{
		case P_IODEBUG_OUTPUT3_1:			//Y30
		   if(UsartReceiveData[2])
			 {
			 	  ResetOutput(O_IODEBUG_OUTPUT_30);	   //置0
			 } 
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_30);
			 }
		   break;	

		case P_IODEBUG_OUTPUT3_2:		    //Y31
		   if(UsartReceiveData[2])
			 {
			 	  ResetOutput(O_IODEBUG_OUTPUT_31);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_31);
			 }
		   break;	

		case P_IODEBUG_OUTPUT3_3:	        //Y32
		   if(UsartReceiveData[2])
			 {
			 	  ResetOutput(O_IODEBUG_OUTPUT_32);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_32);
			 }
		   break;	

		case P_IODEBUG_OUTPUT3_4:	        //Y33
		   if(UsartReceiveData[2])
			 {
			 	  ResetOutput(O_IODEBUG_OUTPUT_33);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_33);
			 }
		   break;

		case P_IODEBUG_OUTPUT3_5:	    	//Y34
		   if(UsartReceiveData[2])
			 {
			 	  ResetOutput(O_IODEBUG_OUTPUT_34);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_34);
			 }
		   break;

		case P_IODEBUG_OUTPUT3_6:			//Y35
		   if(UsartReceiveData[2])
			 {
			 	  ResetOutput(O_IODEBUG_OUTPUT_35);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_35);
			 }
		   break;	

		case P_IODEBUG_OUTPUT3_7:			//Y36
		   if(UsartReceiveData[2])
			 {
			 	  ResetOutput(O_IODEBUG_OUTPUT_36);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_36);
			 }
		   break;	

		case P_IODEBUG_OUTPUT3_8:			//Y37
		   if(UsartReceiveData[2])
			 {
			 	  ResetOutput(O_IODEBUG_OUTPUT_37);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_37);
			 }
		   break;	

		case P_IODEBUG_OUTPUT3_9:		    //Y38
		   if(UsartReceiveData[2])
			 {
			 	  ResetOutput(O_IODEBUG_OUTPUT_38);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_38);
			 }
		   break;

		case P_IODEBUG_OUTPUT3_10:   	    //Y39
		   if(UsartReceiveData[2])
			 {
			  	ResetOutput(O_IODEBUG_OUTPUT_39);
			 }
			 else
			 {
			    SetOutput(O_IODEBUG_OUTPUT_39);
			 }
		   break;
			 
		default:
		   break;
	}
}

/**************************************************************************************************
**  函数名：  ManulDebug()
**	输入参数：无
**	输出参数：无
**	函数功能：监视命令
**	备注：	  无
**  作者：    吴祥     
**  开发日期：2013/12/20 
***************************************************************************************************/
void ManulDebug()
{
	u8 i=0,j=0,Axis=0,Axis_Speed=0,Axis_Dir=0;

	switch(UsartReceiveData[1])
	{
		case P_XAXIS_MANUL_SPEED:  //XY轴-手动速度值
		     Axis_Manul_Speed[X_Axsis] = UsartReceiveData[2];
		     Axis_Manul_Speed[L_Axsis] = UsartReceiveData[2];
			 break;

		case P_XAXIS_STEP_MM:	    //XY轴-寸动距离
			 XAxis_Step_Distance = UsartReceiveData[2];
			 LAxis_Step_Distance = UsartReceiveData[2];
		     break;
		case P_XAXIS_MOVE_LEFT:	   //X轴-左移
		     Axsis_Chosen = X_Axsis;
			 Axsis_Move_Direction[X_Axsis] = POSITIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_X)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_X==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];

					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[X_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
			 break;

		case P_XAXIS_MOVE_RIGHT:   //X轴-右移
		     Axsis_Chosen = X_Axsis;
			 Axsis_Move_Direction[X_Axsis] = NEGATIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_X)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_X==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];

					 Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[X_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_XAXIS_SAVE_POINT:   //X轴-保存   X轴料仓保存
		     //编号
			 //点信息
			/*************2018.6.9  WHITE_L*******************/
			if(UsartReceiveData[2]==0xAA)//无论是点修改还是插入写入位置与点的序号一一对应
			{				
//				W25QXX_Write(&UsartReceiveData[4],Saved_Program[g_Run_Program_Num-1].Address_Point+(UsartReceiveData[4]-1)*ORDER_LENTH,ORDER_LENTH);	 //每次写入一个坐标
			}
			else{
			 Manul_Save_PointX.Code  = UsartReceiveData[2]-1; //1~25  -1可以做数组下标
			 Manul_Save_PointX.Flag  = UsartReceiveData[3];	  //标志位
			 Manul_Save_PointX.Name  = (u32)( ((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24) );
		 	 Manul_Save_PointX.Point = (u32)( ((u32)UsartReceiveData[8])|((u32)UsartReceiveData[9]<<8)|((u32)UsartReceiveData[10]<<16)|((u32)UsartReceiveData[11]<<24) );
			 W25QXX_Write(&UsartReceiveData[2],0x3100+Manul_Save_PointX.Code*0x10,10);
			}				
			 break;

	  case P_XAXIS_BACK_ORIGIN:  //X轴回零
			 Axis_Manul_BackToOrigin=TRUE;	//手动回零标志位
			 Axsis_Chosen = X_Axsis;
			 Axsis_Move_Direction[X_Axsis] = NEGATIVE;
			 Linked_Mode_Enable = ENABLE;
			 Linked_Move_Enable = ENABLE;
			 Jog_Mode_Enable = DISABLE;
			 Jog_Move_Enable = DISABLE;
			 Jog_Pause_Enable= DISABLE;		
			 break;

		case P_ZAXIS_MANUL_SPEED:  //OZ轴-手动速度值
		     Axis_Manul_Speed[Z_Axsis] = UsartReceiveData[2];
		     Axis_Manul_Speed[O_Axsis] = UsartReceiveData[2];
			 break;

		case P_ZAXIS_STEP_MM:	    //Z轴-寸动距离
			 ZAxis_Step_Distance = UsartReceiveData[2];
			 OAxis_Step_Distance = UsartReceiveData[2];
		     break;

		case P_ZAXIS_MOVE_LEFT:	   //Z轴-上移
			 Speed_Change_Flag=1;
		     Axsis_Chosen = Z_Axsis;
			 Axsis_Move_Direction[Z_Axsis] = POSITIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_Z)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_Z==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];
					 
					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[Z_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_ZAXIS_MOVE_RIGHT:   //Z轴-下移
		     Axsis_Chosen = Z_Axsis;
			 Axsis_Move_Direction[Z_Axsis] = NEGATIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_Z)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_Z==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];
					 
					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[Z_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_ZAXIS_SAVE_POINT:   //Z轴-保存
		     //编号
			 //点信息
			/*************2018.6.9  WHITE_L*******************/
			if(UsartReceiveData[2]==0xAA)//无论是点修改还是插入写入位置与点的序号一一对应
			{				
//				W25QXX_Write(&UsartReceiveData[4],Saved_Program[UsartReceiveData[3]].Address_Point+(UsartReceiveData[4]-1)*ORDER_LENTH,ORDER_LENTH);	 //每次写入一个坐标
			}
			else{				
			 Manul_Save_PointZ.Code  = UsartReceiveData[2]-1; //1~25  -1可以做数组下标
			 Manul_Save_PointZ.Flag  = UsartReceiveData[3];	  //标志位
			 Manul_Save_PointZ.Name  = (u32)( ((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24) );
		 	 Manul_Save_PointZ.Point = (u32)( ((u32)UsartReceiveData[8])|((u32)UsartReceiveData[9]<<8)|((u32)UsartReceiveData[10]<<16)|((u32)UsartReceiveData[11]<<24) );
			 W25QXX_Write(&UsartReceiveData[2],0x3400+Manul_Save_PointZ.Code*0x10,10);
			}				
		     break;

	    case P_ZAXIS_BACK_ORIGIN:  //Z轴回零
		     Axis_Manul_BackToOrigin=TRUE;	//手动回零标志位
			 Axsis_Chosen = Z_Axsis;
			 Axsis_Move_Direction[Z_Axsis] = NEGATIVE;
			 Linked_Mode_Enable = ENABLE;
			 Linked_Move_Enable = ENABLE;
			 Jog_Mode_Enable = DISABLE;
			 Jog_Move_Enable = DISABLE;
			 Jog_Pause_Enable= DISABLE;			     
			 break;

		case P_LAXIS_MANUL_SPEED:  //L轴-手动速度值
		     Axis_Manul_Speed[L_Axsis] = UsartReceiveData[2];
			 break;

		case P_LAXIS_STEP_MM:	    //L轴-寸动距离
			 LAxis_Step_Distance = UsartReceiveData[2];
		     break;

		case P_LAXIS_MOVE_LEFT:	   //L轴-左移
		     Axsis_Chosen = L_Axsis;
			 Axsis_Move_Direction[L_Axsis] = NEGATIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_L)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_L==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];
					 
					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[L_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_LAXIS_MOVE_RIGHT:   //L轴-右移
		     Axsis_Chosen = L_Axsis;
			 Axsis_Move_Direction[L_Axsis] = POSITIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_L)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_L==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];
					 
					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[L_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_LAXIS_SAVE_POINT:   //L轴-保存
		   //编号
			 //点信息	
			/*************2018.6.9  WHITE_L*******************/
			if(UsartReceiveData[2]==0xAA)//无论是点修改还是插入写入位置与点的序号一一对应
			{				
//				W25QXX_Write(&UsartReceiveData[4],Saved_Program[UsartReceiveData[3]].Address_Point+(UsartReceiveData[4]-1)*ORDER_LENTH,ORDER_LENTH);	 //每次写入一个坐标
			}
			else{
			 Manul_Save_PointL.Code  = UsartReceiveData[2]-1; //1~25  -1可以做数组下标
			 Manul_Save_PointL.Flag  = UsartReceiveData[3];	  //标志位
			 Manul_Save_PointL.Name  = (u32)( ((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24) );
		 	 Manul_Save_PointL.Point = (u32)( ((u32)UsartReceiveData[8])|((u32)UsartReceiveData[9]<<8)|((u32)UsartReceiveData[10]<<16)|((u32)UsartReceiveData[11]<<24) );
			 W25QXX_Write(&UsartReceiveData[2],0x3700+Manul_Save_PointL.Code*0x10,10);		
			}				
	      break;

	    case P_LAXIS_BACK_ORIGIN:  //L轴回零
		     Axis_Manul_BackToOrigin=TRUE;	//手动回零标志位
			 Axsis_Chosen = L_Axsis;
			 Axsis_Move_Direction[L_Axsis] = NEGATIVE;
			 Linked_Mode_Enable = ENABLE;
			 Linked_Move_Enable = ENABLE;
			 Jog_Mode_Enable = DISABLE;
			 Jog_Move_Enable = DISABLE;
			 Jog_Pause_Enable= DISABLE;			     
			 break;

		case P_OAXIS_MANUL_SPEED:  //O轴-手动速度值
		     Axis_Manul_Speed[O_Axsis] = UsartReceiveData[2];
			 break;

		case P_OAXIS_STEP_MM:	    //O轴-寸动距离
			 OAxis_Step_Distance = UsartReceiveData[2];
		     break;

		case P_OAXIS_MOVE_LEFT:	   //O轴-左移
		     Axsis_Chosen = O_Axsis;
			 Axsis_Move_Direction[O_Axsis] = NEGATIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_O)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_O==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];

  					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[O_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_OAXIS_MOVE_RIGHT:   //O轴-右移
		     Axsis_Chosen = O_Axsis;
			 Axsis_Move_Direction[O_Axsis] = POSITIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_O)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_O==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];
					 
					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[O_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_OAXIS_SAVE_POINT:   //O轴-保存
		     //编号
			 //点信息
			/*************2018.6.9  WHITE_L*******************/
			if(UsartReceiveData[2]==0xAA)//无论是点修改还是插入写入位置与点的序号一一对应
			{				
//				W25QXX_Write(&UsartReceiveData[4],Saved_Program[UsartReceiveData[3]].Address_Point+(UsartReceiveData[4]-1)*ORDER_LENTH,ORDER_LENTH);	 //每次写入一个坐标
			}
			else{
			 Manul_Save_PointO.Code  = UsartReceiveData[2]-1; //1~25  -1可以做数组下标
			 Manul_Save_PointO.Flag  = UsartReceiveData[3];	  //标志位
			 Manul_Save_PointO.Name  = (u32)( ((u32)UsartReceiveData[4])|((u32)UsartReceiveData[5]<<8)|((u32)UsartReceiveData[6]<<16)|((u32)UsartReceiveData[7]<<24) );
		 	 Manul_Save_PointO.Point = (u32)( ((u32)UsartReceiveData[8])|((u32)UsartReceiveData[9]<<8)|((u32)UsartReceiveData[10]<<16)|((u32)UsartReceiveData[11]<<24) );
			 W25QXX_Write(&UsartReceiveData[2],0x3700+Manul_Save_PointO.Code*0x10,10);	
			}
	     break;

	    case P_OAXIS_BACK_ORIGIN:  //O轴回零
		     Axis_Manul_BackToOrigin=TRUE;	//手动回零标志位
			 Axsis_Chosen = O_Axsis;
			 Axsis_Move_Direction[O_Axsis] = NEGATIVE;
			 Linked_Mode_Enable = ENABLE;
			 Linked_Move_Enable = ENABLE;
			 Jog_Mode_Enable = DISABLE;
			 Jog_Move_Enable = DISABLE;
			 Jog_Pause_Enable= DISABLE;			     
			 break;

		case P_VAXIS_MANUL_SPEED:  //V轴-手动速度值
		     Axis_Manul_Speed[A_Axsis] = UsartReceiveData[2];
		     Axis_Manul_Speed[B_Axsis] = UsartReceiveData[2];
			 break;

		case P_VAXIS_STEP_MM:	    //V轴-寸动距离
			 AAxis_Step_Distance = UsartReceiveData[2];
			 BAxis_Step_Distance = UsartReceiveData[2];
		     break;

		case P_VAXIS_MOVE_LEFT:	   //A轴-左移
			 Speed_Change_Flag=1;
		     Axsis_Chosen = A_Axsis;
			 Axsis_Move_Direction[A_Axsis] = NEGATIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_A)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_A==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];
					 
					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[A_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_VAXIS_MOVE_RIGHT:   //A轴-右移
			 Speed_Change_Flag=1;
		     Axsis_Chosen = A_Axsis;
			 Axsis_Move_Direction[A_Axsis] = POSITIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_A)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_A==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];	
					 
					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[A_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_VAXIS_SAVE_POINT:   //V轴-保存
			
	     break;

	    case P_VAXIS_BACK_ORIGIN:  //V轴回零
		   Axis_Manul_BackToOrigin=TRUE;	//手动回零标志位
			 Axsis_Chosen = A_Axsis;
			 Axsis_Move_Direction[A_Axsis] = NEGATIVE;
			 Linked_Mode_Enable = ENABLE;
			 Linked_Move_Enable = ENABLE;
			 Jog_Mode_Enable = DISABLE;
			 Jog_Move_Enable = DISABLE;
			 Jog_Pause_Enable= DISABLE;			     
			 break;
	
		case P_WAXIS_MANUL_SPEED:  //W轴-手动速度值
		     Axis_Manul_Speed[B_Axsis] = UsartReceiveData[2];
			 break;

		case P_WAXIS_STEP_MM:	    //W轴-寸动距离
			 BAxis_Step_Distance = UsartReceiveData[2];
		     break;

		case P_WAXIS_MOVE_LEFT:	   //W轴-左移
		     Axsis_Chosen = B_Axsis;
			 Speed_Change_Flag=1;
			 Axsis_Move_Direction[B_Axsis] = NEGATIVE;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_B)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_B==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];
					 
					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[B_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_WAXIS_MOVE_RIGHT:   //W轴-右移
		     Axsis_Chosen = B_Axsis;
			 Axsis_Move_Direction[B_Axsis] = POSITIVE;
			 Speed_Change_Flag=1;
			 if(UsartReceiveData[2]==0x01)	//寸动
			 {
				 if(UsartReceiveData[3])
				 {  			      
					  if(Timer_Enable_B)
					  {
					  }
					  else
					  {
						  Jog_Mode_Enable = ENABLE;
						  Jog_Move_Enable = ENABLE;
						  Linked_Mode_Enable = DISABLE;	
					  }	
				 }
				 else
				 {
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;					   
				 }			 		 
			 }
			 if(UsartReceiveData[2]==0x02)	//连动
			 {
				 if(UsartReceiveData[3]&&Timer_Enable_B==0)
				 {  			      
					  Speed_Change_Flag=1;
					  Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[4];
					 
					  Linked_Mode_Enable = ENABLE;
					  Linked_Move_Enable = ENABLE;
//					  g_Link_Move_Delay = TRUE;
					  g_Link_Move_Count = 0;
					  Jog_Mode_Enable = DISABLE;
					  Jog_Move_Enable = DISABLE;
					  Jog_Pause_Enable= DISABLE;		
				 }
				 else
				 {
					  Axis_Manul_Speed[B_Axsis] = 0;
					  Linked_Move_Enable = DISABLE;					   
//					  g_Link_Move_Delay = FALSE;
					  g_Link_Move_Count = 0;
				 }
			 }
		     break;

		case P_WAXIS_SAVE_POINT:   //W轴-保存
			
	     break;

	    case P_WAXIS_BACK_ORIGIN:  //W轴回零
		   Axis_Manul_BackToOrigin=TRUE;	//手动回零标志位
			 Axsis_Chosen = B_Axsis;
			 Axsis_Move_Direction[B_Axsis] = NEGATIVE;
			 Linked_Mode_Enable = ENABLE;
			 Linked_Move_Enable = ENABLE;
			 Jog_Mode_Enable = DISABLE;
			 Jog_Move_Enable = DISABLE;
			 Jog_Pause_Enable= DISABLE;			     
			 break;
			
		case P_XZAXIS_SAVE_POINT:   //XZ轴-保存-写IIC		   
		     //编号
//			 //点信息
//			 Manul_Save_Point.Code=UsartReceiveData[2]-1;	  //1~25  -1可以做数组下标
//			 Manul_Save_Point.XPoint = (u32)( ((u32)UsartReceiveData[3])|((u32)UsartReceiveData[4]<<8)|((u32)UsartReceiveData[5]<<16)|((u32)UsartReceiveData[6]<<24) );
//		 	 Manul_Save_Point.ZPoint = (u32)( ((u32)UsartReceiveData[7])|((u32)UsartReceiveData[8]<<8)|((u32)UsartReceiveData[9]<<16)|((u32)UsartReceiveData[10]<<24) );
//			 W25QXX_Write(0x3100+Manul_Save_Point.Code*0x20,&UsartReceiveData[2],9);
			 break;

		case P_XZAXIS_DELETE_POINT:   //XZ轴-删除		   
			 if(UsartReceiveData[2]==0)//X点删除
			 {
				 Current_Delete_XZPoint = UsartReceiveData[3];	 //1~25
				 for(j=0;j<10;j++)
				 {
					Temp_IIC_XZPoint[j]=0;
				 }		     	
				 for(i=Current_Delete_XZPoint-1;i<25;i++)
				 {
				 	 if(i==24)
					 {			 	
						 for(j=0;j<10;j++)
						 {
							Temp_IIC_XZPoint[j]=0;
						 }
						 W25QXX_Write(Temp_IIC_XZPoint,0x3100+i*0x10,10);
					 }
					 else
					 {					
						 W25QXX_Read(Temp_IIC_XZPoint,0x3100+(i+1)*0x10,10);  
						 if(Temp_IIC_XZPoint[0]!=0)
						 {
						 	Temp_IIC_XZPoint[0] = Temp_IIC_XZPoint[0]-1;   //保持编号不变
						 }
						 W25QXX_Write(Temp_IIC_XZPoint,0x3100+i*0x10,10);	    
						 if(Temp_IIC_XZPoint[0]==0)	  //Flag=0,表示当前数组没有存入数据
						 {			 	
							break;
						 }
					 }
				 }	//for()		 
			 } //if(UsartReceiveData[2]==0)  //X点删除
			 else if(UsartReceiveData[2]==1) //Z点删除
			 {
				 Current_Delete_XZPoint = UsartReceiveData[3];	 //1~25
				 for(j=0;j<10;j++)
				 {
					Temp_IIC_XZPoint[j]=0;
				 }		     	
				 for(i=Current_Delete_XZPoint-1;i<25;i++)
				 {
				 	 if(i==24)
					 {			 	
						 for(j=0;j<10;j++)
						 {
							Temp_IIC_XZPoint[j]=0;
						 }
						 W25QXX_Write(Temp_IIC_XZPoint,0x3400+i*0x10,10);
					 }
					 else
					 {					
						 W25QXX_Read(Temp_IIC_XZPoint,0x3400+(i+1)*0x10,10);   
					 	 if(Temp_IIC_XZPoint[0]!=0)
						 {
						 	Temp_IIC_XZPoint[0] = Temp_IIC_XZPoint[0]-1;   //保持编号不变
						 }
						 W25QXX_Write(Temp_IIC_XZPoint,0x3400+i*0x10,10);	    
						 if(Temp_IIC_XZPoint[0]==0)	  //Flag=0,表示当前数组没有存入数据
						 {			 	
							break;
						 }
					 }
				 }	//for()					 
			 }	//if(UsartReceiveData[2]==1)  //Z点删除	 
		     break;

		case P_LOAXIS_DELETE_POINT:
			 if(UsartReceiveData[2]==0)//L点删除
			 {
				 Current_Delete_LOPoint = UsartReceiveData[3];	 //1~25
				 for(j=0;j<10;j++)
				 {
					Temp_IIC_LOPoint[j]=0;
				 }		     	
				 for(i=Current_Delete_LOPoint-1;i<25;i++)
				 {
				 	 if(i==24)
					 {			 	
						 for(j=0;j<10;j++)
						 {
							Temp_IIC_LOPoint[j]=0;
						 }
						 W25QXX_Write(Temp_IIC_LOPoint,0x3700+i*0x10,10);
					 }
					 else
					 {					
						 W25QXX_Read(Temp_IIC_LOPoint,0x3700+(i+1)*0x10,10);   
				 	     if(Temp_IIC_LOPoint[0]!=0)
						 {
						 	Temp_IIC_LOPoint[0] = Temp_IIC_LOPoint[0]-1;   //保持编号不变
						 }
						 W25QXX_Write(Temp_IIC_LOPoint,0x3700+i*0x10,10);	    
						 if(Temp_IIC_LOPoint[0]==0)	  //Flag=0,表示当前数组没有存入数据
						 {			 	
							break;
						 }
					 }
				 }	//for()		 
			 } //if(UsartReceiveData[2]==0)  //L点删除
			 else if(UsartReceiveData[2]==1) //O点删除
			 {
				 Current_Delete_LOPoint = UsartReceiveData[3];	 //1~25
				 for(j=0;j<10;j++)
				 {
					Temp_IIC_LOPoint[j]=0;
				 }		     	
				 for(i=Current_Delete_LOPoint-1;i<25;i++)
				 {
				 	 if(i==24)
					 {			 	
						 for(j=0;j<10;j++)
						 {
							Temp_IIC_LOPoint[j]=0;
						 }
						 W25QXX_Write(Temp_IIC_LOPoint,0x3A00+i*0x10,10);
					 }
					 else
					 {					
						 W25QXX_Read(Temp_IIC_LOPoint,0x3A00+(i+1)*0x10,10);   
					 	 if(Temp_IIC_LOPoint[0]!=0)
						 {
						 	Temp_IIC_LOPoint[0] = Temp_IIC_LOPoint[0]-1;   //保持编号不变
						 }
						 W25QXX_Write(Temp_IIC_LOPoint,0x3A00+i*0x10,10);	    
						 if(Temp_IIC_LOPoint[0]==0)	  //Flag=0,表示当前数组没有存入数据
						 {			 	
							break;
						 }
					 }
				 }	//for()					 
			 }	//if(UsartReceiveData[2]==1)  //O点删除	 
		     break;

		case P_MANUL_FINE_ADJUST:	    //微调
			 g_Manul_Fine_Adjust = UsartReceiveData[2];
		     break;
		
		case E_GEARRATIO_SET://电子齿轮比设置
			 W25QXX_Write(&UsartReceiveData[3],0x1300+UsartReceiveData[2]*2,2);	    			 
			break;
		case P_ACTION_SPEEDADJ:	//机械手速度调节
			
			 Axsis_Chosen = UsartReceiveData[3];			
			 Speed_Change_Flag = TRUE;
			 Axis_Manul_Speed[Axsis_Chosen] = UsartReceiveData[2];
			 Axsis_Move_Direction[Axsis_Chosen] = UsartReceiveData[4];
			 Linked_Move_Enable = ENABLE;
			 Linked_Mode_Enable = ENABLE;
			 g_Link_Move_Count = 0;
			 Jog_Mode_Enable = DISABLE;
			 Jog_Move_Enable = DISABLE;
			 Jog_Pause_Enable= DISABLE;	
//			Axis_Speed = UsartReceiveData[2];
//			Axis			 = UsartReceiveData[3];
//			Axis_Dir	 = UsartReceiveData[4];
//			if(Pulse_ControlStep[Axis]==SPEED_HOLD&&Speed_Up_Flag[Axis]==0&&Speed_Down_Flag[Axis]==0)
//				Sheet_Position[Axis] = Axis_Speed;//需要减速或者加速到哪一个速度对应到加速度表中
//		
//			if((Timer_Enable_X|Timer_Enable_L|Timer_Enable_Z|Timer_Enable_O|Timer_Enable_A|Timer_Enable_B)==0)
//			{
//				 Axsis_Chosen = Axis;
//				 Sheet_Position[Axis] = Axis_Speed;//需要减速或者加速到哪一个速度对应到加速度表中
//				 Axsis_Move_Direction[Axis] = Axis_Dir;		      
//				 Linked_Move_Enable = ENABLE;
//				 Linked_Mode_Enable = ENABLE;
//				 g_Link_Move_Count = 0;
//				 Jog_Mode_Enable = DISABLE;
//				 Jog_Move_Enable = DISABLE;
//				 Jog_Pause_Enable= DISABLE;					
//			}
		break;
		default:
		     break;
	}
}


/**************************************************************************************************
**  函数名：  Order_Package()
**	输入参数：无
**	输出参数：无
**	函数功能：点胶指令封装
**	备注：	  无
**  作者：    White_L     
**  开发日期：2018/7/1 
***************************************************************************************************/
extern u8 Mode_Flag;
u8 LC_Count_X[10],LC_Count_Y[10];
void Order_Package(void)
{
	u8 n=0,j=0;
	u16 m=0,k=0;
	
	 if(Read_Point_Num==0&&Order_Read_Mode==0)
		{
			for(j=0;j<10;j++)
			{
				LC_Count_X[j]=0;
				LC_Count_Y[j]=0;
			}
			if((Free_Program_Operate1.Num-Read_Point_Num)>LARGESTPROGRAMNUM)
			{
				m=LARGESTPROGRAMNUM;
			}
			else if(Free_Program_Operate1.Num-Read_Point_Num>0)
			{
				m=Free_Program_Operate1.Num-Read_Point_Num;			
			}
		 for(j=0;j<m;j++)
		 {
			 /*List key  value1 2 3 4 5*/
			 /*List Value1 2 3 4 5 LC_ID Key IO_Group*/
			W25QXX_Read(Read_IIC_Auto_Program,Saved_Program[g_Run_Program_Num-1].Address_Order+ORDER_LENTH*j,ORDER_LENTH);
			LC_ID = Read_IIC_Auto_Program[22];
			Free_Program_Operate1.Program[j].Flag   = 1;
			Free_Program_Operate1.Program[j].List   = Read_IIC_Auto_Program[0]|Read_IIC_Auto_Program[1]<<8;
//			Free_Program_Operate1.Program[j].Order  = OR_AXISORDER;
			Free_Program_Operate1.Program[j].Key    = Read_IIC_Auto_Program[23];
			Free_Program_Operate1.Program[j].Value1 = (u32)( ((u32)Read_IIC_Auto_Program[2])|((u32)Read_IIC_Auto_Program[3]<<8)|((u32)Read_IIC_Auto_Program[4]<<16)|((u32)Read_IIC_Auto_Program[5]<<24) );
			Free_Program_Operate1.Program[j].Value2 = (u32)( ((u32)Read_IIC_Auto_Program[6])|((u32)Read_IIC_Auto_Program[7]<<8)|((u32)Read_IIC_Auto_Program[8]<<16)|((u32)Read_IIC_Auto_Program[9]<<24) );	
			Free_Program_Operate1.Program[j].Value3 = (u32)( ((u32)Read_IIC_Auto_Program[10])|((u32)Read_IIC_Auto_Program[11]<<8)|((u32)Read_IIC_Auto_Program[12]<<16)|((u32)Read_IIC_Auto_Program[13]<<24) );
			Free_Program_Operate1.Program[j].Value4 = (u32)( ((u32)Read_IIC_Auto_Program[14])|((u32)Read_IIC_Auto_Program[15]<<8)|((u32)Read_IIC_Auto_Program[16]<<16)|((u32)Read_IIC_Auto_Program[17]<<24) );	
			Free_Program_Operate1.Program[j].Value5 = (u32)( ((u32)Read_IIC_Auto_Program[18])|((u32)Read_IIC_Auto_Program[19]<<8)|((u32)Read_IIC_Auto_Program[20]<<16)|((u32)Read_IIC_Auto_Program[21]<<24) );	
			Free_Program_Operate1.Program[j].LC_ID  = LC_ID;
			if(1==Read_IIC_Auto_Program[23]||2==Read_IIC_Auto_Program[23]||3==Read_IIC_Auto_Program[23]||5==Read_IIC_Auto_Program[23])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
			{
				Free_Program_Operate1.Program[j].IO_Group=Read_IIC_Auto_Program[24]|Read_IIC_Auto_Program[25]<<8|Read_IIC_Auto_Program[26]<<16|Read_IIC_Auto_Program[27]<<24;				
			}
			else if(4==Read_IIC_Auto_Program[23])
			{
				Free_Program_Operate1.Program[j].Delay_Time=Read_IIC_Auto_Program[26]|Read_IIC_Auto_Program[27]<<8; 
			} 
			Free_Program_Operate1.Program[j].Value1 = (Free_Program_Operate1.Program[j].Value1 & 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[j].Value2 = (Free_Program_Operate1.Program[j].Value2 & 0x0fffffff)+MINROBOTPOSITION;	
			Free_Program_Operate1.Program[j].Value3 = (Free_Program_Operate1.Program[j].Value3 & 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[j].Value4 = (Free_Program_Operate1.Program[j].Value4 & 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[j].Value5 = (Free_Program_Operate1.Program[j].Value5 & 0x0fffffff)+MINROBOTPOSITION;			 
			if(Free_Program_Operate1.Program[j].Key==A_QuLiao)//取料指令料仓位置计算
			{
//				 if(LC_Count_Y[LC_ID-1]<LC_Parameter_Load[LC_ID-1].V_Num)
//					 LC_Count_Y[LC_ID-1]++;
//				 else if(LC_Count_Y[LC_ID-1]>=LC_Parameter_Load[LC_ID-1].V_Num&&LC_Count_X[LC_ID-1]<LC_Parameter_Load[LC_ID-1].H_Num)
//				 {
//					 LC_Count_X[LC_ID-1]++;
//				 }
//				 else if(LC_Count_X[LC_ID-1]>=LC_Parameter_Load[LC_ID-1].H_Num)
//				 {
//					 LC_Count_X[LC_ID-1]=0;
//					 LC_Count_Y[LC_ID-1]=0;					 
//				 }
				Free_Program_Operate1.Program[j].Value1 = (LC_Load_Table[LC_ID-1].LC_Load_Position[LC_Count_X[LC_ID-1]][LC_Count_Y[LC_ID-1]].XPoint& 0x0fffffff)+MINROBOTPOSITION;
				Free_Program_Operate1.Program[j].Value2 = (LC_Load_Table[LC_ID-1].LC_Load_Position[LC_Count_X[LC_ID-1]][LC_Count_Y[LC_ID-1]].YPoint& 0x0fffffff)+MINROBOTPOSITION;
				Free_Program_Operate1.Program[j].Value3 = (LC_Parameter_Load[LC_ID-1].LC_Hight& 0x0fffffff)+MINROBOTPOSITION;
				Free_Program_Operate1.Program[j].Value4 = MINROBOTPOSITION;	
				Free_Program_Operate1.Program[j].Value5 = MINROBOTPOSITION;				 				 
			 }
		 }
		Read_Point_Num+=m;
		if(Free_Program_Operate1.Num>LARGESTPROGRAMNUM)
			Order_Read_Mode=1;
	 }
	else if(Read_Point_Num<=Free_Program_Operate1.Num&&Order_Read_Mode==1)//指令刷新
	{
			if(Read_Point_Num==Free_Program_Operate1.Num)
			{
				Read_Point_Num=0;
			}
			k = Read_Point_Num;																	//读取的程序位置
			m = ShuZu_Count;					  												//这条指令保存的数组位置
			ShuZu_Count++;
			if(ShuZu_Count==LARGESTPROGRAMNUM)
				ShuZu_Count=0;
			Read_Point_Num++;																		//读取数目
			W25QXX_Read(Read_IIC_Auto_Program,Saved_Program[g_Run_Program_Num-1].Address_Order+ORDER_LENTH*k,ORDER_LENTH);
			LC_ID = Read_IIC_Auto_Program[22];
			Free_Program_Operate1.Program[m].Flag   = 1;
			Free_Program_Operate1.Program[m].List   = Read_IIC_Auto_Program[0]|Read_IIC_Auto_Program[1]<<8;
//			Free_Program_Operate1.Program[m].Order  = OR_AXISORDER;
			Free_Program_Operate1.Program[m].Key    = Read_IIC_Auto_Program[23];
			Free_Program_Operate1.Program[m].Value1 = (u32)( ((u32)Read_IIC_Auto_Program[2])|((u32)Read_IIC_Auto_Program[3]<<8)|((u32)Read_IIC_Auto_Program[4]<<16)|((u32)Read_IIC_Auto_Program[5]<<24) );
			Free_Program_Operate1.Program[m].Value2 = (u32)( ((u32)Read_IIC_Auto_Program[6])|((u32)Read_IIC_Auto_Program[7]<<8)|((u32)Read_IIC_Auto_Program[8]<<16)|((u32)Read_IIC_Auto_Program[9]<<24) );	
			Free_Program_Operate1.Program[m].Value3 = (u32)( ((u32)Read_IIC_Auto_Program[10])|((u32)Read_IIC_Auto_Program[11]<<8)|((u32)Read_IIC_Auto_Program[12]<<16)|((u32)Read_IIC_Auto_Program[13]<<24) );
			Free_Program_Operate1.Program[m].Value4 = (u32)( ((u32)Read_IIC_Auto_Program[14])|((u32)Read_IIC_Auto_Program[15]<<8)|((u32)Read_IIC_Auto_Program[16]<<16)|((u32)Read_IIC_Auto_Program[17]<<24) );	
			Free_Program_Operate1.Program[m].Value5 = (u32)( ((u32)Read_IIC_Auto_Program[18])|((u32)Read_IIC_Auto_Program[19]<<8)|((u32)Read_IIC_Auto_Program[20]<<16)|((u32)Read_IIC_Auto_Program[21]<<24) );	
			Free_Program_Operate1.Program[m].LC_ID  = LC_ID;
			if(1==Read_IIC_Auto_Program[23]||2==Read_IIC_Auto_Program[23]||3==Read_IIC_Auto_Program[23]||5==Read_IIC_Auto_Program[23])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
			{
				Free_Program_Operate1.Program[m].IO_Group=Read_IIC_Auto_Program[24]|Read_IIC_Auto_Program[25]<<8|Read_IIC_Auto_Program[26]<<16|Read_IIC_Auto_Program[27]<<24;				
			}
			else if(4==Read_IIC_Auto_Program[23])
			{
				Free_Program_Operate1.Program[m].Delay_Time=Read_IIC_Auto_Program[26]|Read_IIC_Auto_Program[27]<<8; 
			}
			Free_Program_Operate1.Program[m].Value1 = (Free_Program_Operate1.Program[m].Value1 & 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[m].Value2 = (Free_Program_Operate1.Program[m].Value2 & 0x0fffffff)+MINROBOTPOSITION;	
			Free_Program_Operate1.Program[m].Value3 = (Free_Program_Operate1.Program[m].Value3 & 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[m].Value4 = (Free_Program_Operate1.Program[m].Value4 & 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[m].Value5 = (Free_Program_Operate1.Program[m].Value5 & 0x0fffffff)+MINROBOTPOSITION;		
		 if(Free_Program_Operate1.Program[m].Key==A_QuLiao)//取料指令料仓位置计算
		 {
			 if(LC_Count_Y[LC_ID-1]<LC_Parameter_Load[LC_ID-1].V_Num)
				 LC_Count_Y[LC_ID-1]++;
			 else if(LC_Count_Y[LC_ID-1]>=LC_Parameter_Load[LC_ID-1].V_Num)
			 {
				 LC_Count_Y[LC_ID-1]=0;
				 LC_Count_X[LC_ID-1]++;
			 }
			 else if(LC_Count_X[LC_ID-1]>=LC_Parameter_Load[LC_ID-1].H_Num)
			 {
				 LC_Count_X[LC_ID-1]=0;
				 LC_Count_Y[LC_ID-1]=0;					 
			 }
//			Free_Program_Operate1.Program[m].Value1 = (u32)( ((u32)Read_IIC_Auto_Program[2])|((u32)Read_IIC_Auto_Program[3]<<8)|((u32)Read_IIC_Auto_Program[4]<<16)|((u32)Read_IIC_Auto_Program[5]<<24) );
//			Free_Program_Operate1.Program[m].Value2 = (u32)( ((u32)Read_IIC_Auto_Program[6])|((u32)Read_IIC_Auto_Program[7]<<8)|((u32)Read_IIC_Auto_Program[8]<<16)|((u32)Read_IIC_Auto_Program[9]<<24) );
//			Free_Program_Operate1.Program[m].Value3 = (u32)( ((u32)Read_IIC_Auto_Program[10])|((u32)Read_IIC_Auto_Program[11]<<8)|((u32)Read_IIC_Auto_Program[12]<<16)|((u32)Read_IIC_Auto_Program[13]<<24) );
//			Free_Program_Operate1.Program[m].Value1 = (Free_Program_Operate1.Program[m].Value1 & 0x0fffffff)+MINROBOTPOSITION;
//			Free_Program_Operate1.Program[m].Value2 = (Free_Program_Operate1.Program[m].Value2 & 0x0fffffff)+MINROBOTPOSITION;
//			Free_Program_Operate1.Program[m].Value3 = (Free_Program_Operate1.Program[m].Value3 & 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[m].Value1 = (LC_Load_Table[LC_ID-1].LC_Load_Position[LC_Count_X[LC_ID-1]][LC_Count_Y[LC_ID-1]].XPoint& 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[m].Value2 = (LC_Load_Table[LC_ID-1].LC_Load_Position[LC_Count_X[LC_ID-1]][LC_Count_Y[LC_ID-1]].YPoint& 0x0fffffff)+MINROBOTPOSITION;
			Free_Program_Operate1.Program[m].Value3 = (LC_Parameter_Load[LC_ID].LC_Hight& 0x0fffffff)+MINROBOTPOSITION;		
			Free_Program_Operate1.Program[m].Value4 = MINROBOTPOSITION;	
			Free_Program_Operate1.Program[m].Value5 = MINROBOTPOSITION;			 
		 }
	}

}
/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

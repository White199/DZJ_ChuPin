/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : TIM6_USART1_IRQ.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
#include "stm32f10x_lib.h"
#include "CANopen.h"
#include "Usart.h"
#include "Manual.h"
#include "in.h"
#include "out.h"
#include "Auto.h"
#include "w25qxx.h" 

#include "Error.h"
#include "Parameter.h"
//#include "SpeedControl.h"
#include "StatusControl.h"
#include "Auto_2.h"	
#include "ActionOperate.h"
//--
#include "Manual.h"	
#include "SpeedControl.h" 
#include "PDO.h"	
extern u8 g_DianJiao_Delay_Step;
extern u32 g_DianJiao_Delay_Timer;

void TIM6_IRQHandler(void)
{
	u8 i=0;
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) 
  	{
    	TIM_ClearITPendingBit(TIM6, TIM_IT_Update);	

		 
	/**-- 检测输入信号定时 --**/
		 if(Input_Detect_Enable==DISABLE)
		 {
			 Input_Detect_Time++;
			 if(Input_Detect_Time == 10)	   //100ms检测一次
			 {			 	 
				 Input_Detect_Time = 0;
				 Input_Detect_Enable = ENABLE;
			 }
		 }

	/**-- 动作全程监测，防止过长时间无动作 --**/
		 if(g_Auto_ActionRun_Timer>0)
		 {
		  	g_Auto_ActionRun_Timer++;
			if(g_Auto_ActionRun_Timer>10000)
			{
			    g_Auto_ActionRun_Timer=0;
			}
		 }
	/**-- 料仓动作全程监测，防止过长时间无动作 --**/
		 if(g_LC_ActionRun_Timer>0)
		 {
		  	g_LC_ActionRun_Timer++;
			if(g_LC_ActionRun_Timer>10000)
			{
			    g_LC_ActionRun_Timer=0;
			}
		 }
	/**-- 伺服器延时 --**/
		 if(Action_Done_Flag==TRUE)
		 {
		 	 Action_Delay_Time++;
			 if(Action_Delay_Time==2)
			 {
			 	 Action_Delay_Time=0;
				 Action_Done_Flag=FALSE;
				 Action_Delay_Flag=TRUE;
			 }
		 }


	/**-- 上料完成脉冲信号延时 PE1  500ms--**/
//		 if((Output_Status[0]&0x01)==0x00)	 //低电平有效
//		 {
//		 	Output_Delay_Time++;
//			if(Output_Delay_Time>=100)  //1000ms
//			{
//				GPIO_SetBits(GPIOE,GPIO_Pin_1); 		  
//				Output_Status[0] |= 0x01;
//				Output_Delay_Time=0;
//			}
//		 }
	/**-- 卸料完成脉冲信号延时 PE0  500ms--**/
//		 if((Output_Status[0]&0x02)==0x00)
//		 {
//		 	Output_Delay_Time++;
//			if(Output_Delay_Time>=100)  //1000ms
//			{
//				GPIO_SetBits(GPIOE,GPIO_Pin_0); 			 
//				Output_Status[0] |= 0x02;
//				Output_Delay_Time=0;
//			}
//		 }
//    /**-- 脉冲信号延时  1000ms--**/
//		 if(Puls_Delay_Num>0)
//		 {
//		    for(i=0;i<50;i++)
//			{
//			    if(Puls_Delay_Enable[i])
//				{
//					Puls_Delay_Time[i]++;
//					if(Puls_Delay_Time[i]>=JXS_Parameter.PulseTime)	  //1000ms
//					{
//						SetOutput(i);
//						Puls_Delay_Enable[i]=DISABLE;
//						Puls_Delay_Time[i]=0;
//						if(Puls_Delay_Num>0)
//						{
//						    Puls_Delay_Num--;
//						}
//					}
//				}
//			}
//		 }
/**---------- 全自动运行计数参数 ---------------**/
	     if(g_AutoStatue == AUTO_RUNNING)
	     {
			 g_Auto_Circle_Timer ++;
			 //g_Auto_ActionRun_Timer ++;
			 if(g_ActionDelay_Step == 1)
			 {
			 	g_ActionDelay_Timer ++;		//动作延时计时器
			 }		 
			 if(g_LCDelay_Step == 1)
			 {
			 	g_LCDelay_Timer ++;		//动作延时计时器
			 }	
	      } 
/**---------- 延时指令运行计数参数 ---------------**/		  
		  if(g_Key_Delay_Flag)
		  {
		  	 g_Key_Delay_Timer++;
		  }	
/**---------- 料仓延时指令运行计数参数 ---------------**/		  
		  if(g_LC_Key_Delay_Flag)
		  {
		  	 g_LC_Key_Delay_Timer++;
		  }	
		  if(g_DianJiao_Delay_Step)
		  {
			  g_DianJiao_Delay_Timer++;
		  }
/**---------- 自动运行时串口通信延时计数参数 -----**/		  
		  //if(g_AutoStatue==AUTO_RUNNING)
		  if(Origin_Backed == TRUE)
		  {
		  	 g_USART_Delay_Timer++;	  //1000ms
			 if(g_USART_Delay_Timer>500)
			 {
			    g_USART_Delay_Timer=0;
			 }
		  }		  
	  				 			 			 			 
	}
}

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 				
	    //USART_ClearFlag(USART1, USART_IT_RXNE);
		ReceiveDataBuffer[ReceiveDataCounter] = USART_ReceiveData(USART1);
//
		if(NewOrder == FALSE)
		{			
			if(StartReceiveDataFlag==FALSE)
			{
				if(ReceiveDataBuffer[ReceiveDataCounter]==USART_SEND_START)
				{
					StartReceiveDataFlag=TRUE;     //接收到开始信号
				//	StartReceiveDataTimeFlag=TRUE;
					USART1ErrorFlag = FALSE;
				}
				else 
				{
					ReceiveDataCounter = 0;
				}
			} 
			else
			{
				ReceiveDataCounter++;
				if(ReceiveDataCounter==ReceiveDataBuffer[0])
				{
				
					StartReceiveDataFlag=FALSE;		           //接受完成
					ReceiveDataBufferOverflow=TRUE;		       //置位缓冲区满标志
					if(ReceiveDataBuffer[ReceiveDataCounter-1]!=USART_SEND_END)  //接收到的最后一个数据不是结束信号，表明接收数据失败
					{	
					 	 USART1ErrorFlag=TRUE;	               //表明接收数据出错								 
				         ReceiveDataCounter = 0;	
					}
					else  
					{
						 USART1ErrorFlag=FALSE;                //表明接收数据帧无误
						 UsartDataDecodeFlag=TRUE;
						 ReceiveDataLen=ReceiveDataCounter;    //记录接收数据长度
						 ReceiveDataCounter = 0;	 
						 NewOrder = TRUE;
						// StartReceiveDataTimeFlag=FALSE;
						// StartReceiveDataTime =0;
					}
	
				}
				else if(ReceiveDataCounter>=60)
				{
					     StartReceiveDataFlag=FALSE;			//接受完成								 
				         ReceiveDataCounter = 0;	
				}
			}	 				 
		}
	}
}


void EXTI9_5_IRQHandler(void)
{		
	if(EXTI_GetITStatus(EXTI_Line9)!=RESET)
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
		EXTI_ClearITPendingBit(EXTI_Line9); 
	} 
}

void EXTI15_10_IRQHandler(void)
{
//	if(EXTI_GetITStatus(EXTI_Line14)!=RESET)
//	{
//		PVD_data[0]=Product_Count.Accumulate_Count;
//		PVD_data[1]=Product_Count.Accumulate_Count>>8;
//		PVD_data[2]=Product_Count.Accumulate_Count>>16;
//		PVD_data[3]=Product_Count.Accumulate_Count>>24;
//		W25QXX_Write(PVD_data,0x0014,  4);
//	    /* Clear the Key Button EXTI line pending bit */
//		EXTI_ClearITPendingBit(EXTI_Line14); 
//	}

	if(EXTI_GetITStatus(EXTI_Line10)!=RESET)
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
		EXTI_ClearITPendingBit(EXTI_Line10); 
	}

	if(EXTI_GetITStatus(EXTI_Line11)!=RESET)
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
		EXTI_ClearITPendingBit(EXTI_Line11); 
	}

	if(EXTI_GetITStatus(EXTI_Line12)!=RESET)
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
		EXTI_ClearITPendingBit(EXTI_Line12); 
	} 
}

void PVD_IRQHandler(void)
{
//		PVD_data[0]=Product_Count.Accumulate_Count;
//		PVD_data[1]=Product_Count.Accumulate_Count>>8;
//		PVD_data[2]=Product_Count.Accumulate_Count>>16;
//		PVD_data[3]=Product_Count.Accumulate_Count>>24;

//    	W25QXX_Write(PVD_data,0x0014,  4);
    /* Clear the Key Button EXTI line pending bit */
		EXTI_ClearITPendingBit(EXTI_Line16); 
}

void USB_LP_CAN_RX0_IRQHandler(void)
{    
      CanRxMsg RxMessage;

      CAN_ClearITPendingBit(CAN_IT_FMP0);

      RxMessage.StdId=0x00;
      RxMessage.ExtId=0x00;
      RxMessage.IDE=0;
      RxMessage.DLC=0;
      RxMessage.FMI=0;
      RxMessage.Data[0]=0x00;
      RxMessage.Data[1]=0x00;
      RxMessage.Data[2]=0x00;
      RxMessage.Data[3]=0x00;
      RxMessage.Data[4]=0x00;
      RxMessage.Data[5]=0x00;
      RxMessage.Data[6]=0x00;
      RxMessage.Data[7]=0x00;

      CAN_Receive(CAN_FIFO0, &RxMessage);
          
      STD_ID              = RxMessage.StdId;       //COB-ID            PDO:180h
      CAN_Recieve_Data[0] = RxMessage.Data[0];     //SDO:CS命令符      PDO:6041[0]
      CAN_Recieve_Data[1] = RxMessage.Data[1];     //SDO:              PDO:6041[1]
      CAN_Recieve_Data[2] = RxMessage.Data[2];     //SDO:              PDO:6064[0]
      CAN_Recieve_Data[3] = RxMessage.Data[3];     //SDO:[3:1]:索引    PDO:6064[1]
      CAN_Recieve_Data[4] = RxMessage.Data[4];     //SDO:Data[0]       PDO:6064[2]
      CAN_Recieve_Data[5] = RxMessage.Data[5];     //SDO:Data[1]       PDO:6064[3]
      CAN_Recieve_Data[6] = RxMessage.Data[6];     //SDO:Data[2]                    
      CAN_Recieve_Data[7] = RxMessage.Data[7];     //SDO:Data[3]                    
      COB_ID              = STD_ID&0x780;          //COB-ID[10:7]	
	//---
	     Get_Status_Position();
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/


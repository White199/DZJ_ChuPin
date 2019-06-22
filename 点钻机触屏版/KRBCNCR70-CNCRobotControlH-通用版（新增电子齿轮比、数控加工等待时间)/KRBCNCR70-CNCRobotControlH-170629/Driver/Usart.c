/*************** (C) COPYRIGHT 2015 Kingrobot manipulator Team ****************
* File Name          : Usart.c
* Author             : 
* Version            : 
* Date               : 
* Description        : This file provides USART communicated functions.
******************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "Usart1_Robot.h"
//#include "Usart.h"
#include "DGUS.h"
#include "Delay.h"

/*-------------Usart1 parameter----------------*/
u8  USART1StartReceiveDataFlag=FALSE;        //开始接受数据标志
u8  USART1RecieveFinishedFlag = FALSE;		//通信完成标志位，用于协议反馈
u8  USART1RecieveCount = 0;				   //通信接收计时标志位，如果1s内未接收到，则出错
u8  USART1ErrorFlag=FALSE;	           //通信出错标志位
u8  USART1ReceiveDataBufferOverflow=FALSE;   //接收完成标志位
u8	USART1NewOrder=FALSE;			           //接收到新的数据
u8  USART1DataDecodeFlag = FALSE;		   //
u8  USART1StartReceiveDataTimeFlag = FALSE;  //串口接收数据计时标志位
u8  USART1StartReceiveDataTime = 0;          //串口接收数据计时 

u8  USART1SendBuff[60]={0};            //发送数据缓存
u8  USART1ReceiveDataBuffer[60]={0};         //接收数据缓存

u8  USART1ReceiveData[20]={0};		   //串口接收到的数据
u8  USART1ReceiveDataCounter=0;		       //接收数据计数
u8  USART1ReceiveDataLen=0;				   //记录接收数据长度
u8  USART1temp[4] = {0x01,0x02,0x03,0x04};
u8  USART1_Current_Order = 0;			   //串口1当前查询命令
u8  USART1_Current_Key = 0;				   //串口1当前查询key
u8  USART1_Current_Num = 0;				   //串口1当前查询Num

/*-------------Usart3_LCD parameter----------------*/
u8  LCDStartReceiveDataFlag=FALSE;        //开始接受数据标志
u8  LCDErrorFlag=FALSE;	                  //通信出错标志位
u8  LCDReceiveDataBufferOverflow=FALSE;   //接收完成标志位
u8  LCDStartReceiveDataTimeFlag = FALSE;  //串口接收数据计时标志位
u8  LCDStartReceiveDataTime = 0;          //串口接收数据计时 

u8  LCDReceiveDataBuffer[20]={0};         //接收数据缓存
u8  LCDReceiveData[20]={0};		          //串口接收到的数据
u8  LCDReceiveDataCounter=0;		      //接收数据计数
u8  LCDReceiveDataLen=0;				  //记录接收数据长度
u8  LCDtemp[4] = {0x01,0x02,0x03,0x04};



void UART1_SendByte(u16 Data)
{ 
   while (!(USART1->SR & USART_FLAG_TXE));    
   USART_SendData(USART1, Data); 
}

void UART1Write(vu8* data,u16 len)
{
	u16 i;
	for (i=0; i<len; i++)
	{
		UART1_SendByte(data[i]);
	}
}


void USART1_SendDataArray(u8 DataLen, u8 Order, u8 *Data)
{
	u8 i=3;
//	u8 j=0;
	USART1_Current_Order = 0;
//	while(StartReceiveDataFlag==TRUE);                    //等待上一次的数据接收完成
	while(DMA_GetCurrDataCounter(DMA1_Channel4)!= 0);     //等待上一次DMA完成传输任务
	//while(DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);   //等待上一次DMA完成传输任务
	DMA_ClearFlag(DMA1_FLAG_TC4);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	USART1SendBuff[0]=USART1_SEND_START;  //发送数据开始标志	
	USART1SendBuff[1]=DataLen+3;		 //数据长度   
	USART1SendBuff[2]=Order; 		     //发送命令
	USART1_Current_Order = Order;
	USART1_Current_Key = Data[0];
	if(DataLen>=3)
	{
		USART1_Current_Num = Data[2];
	}
	if(DataLen>0)
	{
		for(i=3;i<DataLen+3;i++)
		{
			USART1SendBuff[i]=Data[i-3];//发送数据
		}
	}
	USART1SendBuff[i]=USART1_SEND_END;	 //发送数据结束标志
	i++;
	DMA1_Channel4->CNDTR = i;
	DMA_Cmd(DMA1_Channel4, ENABLE);	
//	USART_ClearFlag(USART1,USART_FLAG_TC);
//	for(j=0;j<i;j++)
//	{			
//		USART1->DR = USART1SendBuff[j];
//		while((USART1->SR&0X40)==0);	
//	}		
}


void USART2_SendDataArray(u8 DataLen, u8 Order, u8 *Data)
{
	u8 i=3;
	u8 j=0;
	USART1_Current_Order = 0;
	USART1SendBuff[0]=USART1_SEND_START;  //发送数据开始标志	
	USART1SendBuff[1]=DataLen+3;		 //数据长度   
	USART1SendBuff[2]=Order; 		     //发送命令
	USART1_Current_Order = Order;
	USART1_Current_Key = Data[0];
	if(DataLen>=3)
	{
		USART1_Current_Num = Data[2];
	}
	if(DataLen>0)
	{
		for(i=3;i<DataLen+3;i++)
		{
			USART1SendBuff[i]=Data[i-3];//发送数据
		}
	}
	USART1SendBuff[i]=USART1_SEND_END;	 //发送数据结束标志
	i++;
	USART_ClearFlag(USART2,USART_FLAG_TC);
	DIR485_H;  //写方向
	for(j=0;j<i;j++)
	{			
		USART2->DR = USART1SendBuff[j];//---将待发送数据存入数据寄存器DR
		while((USART2->SR&0X40)==0);	
	}
	DIR485_L;  //读方向	
}


/*******************************USART1 Control_Board*********************************/
void USART1_SendData(u8 DataLen, u8 Order, u8 *Data)        //发送未知长度的数据
{	
	u8 static Sendcount1=0;
	USART1ErrorFlag = FALSE;
	while(Sendcount1<100)
	{
		USART1ErrorFlag=TRUE;
		Sendcount1++;
		#ifdef USART1_RS232
			USART1_SendDataArray(DataLen,Order,Data);
		#endif 
		#ifdef USART2_RS485
			USART2_SendDataArray(DataLen,Order,Data);
		#endif 
		USART1Acknowledge();   //串口1主板应答处理函数 
		if(USART1RecieveFinishedFlag == TRUE)
		{
			USART1ErrorFlag=FALSE;
			break;
		}
	}
	Sendcount1=0;
}


/*.....................LCD....................*/

void USART3_SendData(unsigned char *send_buff,u8 length)
{
    u8 i = 0;
	USART_ClearFlag(USART3,USART_FLAG_TC);
	for(i = 0;i < length;i ++)
	{			
		USART3->DR = send_buff[i];
		while((USART3->SR&0X40)==0);	
	}	
}

/******************* (C) COPYRIGHT 2015 Kingrobot manipulator Team *****END OF FILE****/

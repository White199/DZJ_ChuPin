
#include "stm32f10x.h"
//#include "SCI.H"
//#include "misc.h"
//#include "glovar.h"
//串口与DSP通信变量
u8  DSP_Data_Num=0;      //接收的数据个数
u8  DSP_Data[40]={0};    //接收到的数据
u8  DSP_CheckCodeTemp=0; //DSP串口数据校验码暂存（局部）
u8 char_count=0;   //标记接收DSP数据位数

//串口与DGUS通信变量
u8  LCD_Data_Num=0;      //接收的数据个数
u8  Lcd_Data[30]={0};    //接收到的数据


void LED_Configuration(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Enable the GPIO_LED Clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE , ENABLE); 
						 
			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_SetBits(GPIOE , GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5);
}

void USART_Configuration(void)
{ 
  
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 

 /********************主控板和DGUS之间的通信使用串口1**************************/
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO,ENABLE);
  /*
  *  USART1_TX -> PA9 , USART1_RX ->PA10
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无校验
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //发送和接受使能
  USART_Init(USART1, &USART_InitStructure); 
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART1, ENABLE); 
  USART_ClearITPendingBit(USART1, USART_IT_TC);//清除中断TC位



 /********************以下为USART2配置**************************/
 /*****************DSP和主控板之间通信使用串口2**************************/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
  /*
  *  USART2_TX -> Pd5 , USART2_RX ->	PD6
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无校验
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //发送和接受使能
  USART_Init(USART2, &USART_InitStructure); 
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART2, ENABLE); 
  USART_ClearITPendingBit(USART2, USART_IT_TC);//清除中断TC位


 /********************以下为USART3配置**************************/
 /*****************用于将FLASH中的PLT文件发送给DSP芯片**************************/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
  /*
  *  USART3_TX -> PB10 , USART3_RX ->	PB11
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据
  USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;//无校验
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; //发送和接受使能
  USART_Init(USART3, &USART_InitStructure); 
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  USART_Cmd(USART3, ENABLE); 
  USART_ClearITPendingBit(USART3, USART_IT_TC);//清除中断TC位


}

/***********************************************************************
函数名称：USART2_IRQHandler  串口2  中断
功    能：完成SCI的数据的接收，并做标识
输入参数：
输出参数：
编写时间：2014.9.11
编 写 人：我不怕
注    意  将收到的DSP数据发送给主控板用的是USART2.
***********************************************************************/
//void USART2_IRQHandler(void)  
//{
//	u8 usart1_ch;
//	static u8 len = 0;								  
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET && Receive_DSP_Flag == 0)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{	
//		USART_ClearITPendingBit(USART2, USART_IT_TC);
//		GPIO_SetBits( GPIOE,  GPIO_Pin_2); 
//		usart1_ch =USART2->DR;				//读取接收到的数据
//		ReceiveBuff[char_count]=usart1_ch;
//		if(ReceiveBuff[char_count]==0xED && StartRecive == 4)
//		{				  
//			  char_count=0;
//			  Receive_DSP_Flag=1;
//			  StartRecive=0;			
//		}
//		else if(StartRecive == 4)
//		{
//			  char_count=0;
//			  Receive_DSP_Flag=0;
//			  StartRecive=0;
//		}
//		else
//		{	
//			if(StartRecive == 3 && len == 0)
//			{
//				 StartRecive++;
//			}
//			else if(StartRecive == 3)
//			{
//				len--;
//			}
//			if(StartRecive == 2)
//			{
//				StartRecive++;
//				len = ReceiveBuff[char_count];
//			}
//			if(StartRecive==1)
//				StartRecive++;
//			if(StartRecive==0 && ReceiveBuff[char_count]==0x68)
//			{
//			      StartRecive=1;		
//			} 
//			if(StartRecive > 0)
//				char_count++;
//		}	
//	}
//	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	u8 usart1_ch;								  
//	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//	{	
//		
//		GPIO_SetBits( GPIOE,  GPIO_Pin_2); 
//		usart1_ch =USART2->DR;				//读取接收到的数据
//		ReceiveBuff[char_count]=usart1_ch;
//		if(StartRecive==0 && ReceiveBuff[char_count]==0x68)
//		{
//		      StartRecive=1;		
//		} 
//		char_count++;
//		if(ReceiveBuff[char_count-1]==0xED)
//		{				  
//			  char_count=0;
//			  Receive_DSP_Flag=1;
//			  StartRecive=0;			
//		}	
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	}
//}
/***********************************************************************
函数名称：void USART1_IRQHandler(void) 
功    能：完成SCI的数据的接收，并做标识
输入参数：
输出参数：
编写时间：2014.9.11
编 写 人：
注    意：接收DGUS屏传来的数据用的是USART1
***********************************************************************/
//void USART1_IRQHandler(void)  
//{
//	u8 i=0;
//	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET && Receive_Lcd_Flag == 0)
//	{ 	
//		USART_ClearITPendingBit(USART1, USART_IT_TC);			
//		Lcd_Data[LCD_Data_Num]=USART1->DR;
//		LCD_Data_Num++;		 //一个字节一个字节的发送的所以每发送一个字节就进一次中断
//		GPIO_ResetBits(GPIOE , GPIO_Pin_3);	//串口2通信指示灯
//		if(Lcd_Data[2]==0x08) //接收8个数据
//		{
//			if(LCD_Data_Num>10)
//			{		
//				if(Lcd_Data[0]==0x05&&Lcd_Data[1]==0x50&&Lcd_Data[3]==0x83)
//				{
//					{
//						Receive_Lcd_Flag=1;
//						g_AdrID=(Lcd_Data[4]<<8)&0xFF00;
//						g_AdrID+=Lcd_Data[5]&0x00FF;
//						g_KeyID=(Lcd_Data[7]<<24)&0xFF000000;
//						g_KeyID+=(Lcd_Data[8]<<16)&0x00FF0000;
//						g_KeyID+=(Lcd_Data[9]<<8)&0x0000FF00;
//						g_KeyID+=(Lcd_Data[10])&0x000000FF;
//					} 	
//				} 
//	 		}
//		}
//		else if(Lcd_Data[2]==0x06)//接收6个数据
//		{
//			if(LCD_Data_Num>8)
//			{
//				if(Lcd_Data[0]==0x05&&Lcd_Data[1]==0x50&&Lcd_Data[3]==0x83)
//				{
//					{
//						Receive_Lcd_Flag=1;
//						g_AdrID=(Lcd_Data[4]<<8)&0xFF00;
//						g_AdrID+=Lcd_Data[5]&0x00FF;
//						g_KeyID=(Lcd_Data[7]<<8)&0x0000FF00;
//						g_KeyID+=(Lcd_Data[8])&0x000000FF;						
//					} 
//				}
//			} 	
//		}
//		if(Receive_Lcd_Flag==1)		//存储缓冲区清零
//		{
//			for(i=0;i<LCD_Data_Num+1;i++)
//			{
//				 Lcd_Data[i]=0;
//			}
//			LCD_Data_Num=0;	
//		}
//	}
//	USART_ClearITPendingBit(USART1, USART_IT_RXNE);	
//}

/***********************************************************************
函数名称：USART3_Send_Data(unsigned char *send_buff,unsigned int length)
功    能：USART3_Send_Data发送PLT文件给DSP
输入参数：send_buff:待发送的数据指针；length：发送的数据长度（字符个数）
输出参数：
编写时间：2014.12.9
编 写 人：
注    意：
***********************************************************************/
void USART3_Send_Data(unsigned char *send_buff,unsigned int length)
{
 	unsigned int i = 0;
	USART_ClearFlag(USART3,USART_FLAG_TC);
	for(i = 0;i < length;i ++)
	{			
		USART3->DR = send_buff[i];
		while((USART3->SR&0X40)==0);	
	}	
}

/***********************************************************************
函数名称：USART2_Send_Data(unsigned char *send_buff,unsigned int length)
功    能：USART2_Send_Data发送字符串给DGUS屏的串口
输入参数：send_buff:待发送的数据指针；length：发送的数据长度（字符个数）
输出参数：
编写时间：2014.9.11
编 写 人：
注    意：
***********************************************************************/
void USART2_Send_Data(unsigned char *send_buff,unsigned int length)
{
 	unsigned int i = 0;
	USART_ClearFlag(USART2,USART_FLAG_TC);
	for(i = 0;i < length;i ++)
	{			
		USART2->DR = send_buff[i];
		while((USART2->SR&0X40)==0);	
	}	
}

/***********************************************************************
函数名称：USART1_Send_Data(unsigned char *send_buff,unsigned int length)
功    能：USART1发送内容给DSP的串口
输入参数：数据存储区，数据长度
输出参数：
编写时间：2014.9.11
编 写 人：我不怕
注    意：
***********************************************************************/
void USART1_Send_Data(unsigned char *send_buff,unsigned int length)
{
 unsigned int i = 0;
	USART_ClearFlag(USART1,USART_FLAG_TC);
	for(i = 0;i < length;i ++)
	{			
		USART1->DR = send_buff[i];
		while((USART1->SR&0X40)==0);	
	}	
}

/***********************************************************************
函数名称：NVIC_Configuration
功    能：中断优先级配置
输入参数：
输出参数：
编写时间：2014.9.11
编 写 人：我不怕
注    意：
***********************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef   NVIC_InitStructure;

  /* Set the Vector Table base location at 0x08000000 */
//  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  
//  /* Enable the Ethernet global Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);   
//  
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
 void USART3_IRQHandler(void)  
{

}

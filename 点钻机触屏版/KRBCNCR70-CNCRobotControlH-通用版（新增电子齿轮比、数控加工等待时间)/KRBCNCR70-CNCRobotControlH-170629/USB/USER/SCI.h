/***********************************************************************
文件名称：SCI.h
功    能：
编写时间：2012.11.22
编 写 人：
注    意：
***********************************************************************/
#ifndef  _SCI_H_
#define  _SCI_H_

#define RS_485_RX_EN 	GPIO_ResetBits(GPIOD , GPIO_Pin_4)	//接收使能
#define RS_485_TX_EN 	GPIO_SetBits(GPIOD , GPIO_Pin_4)  	//发送使能
#define RS485_REC_BUFF_SIZE				100
#define RS232_REC_BUFF_SIZE				100
#define RS485_END_FLAG1	'?'			//RS485一桢数据结束标志1 
#define RS485_END_FLAG2	';'			//RS485一桢数据结束标志2 
#define RS232_END_FLAG1	'?'			//RS232一桢数据结束标志1 
#define RS232_END_FLAG2	';'			//RS232一桢数据结束标志2 

void LED_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(void);
void USART2_Send_Data(unsigned char *send_buff,unsigned int length);
void USART1_Send_Data(unsigned char *send_buff,unsigned int length);
void USART3_Send_Data(unsigned char *send_buff,unsigned int length);
#endif

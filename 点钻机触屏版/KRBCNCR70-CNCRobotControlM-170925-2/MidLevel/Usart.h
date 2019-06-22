/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __usart_h_
#define __usart_h_

#define USART_SEND_START  0xf5         //数据发送的结束信号
#define USART_SEND_END    0xfa	       //数据发送的起始信号

#define DIR485_H  GPIO_SetBits(GPIOF,GPIO_Pin_7)   //GPIOF->BSRR=1<<7
#define DIR485_L  GPIO_ResetBits(GPIOF,GPIO_Pin_7) //GPIOF->BRR=1<<7

void UART1_SendByte(u16 dat);
void UART1Write(vu8* data,u16 len);
void USART1_SendData(u8,u8,u8*);
void UsartAcknowledge( u8 );
void UsartDataDecode(void);

extern u8  StartReceiveDataFlag;       //开始接受数据标志
extern u8  USART1ErrorFlag;	           //通信出错标志位
extern u8  ReceiveDataBufferOverflow;  //接收完成标志位
extern u8  SendDataBuffer[60];         //发送数据缓存
extern u8  ReceiveDataBuffer[60];      //接收数据缓存
extern u8  UsartReceiveData[60];	   //串口接收数据
extern u8  ReceiveDataCounter;		   //接收数据计数
extern u8  ReceiveDataLen;			   //记录接收数据长度
extern u8  NewOrder;		           //接收到新数据
extern u8  UsartDataDecodeFlag;		   //解码数据标志位
extern u8  StartReceiveDataTimeFlag ;  //串口接收数据计时标志位
extern u8  StartReceiveDataTime ;

#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

/*************** (C) COPYRIGHT 2012 Kingrobot manipulator Team ****************
* File Name          : 24Cxx.h
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 03/07/2012
* Description        : This file packaged the 24Cxx operating.
******************************************************************************/

#ifndef  _24Cxx_H
#define  _24Cxx_H

#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	    8191
#define AT24C128	16383
#define AT24C256	32767
#define AT24C512	65535
//Mini STM32开发板使用的是24c02，所以定义EE_TYPE为AT24C02
#define EE_TYPE 	AT24C512//AT24C16 	 

  
extern void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//从指定地址开始写入指定长度的数据
extern void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//从指定地址开始读出指定长度的数据


				  
extern u8 AT24CXX_ReadOneByte(u16 ReadAddr);							//指定地址读取一个字节
extern u16 AT24CXX_ReadTwoByte(u16 ReadAddr);
extern u32 AT24CXX_ReadFourByte(u16 ReadAddr);
extern void AT24CXX_WriteOneByte(u8 DataToWrite,u16 WriteAddr);		//指定地址写入一个字节
extern void AT24CXX_WriteTwoByte(u16 DataToWrite,u16 WriteAddr);		//指定地址写入一个字
extern void AT24CXX_WriteFourByte(u32 DataToWrite,u16 WriteAddr);
extern void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//指定地址开始写入指定长度的数据
extern u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据

extern u8 AT24CXX_Check(void);  //检查器件

#endif

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/

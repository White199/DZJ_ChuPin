/*************** (C) COPYRIGHT 2012 Kingrobot manipulator Team ****************
* File Name          : 24Cxx.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 03/07/2012
* Description        : This file packaged the 24Cxx operating.
******************************************************************************/

//#include "stm32f10x_lib.h"	
#include "stm32f10x.h"
#include "integer.h"	
#include "i2cint.h"
#include "24Cxx.h"
#include "Delay.h"

/**************************************************************************************************
**  函数名：  AT24CXX_ReadOneByte()
**	输入参数：需要读取数据的地址
**	输出参数：读取的得到的数据
**	函数功能：从24Cxx中读取一个字节数据
**	备注：	  无
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
	u8 temp=0;	
		  	    																 
    I2CStart();  
	if(EE_TYPE>AT24C16)
	{
		I2CSendByte(0XA0);	   //发送写命令
		I2CWaitAck();
		I2CSendByte(ReadAddr>>8);//发送高地址	    
	}
	else
	{
		I2CSendByte((u8)((ReadAddr>>7)&0xfe)|0XA0);   //发送器件地址0XA0,写数据 	   
	}
	I2CWaitAck(); 
    I2CSendByte((u8)(ReadAddr%256));   //发送低地址
	I2CWaitAck();	    
	I2CStart();  	 	   
	I2CSendByte(0xa1);           //进入接收模式			   
	I2CWaitAck();	 
    temp=I2CReadByte(0);		   
    I2CStop();//产生一个停止条件	    
	return temp;
}

/**************************************************************************************************
**  函数名：  AT24CXX_ReadTwoByte()
**	输入参数：需要读出的数据的值，需要写入数据的地址
**	输出参数：无
**	函数功能：向24Cxx中读出一个16位的数据
**	备注：	  无
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
u16 AT24CXX_ReadTwoByte(u16 ReadAddr)
{
	return (((AT24CXX_ReadOneByte(ReadAddr + 1)) << 8) | (AT24CXX_ReadOneByte(ReadAddr)));
}

/**************************************************************************************************
**  函数名：  AT24CXX_ReadFourByte()
**	输入参数：需要读出的数据的值，需要写入数据的地址
**	输出参数：无
**	函数功能：向24Cxx中读出一个32位的数据
**	备注：	  无
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
u32 AT24CXX_ReadFourByte(u16 ReadAddr)
{
	return (((AT24CXX_ReadOneByte(ReadAddr + 3)) << 24) | ((AT24CXX_ReadOneByte(ReadAddr + 2)) << 16) | ((AT24CXX_ReadOneByte(ReadAddr + 1)) << 8) | (AT24CXX_ReadOneByte(ReadAddr)));
}


/**************************************************************************************************
**  函数名：  AT24CXX_WriteOneByte()
**	输入参数：需要写入的数据的值，需要写入数据的地址
**	输出参数：无
**	函数功能：向24Cxx中写入一个字节数据
**	备注：	  无
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
void AT24CXX_WriteOneByte(u8 DataToWrite,u16 WriteAddr)
{				   	  	    																 
    I2CStart();  
	if(EE_TYPE>AT24C16)
	{
		I2CSendByte(0XA0);	    //发送写命令
		I2CWaitAck();
		I2CSendByte(WriteAddr>>8);//发送高地址	  
	}
	else 
	{
		I2CSendByte((u8)((WriteAddr>>7)&0xfe)|0XA0);   //发送器件地址0XA0,写数据 
	}	 
	I2CWaitAck();	   
    I2CSendByte(WriteAddr%256);   //发送低地址
	I2CWaitAck(); 	 										  		   
	I2CSendByte(DataToWrite);     //发送字节							   
	I2CWaitAck();  		    	   
    I2CStop();//产生一个停止条件 
	DelayNms(2);	 
}

/**************************************************************************************************
**  函数名：  AT24CXX_WriteTwoByte()
**	输入参数：需要写入的数据的值，需要写入数据的地址
**	输出参数：无
**	函数功能：向24Cxx中写入一个16位的数据
**	备注：	  无
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
void AT24CXX_WriteTwoByte(u16 DataToWrite,u16 WriteAddr)
{				   	  	    																 
	u8 temp_l=0,temp_h=0;
	temp_l=(u8)DataToWrite;
	temp_h=(u8)(DataToWrite>>8);	
	AT24CXX_WriteOneByte(temp_l,WriteAddr);
	AT24CXX_WriteOneByte(temp_h,WriteAddr+1); 
}

/**************************************************************************************************
**  函数名：  AT24CXX_WriteFourByte()
**	输入参数：需要写入的数据的值，需要写入数据的地址
**	输出参数：无
**	函数功能：向24Cxx中写入一个32位的数据
**	备注：	  无
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
void AT24CXX_WriteFourByte(u32 DataToWrite,u16 WriteAddr)
{				   	  	    																 
	u8 temp[4]={0,0,0,0};
	temp[0]=(u8)DataToWrite;
	temp[1]=(u8)(DataToWrite>>8);
	temp[2]=(u8)(DataToWrite>>16);
	temp[3]=(u8)(DataToWrite>>24);	
	AT24CXX_WriteOneByte(temp[0],WriteAddr);
	AT24CXX_WriteOneByte(temp[1],WriteAddr+1);
	AT24CXX_WriteOneByte(temp[2],WriteAddr+2);
	AT24CXX_WriteOneByte(temp[3],WriteAddr+3); 
}

/**************************************************************************************************
**  函数名：  AT24CXX_WriteLenByte()
**	输入参数：需要写入数据的地址，需要写入的数据的值，需要写入数据的长度
**	输出参数：无
**	函数功能：向24Cxx中写入一个16位或32位的数据
**	备注：	  无
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
	u8 t;
	for(t=0;t<Len;t++)
	{
		AT24CXX_WriteOneByte((DataToWrite>>(8*t))&0xff,WriteAddr+t);
	}												    
}

/**************************************************************************************************
**  函数名：  AT24CXX_ReadLenByte()
**	输入参数：需要读取数据的地址，需要写入的数据的长度，
**	输出参数：读取得到的数据
**	函数功能：从24Cxx中读取一个16位或32位的数据
**	备注：	  无
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
	u8 t;
	u32 temp=0;
	for(t=0;t<Len;t++)
	{
		temp<<=8;
		temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
	}
	return temp;												    
}

/**************************************************************************************************
**  函数名：  AT24CXX_Check()
**	输入参数：无
**	输出参数：检测结果 1：检测失败；0：检测成功
**	函数功能：检测芯片是否正常
**	备注：	  对芯片最后一个地址储存标志字，进行校验
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp=AT24CXX_ReadOneByte(EE_TYPE);//避免每次开机都写AT24CXX			   
	if(temp==0X55)
	{
		return 0;
	}		   
	else//排除第一次初始化的情况
	{
		AT24CXX_WriteOneByte(0X55,EE_TYPE);
	    temp=AT24CXX_ReadOneByte(EE_TYPE);	  
		if(temp==0X55)
		{
			return 0;
		}
	}
	return 1;											  
}

/**************************************************************************************************
**  函数名：  AT24CXX_Read()
**	输入参数：读取数据串的地址，存放数据缓冲区，数据长度
**	输出参数：无
**	函数功能：读取一串数据
**	备注：	  无
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer, u16 NumToRead)
{
	while(NumToRead)
	{
		*pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);	
		NumToRead--;
	}
}  

/**************************************************************************************************
**  函数名：  AT24CXX_Write()
**	输入参数：写入数据串的地址，数据缓冲区，数据长度
**	输出参数：无
**	函数功能：写入一串数据
**	备注：	  无
**  作者：    周海波    
**  开发日期：2012/7/3
***************************************************************************************************/
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
	u16 i=0;

	for(;i < NumToWrite;i++)
	{
		AT24CXX_WriteOneByte(pBuffer[i],WriteAddr);
		WriteAddr++;
	}
}

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/

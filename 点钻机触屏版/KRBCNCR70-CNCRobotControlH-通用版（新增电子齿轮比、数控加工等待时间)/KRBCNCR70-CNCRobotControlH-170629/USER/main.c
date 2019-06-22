/********************(C)COPYRIGHT	2012 Kingrobot manipulator Team**********************
** 文件名字:	main.c
** 作者:   		朱来
** 版本：		V1.0.0
** 日起：		2012/7/10
** 描述：		此文件提供主函数
*****************************************************************************************/	
/*-------USB---------*/
#include "usbh_core.h"
#include "usbh_usr.h"
#include "usbh_msc_core.h"


//#include "stm32f10x_lib.h"
#include "PlatformInit.h"					

#include "Delay.h"
#include "i2cint.h"
#include "24Cxx.h" 

#include "DGUS.h"
//#include "Usart.h"
#include "Usart1_Robot.h"
#include "Key.h"


u8 send_buff2[]={"无报警"};
u8 Beyond_LastDa[]={":超过使用期限,请联系厂家解密!  "};

int main(void)
{  
//	u8 data[500]={0};
	RCC_Configuration();	  		       //打开各模块开时钟
	NVIC_Configuration();	  		       //优先级配置  
	GPIO_Configuration();
	Key_EXTI_Init();
	TimeInit();				  		           //系统时钟配置
	#ifdef USART1_RS232
		USART1_DMA_Configuration();		   //DMA-Usart初始化
	#endif 
	#ifdef USART2_RS485
		USART2_485_Configuration();		   //Usart2初始化
	#endif
	  USART3_LCD_Configuration();		   //Usart3初始化   
	
	I2CInit();				  		           //I2C初始化	

//	 AT24CXX_Write(0x0000, data, 12);
//	 AT24CXX_Write(0x1020,data,10);
//	 AT24CXX_Write(0x1015,data,1);//当前运行程序编号

 	DelayNms(1000);				             //系统延时 
	RTC_Init();	//初始化实时时钟-检查电池是否安装（可能晶振需要一定的起振时间第一次可能开不了机）
	ReadIICInitPar();

/*----------------确定是否超期-------------------*/ 	
	InitLockTime();                    //设置加密锁初始时间
	UnLock_Forever_flag=1;
	if(UnLock_Forever_flag==1)       // 长期有效
	{
		Reset_Flag=1;
		Unlock_LastDay=9999;
	} 
    else				 
	{
		CheckTime();
	}
	//当电池拔掉或者超期是，进入解密界面~解密成功后，继续往下。
	while(!Reset_Flag)	               //Reset_Flag=1时继续往下 
	{
		UnlockCodeInput();	   		
	}
/*****************确定是否超期*******************/

	TIM_Cmd(TIM4,ENABLE);

/*----------------确定主机初始化完成-------------------*/ 
	MainBoard_InitDone();//
/*****************确定主机初始化完成*******************/
	
//	DelayNms(2000);
/*----------------开机同步数据-------------------*/ 
	while(!DataSyc());//同步函数
/*****************开机同步数据*******************/
    //Program_Ready = TRUE;  //??
   
  TIM_Cmd(TIM4,DISABLE);

/*----------------USB初始化-------------------*/ 
 	/* Init Host Library 1*/
  USBH_Init(&USB_OTG_FS_dev, &MSC_cb , &USR_Callbacks);//USB的初始化及配置
/*****************USB初始化*******************/	
	//联动按压数据返回
	LinkMoveDataInit();
	//参数初始化-页面初始化
//	MYDMA_Config();													//DMA初始化（多路模拟量数据传输，减轻CPU负担，降低功耗）
//	Adc_Init();															//ADC初始化（模拟量采集）
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE); // 启动ADC1
//	DMA_Cmd(DMA1_Channel1, ENABLE);  				//启动DMA通道
	ReadIICAlarm();
	Robot_Init();
	while(1)																			  
	{
//	   RodProcess();			//遥杆处理
	   KeyProcess();
	   LCDProcess();			//触摸处理
	   LCDDisplay();			//LCD显示处理
	   LCDRealTimeWatch();//实时监视参数
	   LCDError();
	   //点击进入U盘拷贝界面
	   if(USBH_Process_Flag)
	   {
	   		USBH_Process();
	   }
	}
}

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/

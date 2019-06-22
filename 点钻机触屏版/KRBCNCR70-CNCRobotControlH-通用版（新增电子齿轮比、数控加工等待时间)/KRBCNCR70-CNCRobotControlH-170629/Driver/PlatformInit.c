/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : Usart.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "PlatformInit.h"
#include "usart.h"
#include "Parameter.h"
#include "key.h"
#include "DGUS.h"
 #include "stdlib.h"
 #include "delay.h"
//#define SRC_USART3_DR    (USART3_BASE + 0x04)        //串口接收寄存器作为源头
#define SRC_USART1_DR    (USART1_BASE + 0x04)        //串口接收寄存器作为源头
extern u8 Product_Num;
u16 Rod_Count=0;
u8 Rod_Speed_Rank=0;;//判断当前速度等级
u8 Axis_DIR=0;//当前方向
u8 Link_Step_Flag=0;//寸动联动切换标志位
u16 Rod_X_Positive=0, Rod_X_Negative=0,Rod_Y_Positive=0,Rod_Y_Negative=0,Rod_Centry_X=0,Rod_Centry_Y=0;
u16 Rod_Step_X[8]={0},Rod_Step_Y[8]={0};
//N_flag:1 电机在反转 P_flag: 1电机在正转 
/*---------------程序编辑界面-------------------*/
extern u8 g_Background_Color_ProgramEdit ;			      //编辑背景色
extern u8 gs_Background_Color_ProgramEdit;
extern u8 gs_Current_Operate_List_ProgramEdit;				//点选择界面蓝色背景所在的点编号

extern  u8 gs_LinkMove_Speed[6];	         //X轴连动速度
extern u8 gs_XStepMove_Distance;        //X轴寸动距离(延时)
extern u8 gs_ZStepMove_Distance;        //Z轴寸动距离
extern u8 gs_LStepMove_Distance;        //L轴寸动距离
extern u8 gs_OStepMove_Distance;        //O轴寸动距离
extern u8 gs_AStepMove_Distance;        //A轴寸动距离
extern u8 gs_BStepMove_Distance;        //B轴寸动距离

extern u8 Step_Period_Count;//寸动周期

extern DZJSpeed DZJSpeed1;
u8 Speed_Rank[5]={0};
/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{ 
//  ErrorStatus HSEStartUpStatus;
//  RCC_DeInit();
//  RCC_HSEConfig(RCC_HSE_ON);
//  HSEStartUpStatus = RCC_WaitForHSEStartUp();
//  if(HSEStartUpStatus == SUCCESS)
//  {
//    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
//    FLASH_SetLatency(FLASH_Latency_2);
//    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
//    RCC_PCLK2Config(RCC_HCLK_Div1); 
//    RCC_PCLK1Config(RCC_HCLK_Div2);
//    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);	
//    RCC_PLLCmd(ENABLE);
//    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
//    {
//    }
//    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
//    while(RCC_GetSYSCLKSource() != 0x08)
//    {
//    }
//  }
   /*-----读取时钟信息-----*/
//   RCC_ClocksTypeDef RCC_ClockFreq;
//   RCC_GetClocksFreq(&RCC_ClockFreq);

//  //USB时钟设置wx
//  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);


  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                          RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF |
						              RCC_APB2Periph_AFIO , ENABLE );

  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 , ENABLE );  
						   
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}


/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{ 
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM  
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

   /* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);		
 
  /* Enable the USART1_IRQ/USART3_IRQ Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
		
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	 /* Enable the TIM3_IRQ Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	 /* Enable the TIM4_IRQ Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* 配置端口PB0为外部中断，遥杆按键 */ 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures IO
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

#if defined USE_USART2	&& defined USE_STM3210B_EVAL
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
#endif
  /*-------------------------UART1----PA9-PA10----------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//uart1引脚 TX
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//uart1引脚	RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*-------------------------UART2----PA1-PA2-PA3-------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//485-DIR
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//485-TX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//485-RX
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

 /*-------------------------UART3------PB10-PB11--------------------*/
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;//uart3引脚
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;//uart3引脚
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);



/*-------------------------GPIO---------------------------*/

  /**----中按键膜引脚初始化--输入--**/
  //PB12~PB15	 输出
  //PC5 ~PC11    内部上拉
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15  ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB,GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 |GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  

  /**----遥杆按键(PB0)引脚初始化--输入--**/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  /**----底部按键(PB1)及钥匙(PA0 PC13)引脚初始化--输入--**/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /**----脉冲编码器引脚(PC1 PC2 PC3)--输入--**/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 |GPIO_Pin_3  ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7  ;	  //硬件引脚错误，PA6-PC3 PA7-PC2需要修改20161103
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 

}
void Key_EXTI_Init(void)
{
		EXTI_InitTypeDef  EXTI_InitStructure; 
    EXTI_DeInit();
    EXTI_StructInit(&EXTI_InitStructure);  
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

   //PB0 遥杆按键
    EXTI_InitStructure.EXTI_Line = EXTI_Line0; 			//
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 	//  
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE; 				//  
    EXTI_Init(&EXTI_InitStructure); 						//
}

/*************************************************************************
**  函数名：  Adc_Init()
**	输入参数：
**	输出参数：无
**	函数功能：ADCC初始化
**	备注：	  无
**  作者：    White_L   
**  开发日期：2018/7/20
**************************************************************************/
void Adc_Init(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟

	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	/*----遥杆四路ADC采集（PA4 PA5 靠近板子外边的一个遥杆 PC2 PC3靠近板子内部的一个遥杆）---*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;//ADC12_IN4 ADC12_IN5 通道4.5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;//ADC12_IN12  ADC12_IN13 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	ADC_DeInit(ADC1);  //复位ADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 4;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   
	
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 2, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_239Cycles5 );	
	//使能ADC1的DMA  
  ADC_DMACmd(ADC1, ENABLE);  
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
}
/*************************************************************************
**  函数名：  MYDMA_Config()
**	输入参数：
**	输出参数：无
**	函数功能：DMA初始化
**	备注：	  无
**  作者：    White_L   
**  开发日期：2018/7/20
**************************************************************************/
vu16  AD_Value[10][4];   //用来存放ADC转换结果，也是DMA的目标地址
vu16  After_filter[4];    //用来存放求平均值之后的结果 
void MYDMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	
  DMA_DeInit(DMA1_Channel1);   //将DMA的通道1寄存器重设为缺省值

	DMA_InitStructure.DMA_PeripheralBaseAddr =  (u32)&ADC1->DR;  //DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = 40;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;  //工作在正常模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
	  	
}
/*************************************************************************
**  函数名：  Rod_Select_Order
**	输入参数：
**	输出参数：无
**	函数功能：通过遥杆实现翻页选指令
**	备注：	  无
**  作者：    White_L   
**  开发日期：2018/11/02
**************************************************************************/

void Rod_Select_Order(u8 Page_Type)
{
	int  sum = 0,i=0,j=0,k=0,n=0;
	u8 count=0;
	 for(i=0;i<2;i++)         
	 {            
			for ( count=0;count<10;count++)            
			{   
				sum += AD_Value[count][i];             
			}            
			After_filter[i]=sum/10;            
			sum=0;       
	}
	 /**************************产品编辑页面****************************/
	if(After_filter[0] > 2150)//前推上移
	{
		if(Page_Type==2)
		{
		 if(gs_Current_Operate_List==g_Background_Color)	//程序的前八行
		 {
			 if(gs_Current_Operate_List>1&&Rod_Count==0)//单次上移
			 {
					gs_Current_Operate_List--;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List>1&&Rod_Count>500)//检测长时间遥杆前推一直继续上移
			 {
					Rod_Count=1;
					gs_Current_Operate_List--;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;	
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
		 //不在前八行
		 else
		 {
			 if(gs_Current_Operate_List>1&&Rod_Count==0)//单次上移
			 {
					gs_Current_Operate_List--;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List>1&&Rod_Count>500)//检测长时间遥杆前推一直继续上移
			 {
					Rod_Count=1;
					gs_Current_Operate_List--;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;		
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
	  }
		Rod_Count++;				
	}
	else if(After_filter[0] < 1950)//后推下移
	{
		if(Page_Type==2)//产品编辑页面
		{
		 if(gs_Current_Operate_List==g_Background_Color)	//程序的前八行
		 {
			 if(gs_Current_Operate_List<Product_Num&&Rod_Count==0)//单次下移
			 {
					gs_Current_Operate_List++;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List<Product_Num&&Rod_Count>500)//检测长时间遥杆前推一直继续上移
			 {
					Rod_Count=1;
					gs_Current_Operate_List++;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;	
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
		 //不在前八行
		 else
		 {
			 if(gs_Current_Operate_List<Product_Num&&Rod_Count==0)//单次上移
			 {
					gs_Current_Operate_List++;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List<Product_Num&&Rod_Count>500)//检测长时间遥杆前推一直继续上移
			 {
					Rod_Count=1;
					gs_Current_Operate_List++;
					g_Background_Color=gs_Current_Operate_List%8;
				 if(g_Background_Color==0)
					 g_Background_Color=8;		
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
	  }
			Rod_Count++;				
	}
	else
	{
		Rod_Count=0;
	}
	 /********************************程序编辑页面*****************************************/	
	if(After_filter[0] > 2150)//前推上移
	{
		if(Page_Type==8)
		{
		 if(gs_Current_Operate_List_ProgramEdit==g_Background_Color_ProgramEdit)	//程序的前八行
		 {
			 if(gs_Current_Operate_List_ProgramEdit>1&&Rod_Count==0)//单次上移
			 {
					gs_Current_Operate_List_ProgramEdit--;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List_ProgramEdit>1&&Rod_Count>500)//检测长时间遥杆前推一直继续上移
			 {
					Rod_Count=1;
					gs_Current_Operate_List_ProgramEdit--;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;	
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
		 //不在前八行
		 else
		 {
			 if(gs_Current_Operate_List_ProgramEdit>1&&Rod_Count==0)//单次上移
			 {
					gs_Current_Operate_List_ProgramEdit--;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;
				 //读取前面的LARGESTPROGRAMNUM条程序
				 if(gs_Current_Operate_List_ProgramEdit%LARGESTPROGRAMNUM==0&&Free_Program_Operate.Program[0].List>1)	
				 {
					  //程序载入中
					  LCD_Current_Page = 66;
					  LCD_Pre_Page = LCD_Current_Page;
					  PageChange(LCD_Pre_Page);
						j=Free_Program_Operate.Program[0].List-1;
						for(i=j;i>j-LARGESTPROGRAMNUM;i--)
						{
							AT24CXX_Read(CURRENT_PROGRAM_ADR2+(i-1)*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
							/*List Value1 2 3 4  5 LC_ID Key IO_Group*/
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Flag  = 1;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
							 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
							 {
								Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
							 }
							else if(4==IIC_Temp[26])//延时指令
							{
								Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
							}							
						}
						gs_Current_Operate_List_ProgramEdit=j;
						LCD_Current_Page=8;
				 }
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List_ProgramEdit>1&&Rod_Count>500)//检测长时间遥杆前推一直继续上移
			 {
					Rod_Count=1;
					gs_Current_Operate_List_ProgramEdit--;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;		
				 //读取前面的LARGESTPROGRAMNUM条程序
				 if(gs_Current_Operate_List_ProgramEdit%LARGESTPROGRAMNUM==0&&Free_Program_Operate.Program[0].List>1)	
				 {
						j=Free_Program_Operate.Program[0].List-1;
						for(i=j;i>j-LARGESTPROGRAMNUM;i--)
						{
							AT24CXX_Read(CURRENT_PROGRAM_ADR2+(i-1)*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
							/*List Value1 2 3 4  5 LC_ID Key IO_Group*/
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Flag  = 1;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
							Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
							 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
							 {
								Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
							 }
							else if(4==IIC_Temp[26])//延时指令
							{
								Free_Program_Operate.Program[(i-1)%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
							}							
						}
						LCD_Current_Page=8;						
				 }
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
	  }
		Rod_Count++;				
	}
	else if(After_filter[0] < 1950)//后推下移
	{
		if(Page_Type==8)//产品编辑页面
		{
		 if(gs_Current_Operate_List_ProgramEdit==g_Background_Color_ProgramEdit)	//程序的前八行
		 {
			 if(gs_Current_Operate_List_ProgramEdit<Free_Program_Operate.Num&&Rod_Count==0)//单次下移
			 {
					gs_Current_Operate_List_ProgramEdit++;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;

					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List_ProgramEdit<Free_Program_Operate.Num&&Rod_Count>500)//检测长时间遥杆前推一直继续上移
			 {
					Rod_Count=1;
					gs_Current_Operate_List_ProgramEdit++;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;	
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
		 //不在前八行
		 else
		 {
			 if(gs_Current_Operate_List_ProgramEdit<Free_Program_Operate.Num&&Rod_Count==0)//单次上移
			 {
					gs_Current_Operate_List_ProgramEdit++;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;
				 if((gs_Current_Operate_List_ProgramEdit-1)%LARGESTPROGRAMNUM==0&&Free_Program_Operate.Num>=gs_Current_Operate_List_ProgramEdit)
				 {
					j=Free_Program_Operate.Program[LARGESTPROGRAMNUM-1].List;
					if(Free_Program_Operate.Num-j>LARGESTPROGRAMNUM)
						n=j+LARGESTPROGRAMNUM;
					else
						n=Free_Program_Operate.Num;
					for(i=j;i<n;i++)
					{
						AT24CXX_Read(CURRENT_PROGRAM_ADR2+i*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
					/*List Value1 2 3 4  5 LC_ID Key IO_Group*/
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Flag  = 1;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
						 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
						 {
							Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
						 }
						else if(4==IIC_Temp[26])//延时指令
						{
							Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
						}										
					}
					if(i%LARGESTPROGRAMNUM!=0)
					{
						for(k=i%LARGESTPROGRAMNUM;k<LARGESTPROGRAMNUM;k++)
						{
						/*List key  value1 2 3 4 5*/
							Free_Program_Operate.Program[k].Flag  = 0;
							Free_Program_Operate.Program[k].List  = 0;
							Free_Program_Operate.Program[k].Key   = 0;
							Free_Program_Operate.Program[k].Value1= 0;
							Free_Program_Operate.Program[k].Value2= 0;
							Free_Program_Operate.Program[k].Value3= 0;
							Free_Program_Operate.Program[k].Value4= 0;
							Free_Program_Operate.Program[k].Value5= 0;										
						}
					}
						LCD_Current_Page=8;
				}
					g_Refresh_Dispaly_List_Flag=TRUE;
			 }
			 if(gs_Current_Operate_List_ProgramEdit<Free_Program_Operate.Num&&Rod_Count>500)//检测长时间遥杆前推一直继续上移
			 {
					Rod_Count=1;
					gs_Current_Operate_List_ProgramEdit++;
					g_Background_Color_ProgramEdit=gs_Current_Operate_List_ProgramEdit%8;
				 if(g_Background_Color_ProgramEdit==0)
					 g_Background_Color_ProgramEdit=8;		
				 if((gs_Current_Operate_List_ProgramEdit-1)%LARGESTPROGRAMNUM==0&&Free_Program_Operate.Num>=gs_Current_Operate_List_ProgramEdit)
				 {
					j=Free_Program_Operate.Program[LARGESTPROGRAMNUM-1].List;
					if(Free_Program_Operate.Num-j>LARGESTPROGRAMNUM)
						n=j+LARGESTPROGRAMNUM;
					else
						n=Free_Program_Operate.Num;
					for(i=j;i<n;i++)
					{
						AT24CXX_Read(CURRENT_PROGRAM_ADR2+i*ORDER_LENTH,&IIC_Temp[3],ORDER_LENTH);
						/*List Value1 2 3 4  5 LC_ID Key IO_Group*/
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Flag  = 1;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].List  = IIC_Temp[3]|IIC_Temp[4]<<8;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value1= IIC_Temp[5]|IIC_Temp[6]<<8|IIC_Temp[7]<<16|IIC_Temp[8]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value2= IIC_Temp[9]|IIC_Temp[10]<<8|IIC_Temp[11]<<16|IIC_Temp[12]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value3= IIC_Temp[13]|IIC_Temp[14]<<8|IIC_Temp[15]<<16|IIC_Temp[16]<<24;
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value4= IIC_Temp[17]|IIC_Temp[18]<<8|IIC_Temp[19]<<16|IIC_Temp[20]<<24;	
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Value5= IIC_Temp[21]|IIC_Temp[22]<<8|IIC_Temp[23]<<16|IIC_Temp[24]<<24;		 
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].LC_ID	= IIC_Temp[25];	
						Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Key   = IIC_Temp[26];
						 if(1==IIC_Temp[26]||2==IIC_Temp[26]||3==IIC_Temp[26]||5==IIC_Temp[26])//Order_Type 1 点胶 2点钻 3取料 4 延时 5空移
						 {
							Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].IO_Group  = IIC_Temp[27]|IIC_Temp[28]<<8|IIC_Temp[29]<<16|IIC_Temp[30]<<24;	
						 }
						else if(4==IIC_Temp[26])//延时指令
						{
							Free_Program_Operate.Program[i%LARGESTPROGRAMNUM].Delay_Time  = IIC_Temp[29]|IIC_Temp[30]<<8;				
						}									
					}
					if(i%LARGESTPROGRAMNUM!=0)
					{
						for(k=i%LARGESTPROGRAMNUM;k<LARGESTPROGRAMNUM;k++)
						{
						/*List key  value1 2 3 4 5*/
							Free_Program_Operate.Program[k].Flag  = 0;
							Free_Program_Operate.Program[k].List  = 0;
							Free_Program_Operate.Program[k].Key   = 0;
							Free_Program_Operate.Program[k].Value1= 0;
							Free_Program_Operate.Program[k].Value2= 0;
							Free_Program_Operate.Program[k].Value3= 0;
							Free_Program_Operate.Program[k].Value4= 0;
							Free_Program_Operate.Program[k].Value5= 0;										
						}
					}
						LCD_Current_Page=8;
				}
					g_Refresh_Dispaly_List_Flag=TRUE;				 
			 }
		 }
	  }
			Rod_Count++;				
	}
	else
	{
		Rod_Count=0;
	}	
}
/*************************************************************************
**  函数名：  Rod_Axis_Contrl
**	输入参数：
**	输出参数：无
**	函数功能：遥杆运动控制
**	备注：	  无
**  作者：    White_L   
**  开发日期：2018/12/02
**************************************************************************/
void Rod_Axis_Contrl(u8 Rod_Speed_Rank,u8 Axis_DIR,u8 Axis)
{
	if(Axis_DIR)
	{
		  if(Link_Step_Flag==0)//联动模式
		  {
			if(g_Axis_Is_Run==0||(g_Axis_Is_Run!=0&&Axsis_Move_Direction[Axis]==1&&gs_LinkMove_Speed[Axis]!=Rod_Speed_Rank))
			{
				 Axis_Position_Scan=TRUE;		 					                    //自动运行时，轴位置查询
				gs_LinkMove_Speed[Axis] = Rod_Speed_Rank;
				Order_Temp[0]=P_ACTION_SPEEDADJ;            //遥杆倾角变大速度变快
				Order_Temp[1]=gs_LinkMove_Speed[Axis];            //
				Order_Temp[2]=Axis;            //		
				Order_Temp[3]=1;            			//	方向								
				USART2_SendDataArray(4,MANUL_OPERATE,&Order_Temp[0]);//串口发送机械手自动运行中的启动到主控板
			}
			else if(g_Axis_Is_Run!=0&&Axsis_Move_Direction[Axis]==0)
			{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				switch(Axis)
				{
					case X_Axsis:
						Order_Temp[0] =XAXIS_MOVE_LEFT;        //X轴寸动右移
					break;
					case L_Axsis:
						Order_Temp[0] =LAXIS_MOVE_LEFT;        //X轴寸动右移
					break;	
					case Z_Axsis:
						Order_Temp[0] =ZAXIS_MOVE_LEFT;        //X轴寸动右移
					break;
					case O_Axsis:
						Order_Temp[0] =OAXIS_MOVE_LEFT;        //X轴寸动右移
					break;
					case A_Axsis:
						Order_Temp[0] =AAXIS_MOVE_LEFT;        //X轴寸动右移
					break;
					case B_Axsis:
						Order_Temp[0] =BAXIS_MOVE_LEFT;        //X轴寸动右移
					break;
				}
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X轴停止								
			}
		}
		else//连续寸动模式
		{
//			if(Step_Period_Count>=5)//每隔50ms寸动一次
//			{
//				Step_Period_Count=0;
//				switch(Axis)
//				{
//					case X_Axsis:
//						Order_Temp[0] =XAXIS_MOVE_RIGHT;        //X轴寸动右移
//					break;
//					case L_Axsis:
//						Order_Temp[0] =LAXIS_MOVE_RIGHT;        //X轴寸动右移
//					break;	
//					case Z_Axsis:
//						Order_Temp[0] =ZAXIS_MOVE_RIGHT;        //X轴寸动右移
//					break;
//					case O_Axsis:
//						Order_Temp[0] =OAXIS_MOVE_RIGHT;        //X轴寸动右移
//					break;
//					case A_Axsis:
//						Order_Temp[0] =AAXIS_MOVE_RIGHT;        //X轴寸动右移
//					break;
//					case B_Axsis:
//						Order_Temp[0] =BAXIS_MOVE_RIGHT;        //X轴寸动右移
//					break;
//				}
//			 	 Order_Temp[1] = JOG_MOVE_MODE;	            //寸动模式
//				 Order_Temp[2] = 1;	
//				 USART1_SendData(3,MANUL_OPERATE,&Order_Temp[0]);//通信：低位在前-高位在后
//				 Axis_Position_Scan=TRUE;                   //自动运行时，轴位置查询
//			}				
		}			
	}
	else
	{
		  if(Link_Step_Flag==0)//联动模式
		  {
			if(g_Axis_Is_Run==0||(g_Axis_Is_Run!=0&&Axsis_Move_Direction[Axis]==0&&gs_LinkMove_Speed[Axis]!=Rod_Speed_Rank))
			{
				 Axis_Position_Scan=TRUE;		 					                    //自动运行时，轴位置查询
				gs_LinkMove_Speed[Axis] = Rod_Speed_Rank;
				Order_Temp[0]=P_ACTION_SPEEDADJ;            //遥杆倾角变大速度变快
				Order_Temp[1]=gs_LinkMove_Speed[Axis];            //
				Order_Temp[2]=Axis;            //		
				Order_Temp[3]=0;            //	方向								
				USART2_SendDataArray(4,MANUL_OPERATE,&Order_Temp[0]);//串口发送机械手自动运行中的启动到主控板
			}		
			else if(g_Axis_Is_Run!=0&&Axsis_Move_Direction[Axis]==1)
			{
			 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
			switch(Axis)
			{
				case X_Axsis:
					Order_Temp[0] =XAXIS_MOVE_RIGHT;        //X轴寸动右移
				break;
				case L_Axsis:
					Order_Temp[0] =LAXIS_MOVE_RIGHT;        //X轴寸动右移
				break;	
				case Z_Axsis:
					Order_Temp[0] =ZAXIS_MOVE_RIGHT;        //X轴寸动右移
				break;
				case O_Axsis:
					Order_Temp[0] =OAXIS_MOVE_RIGHT;        //X轴寸动右移
				break;
				case A_Axsis:
					Order_Temp[0] =AAXIS_MOVE_RIGHT;        //X轴寸动右移
				break;
				case B_Axsis:
					Order_Temp[0] =BAXIS_MOVE_RIGHT;        //X轴寸动右移
				break;
			}
			 Order_Temp[1] = LINK_MOVE_MODE;	                   
			 Order_Temp[2] = 0;
			 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X轴停止								
			}
		 }	
		else//连续寸动模式
		{
//			if(Step_Period_Count>=5)//每隔50ms寸动一次
//			{
//				Step_Period_Count=0;
//				switch(Axis)
//				{
//					case X_Axsis:
//						Order_Temp[0] =XAXIS_MOVE_LEFT;        //X轴寸动右移
//					break;
//					case L_Axsis:
//						Order_Temp[0] =LAXIS_MOVE_LEFT;        //X轴寸动右移
//					break;	
//					case Z_Axsis:
//						Order_Temp[0] =ZAXIS_MOVE_LEFT;        //X轴寸动右移
//					break;
//					case O_Axsis:
//						Order_Temp[0] =OAXIS_MOVE_LEFT;        //X轴寸动右移
//					break;
//					case A_Axsis:
//						Order_Temp[0] =AAXIS_MOVE_LEFT;        //X轴寸动右移
//					break;
//					case B_Axsis:
//						Order_Temp[0] =BAXIS_MOVE_LEFT;        //X轴寸动右移
//					break;
//				}
//			 	 Order_Temp[1] = JOG_MOVE_MODE;	            //寸动模式
//				 Order_Temp[2] = 1;	
//				 USART1_SendData(3,MANUL_OPERATE,&Order_Temp[0]);//通信：低位在前-高位在后
//				 Axis_Position_Scan=TRUE;                   //自动运行时，轴位置查询
//			}			
		}				
	}
}
/*************************************************************************
**  函数名：  Rob_Move
**	输入参数：
**	输出参数：无
**	函数功能：求ADC均值并发送对应的运动指令，停止指令马上发送，运动指令在MANULOPERATE处理
**	备注：	  无
**  作者：    DH   
**  开发日期：2018/8/02
**************************************************************************/
//遥杆对应的运动方式：800  2100-2900:寸动移动，  
//												 			寸动--->寸动  等电机停下再进行寸动
//												 			联动--->寸动	等电机停下再进行寸动
//										600  2900-3500:联动速度V1 达到速度V1以后（匀速运动阶段）开始接收遥杆速度改变	
//															联动--->联动	匀速后开始变速
//															寸动--->联动	电机停止后进行联动
//										500  3500-4000:联动速度V2 达到速度V2以后（匀速运动阶段）开始接收遥杆速度改变
//										100  4000-4096:联动速度V3 达到速度V3以后（匀速运动阶段）开始接收遥杆速度改变

void Rob_Move(void)
{    
	int  sum = 0,i=0;
	 u8  count;       
	 for(i=0;i<2;i++)         
	 {            
			for ( count=0;count<10;count++)            
			{   
				sum += AD_Value[count][i];             
			}            
			After_filter[i]=sum/10;            
			sum=0;       
	 }
	 if(((After_filter[0]) >= Rod_Step_X[4]) || ((After_filter[0]) <= Rod_Step_X[3]) || ((After_filter[1]) >= Rod_Step_Y[4]) || ((After_filter[1]) <= Rod_Step_Y[3]))
	 {
/**************************遥杆的一个方向控制XZA轴动作************************************/
		if((abs(After_filter[0]-Rod_Centry_X))>= (abs(After_filter[1]-Rod_Centry_Y)))
		{	
			if((After_filter[0] >= Rod_Step_X[4]&&After_filter[0]<Rod_Step_X[5]))
			{
				Rod_Speed_Rank=5;
				Axis_DIR=1;
			}
			if(After_filter[0] <= Rod_Step_X[3]&&After_filter[0]>Rod_Step_X[2])
			{
				Rod_Speed_Rank=5;
				Axis_DIR=0;
			}
		  if(After_filter[0] >= Rod_Step_X[5]&&After_filter[0] < Rod_Step_X[6])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]>>2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]>>2;				
				Axis_DIR=1;			
			}
			if(After_filter[0] <= Rod_Step_X[2]&&After_filter[0]>Rod_Step_X[1])
			{
				Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]>>2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]>>2;
				Axis_DIR=0;					
			}
			if(After_filter[0] >= Rod_Step_X[6]&&After_filter[0] < Rod_Step_X[7])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=1;			
			}
			if(After_filter[0] <= Rod_Step_X[1]&&After_filter[0]>Rod_Step_X[0])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=0;					
			}
			if(After_filter[0] >= Rod_Step_X[7])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=1;					
			}
			if(After_filter[0] <= Rod_Step_X[0])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=0;					
			}
/**********************************----遥杆速度控制V1 2 3 4-----****************************************/
		switch(LCD_Current_Page)
			{
				case 148://X
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,X_Axsis);
					break;
				case 149://Z
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,Z_Axsis);
					break;
				case 150://A
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,A_Axsis);
					break;
				case 0x04://X
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,X_Axsis);
					break;
				case 0x09://Z
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,Z_Axsis);
					break;
				case 0x0A://A
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,A_Axsis);
					break;
				default:
					break;
			}
		}
/**************************遥杆的一个方向控制YOB轴动作************************************/
		else
		{
			if((After_filter[1] >= Rod_Step_Y[4]&&After_filter[1]<Rod_Step_Y[5]))
			{
				Rod_Speed_Rank=5;
				Axis_DIR=1;
			}
			if(After_filter[1] <= Rod_Step_Y[3]&&After_filter[1]>Rod_Step_Y[2])
			{
				Rod_Speed_Rank=5;
				Axis_DIR=0;
			}
		  if(After_filter[1] >= Rod_Step_Y[5]&&After_filter[1] < Rod_Step_Y[6])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]>>2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]>>2;
				Axis_DIR=1;			
			}
			if(After_filter[1] <= Rod_Step_Y[2]&&After_filter[1]>Rod_Step_Y[1])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]>>2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]>>2;
				Axis_DIR=0;					
			}
			if(After_filter[1] >= Rod_Step_Y[6]&&After_filter[1] < Rod_Step_Y[7])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=1;			
			}
			if(After_filter[1] <= Rod_Step_Y[1]&&After_filter[1]>Rod_Step_Y[0])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=0;					
			}
			if(After_filter[1] >= Rod_Step_Y[7])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=1;					
			}
			if(After_filter[1] <= Rod_Step_Y[0])
			{
				if(LCD_Current_Page==150||LCD_Current_Page==0x0A)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=0;					
			}
/**********************************----遥杆速度控制V1 2 3 4-----****************************************/
		switch(LCD_Current_Page)
			{
				case 148://Y
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,L_Axsis);
					break;
				case 149://O
//						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,O_Axsis);
					break;
				case 150://B
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,B_Axsis);
					break;
				case 0x04://Y
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,L_Axsis);
					break;
				case 0x09://O
//						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,O_Axsis);
					break;
				case 0x0A://B
						Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,B_Axsis);
					break;
				default:
					break;
			}			
		}
		LCD_RealTime_Watch=TRUE;
		Axis_Position_Scan=TRUE;                   //自动运行时，轴位置查询
	}
	else
	{
		switch(LCD_Current_Page)
		{
			case 148://X
				if(Axsis_Move_Direction[X_Axsis]==1&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = XAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X轴停止					
				}
				if(Axsis_Move_Direction[X_Axsis]==0&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = XAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X轴停止					
				}
				DelayNms(20);
				if(Axsis_Move_Direction[L_Axsis]==1&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = LAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y轴停止	
				}
				if(Axsis_Move_Direction[L_Axsis]==0&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = LAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y轴停止					
				}
				break;
			case 149://O
				if(Axsis_Move_Direction[O_Axsis]==1&&(g_Axis_Is_Run&0x08)==0x08)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = OAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0轴停止					
				}
				if(Axsis_Move_Direction[O_Axsis]==0&&(g_Axis_Is_Run&0x08)==0x08)
				{			
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = OAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0轴停止						
				}		
				DelayNms(20);
				if(Axsis_Move_Direction[Z_Axsis]==1&&(g_Axis_Is_Run&0x02)==0x02)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = ZAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z轴停止						
				}
				if(Axsis_Move_Direction[Z_Axsis]==0&&(g_Axis_Is_Run&0x02)==0x02)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = ZAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z轴停止						
				}
				break;
			case 150:
				if(Axsis_Move_Direction[A_Axsis]==1&&(g_Axis_Is_Run&0x10)==0x10)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = AAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V轴停止		
				}
				if(Axsis_Move_Direction[A_Axsis]==0&&(g_Axis_Is_Run&0x10)==0x10)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = AAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V轴停止						
				}
				DelayNms(20);
				if(Axsis_Move_Direction[B_Axsis]==1&&(g_Axis_Is_Run&0x20)==0x20)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = BAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W轴停止	
				}	
				if(Axsis_Move_Direction[B_Axsis]==0&&(g_Axis_Is_Run&0x20)==0x20)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = BAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W轴停止
				}							
				break;
			case 0x04://X
				if(Axsis_Move_Direction[X_Axsis]==1&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = XAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X轴停止					
				}
				if(Axsis_Move_Direction[X_Axsis]==0&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = XAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X轴停止					
				}
				DelayNms(20);
				if(Axsis_Move_Direction[L_Axsis]==1&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = LAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y轴停止	
				}
				if(Axsis_Move_Direction[L_Axsis]==0&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = LAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y轴停止					
				}
				break;
			case 0x09://O
				if(Axsis_Move_Direction[O_Axsis]==1&&(g_Axis_Is_Run&0x08)==0x08)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = OAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0轴停止					
				}
				if(Axsis_Move_Direction[O_Axsis]==0&&(g_Axis_Is_Run&0x08)==0x08)
				{			
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = OAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0轴停止						
				}		
				DelayNms(20);
				if(Axsis_Move_Direction[Z_Axsis]==1&&(g_Axis_Is_Run&0x02)==0x02)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = ZAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z轴停止						
				}
				if(Axsis_Move_Direction[Z_Axsis]==0&&(g_Axis_Is_Run&0x02)==0x02)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = ZAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z轴停止						
				}
				break;
			case 0x0A:
				if(Axsis_Move_Direction[A_Axsis]==1&&(g_Axis_Is_Run&0x10)==0x10)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = AAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V轴停止		
				}
				if(Axsis_Move_Direction[A_Axsis]==0&&(g_Axis_Is_Run&0x10)==0x10)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = AAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V轴停止						
				}
				DelayNms(20);
				if(Axsis_Move_Direction[B_Axsis]==1&&(g_Axis_Is_Run&0x20)==0x20)
				{
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = BAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W轴停止	
				}	
				if(Axsis_Move_Direction[B_Axsis]==0&&(g_Axis_Is_Run&0x20)==0x20)
				{	
				 Axis_Position_Scan=FALSE;		 					                    //自动运行时，轴位置查询
				 g_Axis_Is_Run = 0;
				 Order_Temp[0] = BAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W轴停止
				}							
				break;
		}		
	}

}
/*********************************************************************************************************
** 函数名称 ：Rod_Adjust()  
** 函数功能 ：遥杆校准
** 入口参数 ：
** 出口参数 ：无
** 作者			：White_L
********************************************************************************************************/
int Rod_Display_X=0,Rod_Display_Y=0;
void Rod_Adjust(void)
{
	int  sum = 0,i=0;
	 u8  count;       
	 for(i=0;i<2;i++)         
	 {            
			for ( count=0;count<10;count++)            
			{   
				sum += AD_Value[count][i];             
			}            
			After_filter[i]=sum/10;            
			sum=0;       
	 }	
	 if(((After_filter[0]) >= Rod_Step_X[4]) || ((After_filter[0]) <= Rod_Step_X[3]) || ((After_filter[1]) >= Rod_Step_Y[4]) || ((After_filter[1]) <= Rod_Step_Y[3]))
	 {
		if((abs(After_filter[0]-Rod_Centry_X))>= (abs(After_filter[1]-Rod_Centry_Y)))
		{
			if(After_filter[0] <= Rod_Step_X[7])//正方向
			{
				Rod_Display_X = ((After_filter[0]-Rod_Centry_X)/20);
			}
			if(After_filter[0] >= Rod_Step_X[7])
			{
				Rod_Display_X=100;
			}
			if(After_filter[0] <= Rod_Step_X[3]&&After_filter[0] >= Rod_Step_X[0])//反方向
			{
				Rod_Display_X = -(int)((Rod_Centry_X-After_filter[0])/20);
			}
			if(After_filter[0] <= Rod_Step_X[0])
			{
				Rod_Display_X=-100;
			}			
		}
		else
		{
			if(After_filter[1] <= Rod_Step_Y[7])//正方向
			{
				Rod_Display_Y = ((After_filter[1]-Rod_Centry_X)/20);
			}
			if(After_filter[1] >= Rod_Step_Y[7])
			{
				Rod_Display_Y=100;
			}
			if(After_filter[1] <= Rod_Step_Y[3]&&After_filter[1] >= Rod_Step_Y[0])//反方向
			{
				Rod_Display_Y = -(int)((Rod_Centry_X-After_filter[1])/20);
			}	
			if(After_filter[1] <= Rod_Step_Y[0])
			{
				Rod_Display_Y=-100;
			}			
		}
	 }
	 else//显示0
	 {
		 Rod_Display_X=0;
		 Rod_Display_Y=0;
	 }
}
/*************************************************************************
**  函数名：  Rob_Link_Move
**	输入参数：
**	输出参数：无
**	函数功能：求ADC均值并发送对应的运动指令，停止指令马上发送，运动指令在MANULOPERATE处理
**	备注：	  无
**  作者：    DH   
**  开发日期：2018/8/02
**************************************************************************/
extern u8 Order_Temp[];
extern u8 AXIS_Choose;
void Rob_Link_Move(void)
{ 
	int  sum = 0,i=0;
	 u8  count;       
	 for(i=0;i<2;i++)         
	 {            
			for ( count=0;count<10;count++)            
			{   
				sum += AD_Value[count][i];             
			}            
			After_filter[i]=sum/10;            
			sum=0;       
	 }
	 if(((After_filter[0]) >= Rod_Step_X[4]) || ((After_filter[0]) <= Rod_Step_X[3]) || ((After_filter[1]) >= Rod_Step_Y[4]) || ((After_filter[1]) <= Rod_Step_Y[3]))
	 {
/**************************遥杆的一个方向控制XZA轴动作************************************/
		if((abs(After_filter[0]-Rod_Centry_X))>= (abs(After_filter[1]-Rod_Centry_Y)))
		{	
			if((After_filter[0] >= Rod_Step_X[4]&&After_filter[0]<Rod_Step_X[5]))
			{
				Rod_Speed_Rank=1;
				Axis_DIR=1;
			}
			if(After_filter[0] <= Rod_Step_X[3]&&After_filter[0]>Rod_Step_X[2])
			{
				Rod_Speed_Rank=1;
				Axis_DIR=0;
			}
		  if(After_filter[0] >= Rod_Step_X[5]&&After_filter[0] < Rod_Step_X[6])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];				
				Axis_DIR=1;			
			}
			if(After_filter[0] <= Rod_Step_X[2]&&After_filter[0]>Rod_Step_X[1])
			{
				Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=0;					
			}
			if(After_filter[0] >= Rod_Step_X[6]&&After_filter[0] < Rod_Step_X[7])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=1;			
			}
			if(After_filter[0] <= Rod_Step_X[1]&&After_filter[0]>Rod_Step_X[0])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<1;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<1;
				Axis_DIR=0;					
			}
			if(After_filter[0] >= Rod_Step_X[7])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<2;
				Axis_DIR=1;					
			}
			if(After_filter[0] <= Rod_Step_X[0])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[1]<<2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]<<2;
				Axis_DIR=0;					
			}
/**********************************----遥杆速度控制V1 2 3 4-----****************************************/
		switch(LCD_Current_Page)
			{
				case 4://X
						if(Axis_DIR==0&&g_XAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,X_Axsis);
						}
					break;
				case 9://Z
						if(Axis_DIR==0&&g_ZAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,Z_Axsis);
						}
					break;
				case 10://V
						if(Axis_DIR==0&&g_AAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(1,Axis_DIR,A_Axsis);
						}
					break;
				default:
					break;
			}
		}
/**************************遥杆的一个方向控制YOB轴动作************************************/
		else
		{
			if((After_filter[1] >= Rod_Step_Y[4]&&After_filter[1]<Rod_Step_Y[5]))
			{
				Rod_Speed_Rank=1;
				Axis_DIR=1;
			}
			if(After_filter[1] <= Rod_Step_Y[3]&&After_filter[1]>Rod_Step_Y[2])
			{
				Rod_Speed_Rank=1;
				Axis_DIR=0;
			}
		  if(After_filter[1] >= Rod_Step_Y[5]&&After_filter[1] < Rod_Step_Y[6])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]/4;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]/4;
				Axis_DIR=1;			
			}
			if(After_filter[1] <= Rod_Step_Y[2]&&After_filter[1]>Rod_Step_Y[1])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]/4;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]/4;
				Axis_DIR=0;					
			}
			if(After_filter[1] >= Rod_Step_Y[6]&&After_filter[1] < Rod_Step_Y[7])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]/2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]/2;
				Axis_DIR=1;			
			}
			if(After_filter[1] <= Rod_Step_Y[1]&&After_filter[1]>Rod_Step_Y[0])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2]/2;
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0]/2;
				Axis_DIR=0;					
			}
			if(After_filter[1] >= Rod_Step_Y[7])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=1;					
			}
			if(After_filter[1] <= Rod_Step_Y[0])
			{
				if(LCD_Current_Page==10)
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[2];
				else
					Rod_Speed_Rank=DZJSpeed1.ManualSpeed_H[0];
				Axis_DIR=0;					
			}
/**********************************----遥杆速度控制V1 2 3 4-----****************************************/
		switch(LCD_Current_Page)
			{
				case 4://Y
						if(Axis_DIR==0&&g_LAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,L_Axsis);
						}
					break;
				case 9://O
						if(Axis_DIR==0&&g_OAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(Rod_Speed_Rank,Axis_DIR,O_Axsis);
						}
					break;
				case 10://W
						if(Axis_DIR==0&&g_BAxis_Position==0)
						{}
						else
						{
							Rod_Axis_Contrl(1,Axis_DIR,B_Axsis);
						}
					break;
			}			
		}
		LCD_RealTime_Watch=TRUE;
		Axis_Position_Scan=TRUE;                   //自动运行时，轴位置查询
	}
	else
	{
		switch(LCD_Current_Page)
		{
			case 4://X
				if(Axsis_Move_Direction[X_Axsis]==1&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Order_Temp[0] = XAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X轴停止					
				}
				if(Axsis_Move_Direction[X_Axsis]==0&&(g_Axis_Is_Run&0x01)==0x01)
				{
				 Order_Temp[0] = XAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //X轴停止					
				}
				DelayNms(100);
				if(Axsis_Move_Direction[L_Axsis]==1&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Order_Temp[0] = LAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y轴停止	
				}
				if(Axsis_Move_Direction[L_Axsis]==0&&(g_Axis_Is_Run&0x04)==0x04)
				{
				 Order_Temp[0] = LAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Y轴停止					
				}
				break;
			case 9://O
				if(Axsis_Move_Direction[O_Axsis]==1&&(g_Axis_Is_Run&0x08)==0x08)
				{
				 Order_Temp[0] = OAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0轴停止					
				}
				if(Axsis_Move_Direction[O_Axsis]==0&&(g_Axis_Is_Run&0x08)==0x08)
				{			
				 Order_Temp[0] = OAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //0轴停止						
				}		
				if(Axsis_Move_Direction[Z_Axsis]==1&&(g_Axis_Is_Run&0x02)==0x02)
				{
				 Order_Temp[0] = ZAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z轴停止						
				}
				if(Axsis_Move_Direction[Z_Axsis]==0&&(g_Axis_Is_Run&0x02)==0x02)
				{	
				 Order_Temp[0] = ZAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //Z轴停止						
				}
				break;
			case 10:
				if(Axsis_Move_Direction[A_Axsis]==1&&(g_Axis_Is_Run&0x10)==0x10)
				{
				 Order_Temp[0] = AAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V轴停止		
				}
				if(Axsis_Move_Direction[A_Axsis]==0&&(g_Axis_Is_Run&0x10)==0x10)
				{	
				 Order_Temp[0] = AAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //V轴停止						
				}
				if(Axsis_Move_Direction[B_Axsis]==1&&(g_Axis_Is_Run&0x20)==0x20)
				{
				 Order_Temp[0] = BAXIS_MOVE_RIGHT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W轴停止	
				}	
				if(Axsis_Move_Direction[B_Axsis]==0&&(g_Axis_Is_Run&0x20)==0x20)
				{	
				 Order_Temp[0] = BAXIS_MOVE_LEFT;                     
				 Order_Temp[1] = LINK_MOVE_MODE;	                   
				 Order_Temp[2] = 0;
				 USART2_SendDataArray(3,MANUL_OPERATE,&Order_Temp[0]); //W轴停止
				}							
				break;
		}		
	}

}
/*******************************************************************************
* Function Name  : USART1_DMA_Configuration
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/	
void USART1_DMA_Configuration(void)
{
    USART_ClockInitTypeDef  USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef   DMA_InitStructure;


	DMA_DeInit(DMA1_Channel4);    
    DMA_InitStructure.DMA_PeripheralBaseAddr = SRC_USART1_DR;  
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART1SendBuff;  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  
    DMA_InitStructure.DMA_BufferSize = 0;//sizeof(g_SendBuffer);  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  
    DMA_Init(DMA1_Channel4, &DMA_InitStructure); 

	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART1, &USART_ClockInitStructure);

	USART_InitStructure.USART_BaudRate            = 115200;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	/* Enable USART1 DMA Txrequest */  
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  
    /* Enable DMA1 Channel4 */  
    DMA_Cmd(DMA1_Channel4, ENABLE); 
}

/*******************************************************************************
* Function Name  : USART1_Robot_Configuration
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_Robot_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART1, &USART_ClockInitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	USART_ClearITPendingBit(USART1, USART_IT_TC);

}

/*******************************************************************************
* Function Name  : USART2_485_Configuration
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_485_Configuration()
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART2, &USART_ClockInitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	USART_ClearITPendingBit(USART2, USART_IT_TC);
}
/*******************************************************************************
* Function Name  : USART3_LCD_Configuration
* Description    : 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_LCD_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
	USART_ClockInit(USART3, &USART_ClockInitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
	USART_ClearITPendingBit(USART3, USART_IT_TC);

}



void TimeInit(void)
{ 
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;
	TIM_ICInitTypeDef TIM_ICInitStructure;                       
 	/**************************************************************************
 	//     定时器4设置： 720分频，向上计数，计数值为1000->10ms	  
 	***************************************************************************/
 	TIM_TimeBaseStructure.TIM_Period = 1000;
  	TIM_TimeBaseStructure.TIM_Prescaler = 719;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

 	/**************************************************************************
 	//     定时器3设置： 正交编码器功能	  	
 	***************************************************************************/
 	TIM_TimeBaseStructure.TIM_Period = 0xffff;
  	TIM_TimeBaseStructure.TIM_Prescaler = 0;
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI1,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);   //上升沿捕捉
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0x06;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	TIM_ClearFlag(TIM3, TIM_FLAG_Update); 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
//	TIM3->CNT =0;	      //清除定时器
//	TIM_Cmd(TIM3, ENABLE);//使能定时器  
//	TIM_GetCounter(TIM3); //读取脉冲数
}

void RTC_Configuration()
{
	  /* Enable PWR and BKP clocks */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	  
	  /* Allow access to BKP Domain */
	  PWR_BackupAccessCmd(ENABLE);
	
	  /* Reset Backup Domain */
	  BKP_DeInit();
	  /* Enable LSE */
	  RCC_LSEConfig(RCC_LSE_ON);
	  /* Wait till LSE is ready */
	  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	  {
	  }
	
	  /* Select LSE as RTC Clock Source */
	  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  
	  /* Enable RTC Clock */
	  RCC_RTCCLKCmd(ENABLE);
	
	
	#ifdef RTCClockOutput_Enable  
	  /* Disable the Tamper Pin */
	  BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
	                               functionality must be disabled */
	                               
	  /* Enable RTC Clock Output on Tamper Pin */
	  BKP_RTCCalibrationClockOutputCmd(ENABLE);
	#endif 
	
	  /* Wait for RTC registers synchronization */
	  RTC_WaitForSynchro();
	
	  /* Wait until last write operation on RTC registers has finished */
	  RTC_WaitForLastTask();
	  
	  /* Enable the RTC Second */  
	 // RTC_ITConfig(RTC_IT_SEC, ENABLE);
	
	  /* Wait until last write operation on RTC registers has finished */
	  RTC_WaitForLastTask();
	  RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
	  
	  /* Wait until last write operation on RTC registers has finished */
	  RTC_WaitForLastTask();
}

/*******************************************************************************
* Function Name  : Time_Adjust
* Description    : Adjusts time.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Time_Adjust(void)
{
	  /* Wait until last write operation on RTC registers has finished */
	  RTC_WaitForLastTask(); 
	  /* Change the current time */
	  RTC_SetCounter(0x00000000);
	  /* Wait until last write operation on RTC registers has finished */
	  RTC_WaitForLastTask();   
}


void RTC_Init()
{
	u32 temp=0;
	temp=BKP_ReadBackupRegister(BKP_DR1);
	  if( temp!= 0xA5A5)
	  {
	  	Check_battery=1;			  //电池被拔掉过的标志位(每次进来都判断为没有电池)
	    RTC_Configuration();
	    /* Adjust time by values entred by the user on the hyperterminal */
	    Time_Adjust();
	    BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);    
	  }
	  else
	  {
	    
	    /* Wait for RTC registers synchronization */
	    RTC_WaitForSynchro();
	    /* Wait until last write operation on RTC registers has finished */
	    RTC_WaitForLastTask();
	  }
}


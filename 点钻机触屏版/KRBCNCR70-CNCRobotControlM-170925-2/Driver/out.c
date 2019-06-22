/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ****************
* File Name          : out.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This file provides all the GPIO output functions.
******************************************************************************/

#include "stm32f10x_lib.h"
#include "out.h"
#include "Delay.h"
#include "Manual.h"	 

/*-----------------------------------------------------------------------------------
--  全局变量名：       Output_Status
--  变量作用：         4个成员分别用于记录27个输出信号端口
--  变量有效范围：	   0x00-0xff
--  变量使用范围： 	   SetOutput函数中具有“写”操作权
--  其它：			   变量的每一位表示一个端口，共32个端口
------------------------------------------------------------------------------------*/
u8 Output_Status[5] = {0xff, 0xff, 0xff , 0xff, 0xff};



/**************************************************************************************************
**  函数名：  SetOutput()
**	输入参数：u8 IO_Num:需要输出的端口  
**	输出参数：无
**	函数功能：置位端口电平信号
**	备注：	  
**  作者：    吴祥      
**  开发日期：2013/12/17 
***************************************************************************************************/
void SetOutput(u8 IO_Num)
{
	switch (IO_Num-1)
	{
		case 0://Y0-PE1		1-上料完成
			 GPIO_SetBits(GPIOB,GPIO_Pin_5); 
		     Delay(1);		  
		     Output_Status[0] |= 0x01;
			 break;

		case 1://Y1-PE0
			 GPIO_SetBits(GPIOG,GPIO_Pin_15); 
			 Delay(1);			 
			 Output_Status[0] |= 0x02;	 
			 break;

		case 2://Y2-PB5 
	 	     GPIO_SetBits(GPIOG,GPIO_Pin_14); 
			 Delay(1);	  
			 Output_Status[0] |= 0x04;	
			 break;
	
		case 3://Y3-PG15
	         GPIO_SetBits(GPIOG,GPIO_Pin_13); 
			 Delay(1);	  
			 Output_Status[0] |= 0x08;	
			 break;

		case 4://Y4-PG14 		1-机床1卡盘A开
			 GPIO_SetBits(GPIOG,GPIO_Pin_12); 
			 Delay(1);	  
			 Output_Status[0] |= 0x10;	
			 break;

		case 5://Y5-PG13 			
			 GPIO_SetBits(GPIOG,GPIO_Pin_11); 
			 Delay(1);	  
			 Output_Status[0] |= 0x20;	
			 break;
		
		case 6://Y6-PG12 			
 	         GPIO_SetBits(GPIOG,GPIO_Pin_10); 
			 Delay(1);	  
			 Output_Status[0] |= 0x40;	
			 break;
		case 7://Y7-PG11 
	         GPIO_SetBits(GPIOG,GPIO_Pin_9); 
			 Delay(1);	  
			 Output_Status[0] |= 0x80;
			 break;

		case 8://Y8-PG10
  	         GPIO_SetBits(GPIOD,GPIO_Pin_7); 
			 Delay(1);		  
			 Output_Status[1] |= 0x01;
			 break;

		case 9://Y9-PG9
			 GPIO_SetBits(GPIOD,GPIO_Pin_6); 
			 Delay(1);			 
			 Output_Status[1] |= 0x02; 
			 break;

		case 10://Y10-PD7 
			 GPIO_SetBits(GPIOD,GPIO_Pin_5); 
			 Delay(1);	  
			 Output_Status[1] |= 0x04;	
			 break;
	
		case 11://Y11-PD6 		  机床1输出预留
	         GPIO_SetBits(GPIOD,GPIO_Pin_4); 
			 Delay(1);	  
			 Output_Status[1] |= 0x08;	
			 break;

		case 12://Y12-PD5 		  2-机床2卡盘A开
			 GPIO_SetBits(GPIOD,GPIO_Pin_3); 
			 Delay(1);	  
			 Output_Status[1] |= 0x10;
			 break;

		case 13://Y13-PD4 
  	         GPIO_SetBits(GPIOD,GPIO_Pin_1); 
		     Delay(1);	  
			 Output_Status[1] |= 0x20;
			 break;
		
		case 14://Y14-PD3 
		  	 GPIO_SetBits(GPIOD,GPIO_Pin_0); 
			 Delay(1);	  
			 Output_Status[1] |= 0x40;
			 break;

		case 15://Y15-PD2 
			 GPIO_SetBits(GPIOA,GPIO_Pin_8); 
			 Delay(1);	  
			 Output_Status[1] |= 0x80;	
			 break;

 		case 16://Y16-PD1
		  	 GPIO_SetBits(GPIOG,GPIO_Pin_8); 
			 Delay(1);		  
			 Output_Status[2] |= 0x01;
			 break;

		case 17://Y17-PD0
		  	 GPIO_SetBits(GPIOG,GPIO_Pin_7); 
			 Delay(1);			 
			 Output_Status[2] |= 0x02;
             break;

		case 18://Y18-PC12 
		  	 GPIO_SetBits(GPIOG,GPIO_Pin_6); 
			 Delay(1);	  
			 Output_Status[2] |= 0x04;
			 break;
	
		case 19://Y19-PC11 
			 GPIO_SetBits(GPIOG,GPIO_Pin_5); 
			 Delay(1);	  
			 Output_Status[2] |= 0x08;	
			 break;

		case 20://Y20-PC10 		2-翻转台正摆
		  	 GPIO_SetBits(GPIOG,GPIO_Pin_4); 
			 Delay(1);	  
			 Output_Status[2] |= 0x10;	
			 break;

		case 21://Y21-PA8 
		  	 GPIO_SetBits(GPIOG,GPIO_Pin_3); 
			 Delay(1);	  
			 Output_Status[2] |= 0x20;	
			 break;
		
		case 22://Y22-PC9 
			 GPIO_SetBits(GPIOG,GPIO_Pin_2);
			 Delay(1);	  
			 Output_Status[2] |= 0x40;	
			 break;

		case 23://Y23-PC8
			 GPIO_SetBits(GPIOD,GPIO_Pin_15);
			 Delay(1);	  
			 Output_Status[2] |= 0x80;	
			 break;

    	case 24://Y24-PC7		 3-上料爪开
			 GPIO_SetBits(GPIOD,GPIO_Pin_14);
			 Delay(1);		  
			 Output_Status[3] |= 0x01;
			 break;

		case 25://Y25-PC6
			 GPIO_SetBits(GPIOD,GPIO_Pin_13);
			 Delay(1);			 
			 Output_Status[3] |= 0x02;	 
			 break;

		case 26://Y26-PG8
			 GPIO_SetBits(GPIOD,GPIO_Pin_12);
			 Delay(1);	  
			 Output_Status[3] |= 0x04;	
			 break;
	
		case 27://Y27-PG7
			 GPIO_SetBits(GPIOD,GPIO_Pin_11);
			 Delay(1);	  
			 Output_Status[3] |= 0x08;	
			 break;

		case 28://Y28-PG6 
			 GPIO_SetBits(GPIOD,GPIO_Pin_10);
			 Delay(1);	  
			 Output_Status[3] |= 0x10;	
			 break;

		case 29://Y29-PG5 
			 GPIO_SetBits(GPIOD,GPIO_Pin_9);
			 Delay(1);	  
			 Output_Status[3] |= 0x20;	
			 break;
	   
		case 30://Y30-PG4 
			 GPIO_SetBits(GPIOG,GPIO_Pin_4); 
			 Delay(1);
			 Output_Status[3] |= 0x40;	  	
			 break;

		case 31://Y31-PG3 
			 GPIO_SetBits(GPIOG,GPIO_Pin_3); 
			 Delay(1);	  
			 Output_Status[3] |= 0x80;	
			 break;

    	case 32://Y32-PG2			3-手抓吹气
			 GPIO_SetBits(GPIOG,GPIO_Pin_2); 
			 Delay(1);		  
			 Output_Status[4] |= 0x01;
			 break;

		case 33://Y33-PD15			3-润滑使能
			 GPIO_SetBits(GPIOD,GPIO_Pin_15); 
			 Delay(1);			 
			 Output_Status[4] |= 0x02;	 
			 break;

		case 34://Y34-PD14 
			 GPIO_SetBits(GPIOD,GPIO_Pin_14);
			 Delay(1);	  
			 Output_Status[4] |= 0x04;	
			 break;
	
		case 35://Y35-PD13
			 GPIO_SetBits(GPIOD,GPIO_Pin_13);
			 Delay(1);	  
			 Output_Status[4] |= 0x08;	
			 break;

		case 36://Y36-PD12 		  3-报警灯
			 GPIO_SetBits(GPIOD,GPIO_Pin_12); 
			 Delay(1);	  
			 Output_Status[4] |= 0x10;	
			 break;

		case 37://Y37-PD11 
			 GPIO_SetBits(GPIOD,GPIO_Pin_11); 
			 Delay(1);	  
			 Output_Status[4] |= 0x20;	
			 break;
	   
		case 38://Y38-PD10 
			 GPIO_SetBits(GPIOD,GPIO_Pin_10); 
			 Delay(1);
			 Output_Status[4] |= 0x40;	  	
			 break;

		case 39://Y39-PD9 
			 GPIO_SetBits(GPIOD,GPIO_Pin_9); 
			 Delay(1);	  
			 Output_Status[4] |= 0x80;	
			 break;

	  /**------ 电机方向控制信号 ------**/
		case 40://PA7 - X_Axsis 
			 GPIO_SetBits(GPIOA,GPIO_Pin_7); 
			 Delay(1);	  	
			 break;

		case 41://PB11 - Z_Axsis 
			 GPIO_SetBits(GPIOB,GPIO_Pin_11); 
			 Delay(1);	  	
			 break;

		case 42://PE14 - L_Axsis 
			 GPIO_SetBits(GPIOE,GPIO_Pin_14); 
			 Delay(1);	  
			 break;

		case 43://PB9 - O_Axsis 
			 GPIO_SetBits(GPIOB,GPIO_Pin_9); 
			 Delay(1);	  
			 break;
		
		case 44://PC9 - A_Axsis 
			 GPIO_SetBits(GPIOC,GPIO_Pin_7); 
			 Delay(1);	  
			 break;
		
		case 45://PA9 - B_Axsis 
			 GPIO_SetBits(GPIOA,GPIO_Pin_2); 
			 Delay(1);	  
			 break;
		}
}


/**************************************************************************************************
**  函数名：  ResetOutput()
**	输入参数：u8 IO_Num:需要输出的端口  
**	输出参数：无
**	函数功能：复位端口电平信号
**	备注：	  
**  作者：    吴祥      
**  开发日期：2013/12/17 
***************************************************************************************************/
void ResetOutput(u8 IO_Num)
{
	switch (IO_Num-1)
	{
		case 0://Y0-PE1
			 GPIO_ResetBits(GPIOB,GPIO_Pin_5); 
			 Delay(1);		  
			 Output_Status[0] &= 0xfe;			
			 break;

		case 1://Y1-PE0
			 GPIO_ResetBits(GPIOG,GPIO_Pin_15); 
			 Delay(1);			 
			 Output_Status[0] &= 0xfd;
			 break;

		case 2://Y2-PB5 
			 GPIO_ResetBits(GPIOG,GPIO_Pin_14); 
		     Delay(1);	  
		     Output_Status[0] &= 0xfb;	
			 break;
	
		case 3://Y3-PG15
			 GPIO_ResetBits(GPIOG,GPIO_Pin_13); 
			 Delay(1);	  
			 Output_Status[0] &= 0xf7;	
			 break;

		case 4://Y4-PG14 
			 GPIO_ResetBits(GPIOG,GPIO_Pin_12); 
			 Delay(1);	  
		     Output_Status[0] &= 0xef;	
			 break;

		case 5://Y5-PG13 			
			 GPIO_ResetBits(GPIOG,GPIO_Pin_11); 
			 Delay(1);	  
			 Output_Status[0] &= 0xdf;
			 break;
		
		case 6://Y6-PG12 			
			 GPIO_ResetBits(GPIOG,GPIO_Pin_10); 
			 Delay(1);	  
			 Output_Status[0] &= 0xbf;		
			 break;

		case 7://Y7-PG11 
		     GPIO_ResetBits(GPIOG,GPIO_Pin_9); 
			 Delay(1);	  
			 Output_Status[0] &= 0x7f;	
			 break;

		case 8://Y8-PG10
			 GPIO_ResetBits(GPIOD,GPIO_Pin_7); 
			 Delay(1);		  
			 Output_Status[1] &= 0xfe;
			 break;

		case 9://Y9-PG9
			 GPIO_ResetBits(GPIOD,GPIO_Pin_6); 
			 Delay(1);			 
			 Output_Status[1] &= 0xfd;	 
			 break;

		case 10://Y10-PD7 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_5); 
			 Delay(1);	  
			 Output_Status[1] &= 0xfb;	
			 break;
	
		case 11://Y11-PD6 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_4); 
			 Delay(1);	  
			 Output_Status[1] &= 0xf7;
			 break;
		case 12://Y12-PD5 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_3); 
			 Delay(1);	  
			 Output_Status[1] &= 0xef;	
			 break;

		case 13://Y13-PD4 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_1); 
			 Delay(1);	  
			 Output_Status[1] &= 0xdf;	
			 break;
		
		case 14://Y14-PD3 
			 GPIO_ResetBits(GPIOD,GPIO_Pin_0); 
			 Delay(1);	  
			 Output_Status[1] &= 0xbf;	
			 break;

		case 15://Y15-PD2 
			 GPIO_ResetBits(GPIOA,GPIO_Pin_8); 
			 Delay(1);	  
			 Output_Status[1] &= 0x7f;	
			 break;

 		case 16://Y16-PD1
			 GPIO_ResetBits(GPIOG,GPIO_Pin_8); 
			 Delay(1);		  
			 Output_Status[2] &= 0xfe;
		 	 break;

		case 17://Y17-PD0
			 GPIO_ResetBits(GPIOG,GPIO_Pin_7); 
			 Delay(1);			 
			 Output_Status[2] &= 0xfd;	 
			 break;

		case 18://Y18-PC12 
			 GPIO_ResetBits(GPIOG,GPIO_Pin_6); 
			 Delay(1);	  
			 Output_Status[2] &= 0xfb;	
			 break;
	
		case 19://Y19-PC11 
			 GPIO_ResetBits(GPIOG,GPIO_Pin_5); 
			Delay(1);	  
			Output_Status[2] &= 0xf7;	
			break;

		case 20://Y20-PC10 
			 GPIO_ResetBits(GPIOG,GPIO_Pin_4); 
			Delay(1);	  
			Output_Status[2] &= 0xef;	
			break;

		case 21://Y21-PA8 
			 GPIO_ResetBits(GPIOG,GPIO_Pin_3); 
			Delay(1);	  
			Output_Status[2] &= 0xdf;	
			break;
		
		case 22://Y22-PC9 
			 GPIO_ResetBits(GPIOG,GPIO_Pin_2); 
			Delay(1);	  
			Output_Status[2] &= 0xbf;	
			break;

		case 23://Y23-PC8 
			GPIO_ResetBits(GPIOD,GPIO_Pin_15); 
			Delay(1);	  
			Output_Status[2] &= 0x7f;	
			break;

    	case 24://Y24-PC7
			GPIO_ResetBits(GPIOD,GPIO_Pin_14); 
			Delay(1);		  
			Output_Status[3] &= 0xfe;
			break;

		case 25://Y25-PC6
			GPIO_ResetBits(GPIOD,GPIO_Pin_13); 
			Delay(1);			 
			Output_Status[3] &= 0xfd;	 
			break;

		case 26://Y26-PG8 
			GPIO_ResetBits(GPIOD,GPIO_Pin_12); 
			Delay(1);	  
			Output_Status[3] &= 0xfb;	
			break;
	
		case 27://Y27-PG7 
			GPIO_ResetBits(GPIOD,GPIO_Pin_11); 
			Delay(1);	  
			Output_Status[3] &= 0xf7;	
			break;

		case 28://Y28-PG6
			GPIO_ResetBits(GPIOD,GPIO_Pin_10); 
			Delay(1);	  
			Output_Status[3] &= 0xef;	
			break;

		case 29://Y29-PG5 
			GPIO_ResetBits(GPIOD,GPIO_Pin_9); 
			Delay(1);	  
			Output_Status[3] &= 0xdf;	
			break;
		
		case 30://Y30-PG4
//			GPIO_ResetBits(GPIOD,GPIO_Pin_9); 
//			Delay(1);	  
//			Output_Status[3] &= 0xbf;	
			break;

		case 31://Y31-PG3
			GPIO_ResetBits(GPIOG,GPIO_Pin_3); 
			Delay(1);	  
			Output_Status[3] &= 0x7f;	
			break;

    	case 32://Y32-PG2
			GPIO_ResetBits(GPIOG,GPIO_Pin_2); 
			Delay(1);		  
			Output_Status[4] &= 0xfe;
			break;

		case 33://Y33-PD15
			GPIO_ResetBits(GPIOD,GPIO_Pin_15); 
			Delay(1);			 
			Output_Status[4] &= 0xfd;	 
			break;

		case 34://Y34-PD14 
			GPIO_ResetBits(GPIOD,GPIO_Pin_14); 
			Delay(1);	  
			Output_Status[4] &= 0xfb;	
			break;
	
		case 35://Y35-PD13
			GPIO_ResetBits(GPIOD,GPIO_Pin_13); 
			Delay(1);	  
			Output_Status[4] &= 0xf7;	
			break;

		case 36://Y36-PD12
			GPIO_ResetBits(GPIOD,GPIO_Pin_12); 
			Delay(1);	  
			Output_Status[4] &= 0xef;	
			break;

		case 37://Y37-PD11 
			GPIO_ResetBits(GPIOD,GPIO_Pin_11); 
			Delay(1);	  
			Output_Status[4] &= 0xdf;	
			break;
		
		case 38://Y38-PD10
			GPIO_ResetBits(GPIOD,GPIO_Pin_10); 
			Delay(1);	  
			Output_Status[4] &= 0xbf;	
			break;

		case 39://Y39-PD9
			GPIO_ResetBits(GPIOD,GPIO_Pin_9); 
			Delay(1);	  
			Output_Status[4] &= 0x7f;	
			break;
	  /**------ 电机方向控制信号 ------**/
		case 40://PA7 - X_Axsis 
			 GPIO_ResetBits(GPIOA,GPIO_Pin_7); 
			 Delay(1);	  	
			 break;

		case 41://PA1 - Z_Axsis 
			 GPIO_ResetBits(GPIOA,GPIO_Pin_1); 
			 Delay(1);	  	
			 break;

		case 42://PE14 - L_Axsis 
			 GPIO_ResetBits(GPIOE,GPIO_Pin_14); 
			 Delay(1);	  
			 break;

		case 43://PB9 - O_Axsis 
			 GPIO_ResetBits(GPIOB,GPIO_Pin_9); 
			 Delay(1);	  
			 break;

		}
}





/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/

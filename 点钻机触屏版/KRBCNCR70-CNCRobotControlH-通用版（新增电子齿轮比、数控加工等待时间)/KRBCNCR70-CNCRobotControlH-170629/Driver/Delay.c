/*************** (C) COPYRIGHT 2012 Kingrobot manipulator Team ****************
* File Name          : Delay.c
* Author             : Fenkella Zhou
* Version            : V1.0.0
* Date               : 20/06/2012
* Description        : This file delimits all the delay functions.
******************************************************************************/
//#include "stm32f10x_lib.h"
#include "stm32f10x.h"
#include "integer.h"
#include "Delay.h"

vs32 g_LCD_LightControl_Timer = 0;

void DelayNus(vu32 dly)			 //延时dly us：dly>=6，最小延时单位6us
{
	vu32 n;
	while(dly--)			   	//外部晶振：8M；PPL_9；Sysclk=8MHz*9=72MHz
	{
		n=8;					//微调参数，保证延时精度
		while(n--);
	}
}

void DelayNms(vu32 dly)		    //延时dly ms
{
	while(dly--)				//外部晶振：8M；PPL_9；Sysclk=8MHz*9=72MHz
		DelayNus(1000);			//1ms延时补偿
}

/******************* (C) COPYRIGHT 2012 Kingrobot manipulator Team *****END OF FILE****/

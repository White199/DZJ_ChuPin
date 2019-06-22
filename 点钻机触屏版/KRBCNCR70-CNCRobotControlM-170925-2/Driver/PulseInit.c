#include "stm32f10x_lib.h"
#include "PulseInit.h"

/**************************************************************************************************
**  函数名：  PulseInit
**	输入参数：无
**	输出参数：无
**	函数功能：初始化脉冲
**	备注：	  无
**  作者：         
**  开发日期：
***************************************************************************************************/
void PulseInit(void)
{
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;	
   	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_OCStructInit(&TIM_OCInitStructure); 

  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;                  //时钟分割
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	  //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = 9999;                      //写给TIMx_ARR的值，
	TIM_TimeBaseStructure.TIM_Prescaler = 1;                     //36M/1/10000=3600HZ

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;             //脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 5000;                          //待装入捕捉比较寄存器的脉冲值,占空比：5000/10000=50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //TIM输出比较极性高


	TIM_DeInit(TIM2);                                             //TIM2-CH1 ：PA0
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);	

	TIM_DeInit(TIM3);                                            //TIM3-CH1 ：PA6
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

	TIM_DeInit(TIM4);                                            //TIM4-CH3 ：PB8
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
 
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;                  //时钟分割
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	  //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = 9999;                      //写给TIMx_ARR的值，
	TIM_TimeBaseStructure.TIM_Prescaler = 1;                      //72M/2/10000=3600HZ
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;             //脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 5000;                          //待装入捕捉比较寄存器的脉冲值,占空比：5000/10000=50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //TIM输出比较极性高

	TIM_DeInit(TIM1);                                            //TIM1-CH3 ：PE13
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	TIM_Cmd(TIM2, DISABLE);      //TIM2-CH1 ：PA0
	TIM_Cmd(TIM3, DISABLE);      //TIM3-CH1 ：PA6
	TIM_Cmd(TIM1, DISABLE);	     //TIM1-CH3 ：PE13
	TIM_Cmd(TIM4, DISABLE);	     //TIM4-CH3 ：PB8



} 
/**************************************************************************************************
**  函数名：  PulseInit_Auto
**	输入参数：无
**	输出参数：无
**	函数功能：初始化脉冲
**	备注：	  无
**  作者：         
**  开发日期：
***************************************************************************************************/
void PulseInit_Auto(void)
{
  TIM_OCInitTypeDef TIM_OCInitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure ;	
   	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_OCStructInit(&TIM_OCInitStructure); 
	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;                  //时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	  //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = 9999;                      //写给TIMx_ARR的值，
	TIM_TimeBaseStructure.TIM_Prescaler = 1;                      //72M/2/10000=3600HZ
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;             //脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 5000;                         //待装入捕捉比较寄存器的脉冲值,占空比：5000/10000=50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //TIM输出比较极性高
	
	
	TIM_DeInit(TIM3);                                             //TIM3-CH1 ：PA6  O
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);

	TIM_DeInit(TIM4);                                             //TIM4-CH3 ：PB8  X
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	TIM_ITConfig(TIM4, TIM_IT_Update,ENABLE);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
	
	TIM_DeInit(TIM2);                                             //TIM2-CH3 ：PB10 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM2, ENABLE);	
	
	TIM_DeInit(TIM5);                                             //TIM5-CH4 ：PA2 
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
	TIM_ITConfig(TIM5, TIM_IT_Update,ENABLE);
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM5, ENABLE);

  TIM_TimeBaseStructure.TIM_ClockDivision = 0;                  //时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	  //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = 9999;                      //写给TIMx_ARR的值，
	TIM_TimeBaseStructure.TIM_Prescaler = 1;                      //72M/2/10000=3600HZ
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;								//高级定时器重复计数次数
	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;             //脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 5000;                         //待装入捕捉比较寄存器的脉冲值,占空比：5000/10000=50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //TIM输出比较极性高

	TIM_DeInit(TIM1);                                             //TIM1-CH3 ：PE13
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	TIM_DeInit(TIM8);                                             //TIM8-CH1 ：PC6
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	TIM_ClearITPendingBit(TIM8,TIM_IT_Update);
	TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	
	TIM_Cmd(TIM3, DISABLE);      //TIM3-CH1 ：PA6  O
	TIM_Cmd(TIM1, DISABLE);	     //TIM1-CH3 ：PE13 L
	TIM_Cmd(TIM4, DISABLE);	     //TIM4-CH3 ：PB8  X
	TIM_Cmd(TIM2, DISABLE); 	 //TIM2-CH3 ：PB10 Z
	TIM_Cmd(TIM5, DISABLE);	     //TIM5-CH4 ：PA2  B
	TIM_Cmd(TIM8, DISABLE);	     //TIM8-CH1 ：PC6  A
} 
/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team ******** END OF FILE ************************/

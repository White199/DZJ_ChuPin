/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 12/17/2013
* Description        : This is the ....
***************************************************************************************/

#ifndef __platform_init_h_
#define __platform_init_h_

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void 	Key_EXTI_Init(void);
void TimeInit(void);
void USART1_DMA_Configuration(void);
void USART1_Robot_Configuration(void);
void USART2_485_Configuration(void);
void USART3_LCD_Configuration(void);
void RTC_Configuration(void);
void RTC_Init(void);

void Rod_Adjust(void);
void Rod_Axis_Contrl(u8 Rod_Speed_Rank,u8 Axis_DIR,u8 Axis);
void Rob_Move(void);
void Rob_Link_Move(void);
void Rob_Step_Move(void);
void Rod_Select_Order(u8 Page_Type);
void MYDMA_Config(void);
void Adc_Init(void);
#endif

/******************* (C) COPYRIGHT 2013 Kingrobot manipulator Team *****END OF FILE****/


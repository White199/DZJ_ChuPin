/**
  ******************************************************************************
  * @file    usbh_usr.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11/29/2010
  * @brief   Header file for usbh_usr.c
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USH_USR_H__
#define __USH_USR_H__

/* Includes ------------------------------------------------------------------*/
#include "ff.h"
#include "usbh_core.h"
#include <stdio.h>
#include "usbh_msc_core.h"



/** @addtogroup USBH_USER
  * @{
  */

/** @addtogroup USBH_MSC_DEMO_USER_CALLBACKS
  * @{
  */

/** @defgroup USBH_USR
  * @brief This file is the Header file for usbh_usr.c
  * @{
  */ 


/** @defgroup USBH_USR_Exported_Types
  * @{
  */ 


extern  USBH_Usr_cb_TypeDef USR_Callbacks;



/**
  * @}
  */ 

//#define HOST_OVRCURR_PORT                  GPIOE
//#define HOST_OVRCURR_LINE                  GPIO_Pin_1
//#define HOST_OVRCURR_PORT_SOURCE           GPIO_PortSourceGPIOE
//#define HOST_OVRCURR_PIN_SOURCE            GPIO_PinSource1
//#define HOST_OVRCURR_PORT_RCC              RCC_APB2Periph_GPIOE
//#define HOST_OVRCURR_EXTI_LINE             EXTI_Line1
//#define HOST_OVRCURR_IRQn                  EXTI1_IRQn 

#define HOST_POWERSW_PORT_RCC              RCC_APB2Periph_GPIOC
#define HOST_POWERSW_PORT                  GPIOC
#define HOST_POWERSW_VBUS                  GPIO_Pin_4

#define HOST_SOF_OUTPUT_RCC                RCC_APB2Periph_GPIOA
#define HOST_SOF_PORT                      GPIOA
#define HOST_SOF_SIGNAL                    GPIO_Pin_8

#define KEYA_PORT_RCC					   RCC_APB2Periph_GPIOC
#define KEYA_PORT						   GPIOC
#define KEYA_PORT_PIN					   GPIO_Pin_6

#define KEYB_PORT_RCC					   RCC_APB2Periph_GPIOC
#define KEYB_PORT						   GPIOC
#define KEYB_PORT_PIN					   GPIO_Pin_7

#define KEYC_PORT_RCC					   RCC_APB2Periph_GPIOC
#define KEYC_PORT						   GPIOC
#define KEYC_PORT_PIN					   GPIO_Pin_8

/** @defgroup USBH_USR_Exported_Defines
  * @{
  */ 
/* State Machine for the USBH_USR_ApplicationState */
#define USH_USR_FS_INIT       0
#define USH_USR_FS_READLIST   1
#define USH_USR_FS_WRITEFILE  2
#define USH_USR_FS_READFILE   3
#define USB_USR_FS_READLIST   4
#define USH_USR_FS_NULL       5
/**
  * @}
  */ 

/** @defgroup USBH_USR_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_USR_Exported_Variables
  * @{
  */ 
extern  uint8_t USBH_USR_ApplicationState ;
extern  u8  USB_Button_Ok;		   //确定按钮
extern  u8  USB_Button_Cancle;	   //取消按钮
/**
  * @}
  */ 

/** @defgroup USBH_USR_Exported_FunctionsPrototype
  * @{
  */ 
void USBH_USR_ApplicationSelected(void);
void USBH_USR_Init(void);
void USBH_USR_DeviceAttached(void);
void USBH_USR_ResetDevice(void);
void USBH_USR_DeviceDisconnected (void);
void USBH_USR_OverCurrentDetected (void);
void USBH_USR_DeviceSpeedDetected(uint8_t DeviceSpeed); 
void USBH_USR_Device_DescAvailable(void *);
void USBH_USR_DeviceAddressAssigned(void);
void USBH_USR_Configuration_DescAvailable(USBH_CfgDesc_TypeDef * cfgDesc,
                                          USBH_InterfaceDesc_TypeDef *itfDesc,
                                          USBH_EpDesc_TypeDef *epDesc);
void USBH_USR_Manufacturer_String(void *);
void USBH_USR_Product_String(void *);
void USBH_USR_SerialNum_String(void *);
void USBH_USR_EnumerationDone(void);
USBH_USR_Status USBH_USR_UserInput(void);
void USBH_USR_HexToASCII(uint32_t);
int USBH_USR_MSC_Application(void);
void DisplayBMPImage(void);
void USBH_USR_HexToASCII(uint32_t);
void USBH_USR_DeInit(void);
void USBH_USR_DeviceNotSupported(void);
void USBH_USR_UnrecoveredError(void);
//void ReadFileContent(unsigned char);//读取指定文件名中的文件内容，并保存到FLASH中去
/**
  * @}
  */ 

#endif /*__USH_USR_H__*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/





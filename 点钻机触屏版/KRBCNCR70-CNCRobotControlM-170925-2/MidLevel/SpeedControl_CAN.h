/*************** (C) COPYRIGHT 2013 Kingrobot manipulator Team ************************
* File Name          : ***.c
* Author             : Wu Xiang
* Version            : V1.0.0
* Date               : 02/10/2017
* Description        : This is the ....
***************************************************************************************/
#ifndef __speedcontrol_can_h_
#define __speedcontrol_can_h_

#define MAX_AXIS_NUM    6      //伺服器CANopen模式控制轴数	编号为0,1,2,...,MAX_AXIS_NUM-1

extern void MoveToTarget_CAN(u8,u32,u32);
void ServoStop_CAN(u8);
void ServoEmergencyStop_CAN(u8);

extern u8 CAN_Servo_Stop_Action[MAX_AXIS_NUM+1];

#endif


/******************* (C) COPYRIGHT 2017 Kingrobot manipulator Team *****END OF FILE****/



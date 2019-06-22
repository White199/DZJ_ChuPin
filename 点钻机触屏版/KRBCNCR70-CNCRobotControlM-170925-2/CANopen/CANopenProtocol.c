/******************** COPYRIGHT 2013 KingRobot ********************
* File Name          : CANopenProtocol.c
* Author             : WuXiang
* Version            : V1.0.0
* Date               : 07/11/2013
* Description        : CANopen protocol
*                      This file implement CANopen protocol
********************************************************************************/

#include "CANopen.h"
//
//
//
//u8 Recieve_Data[10] = { 0 } ;
//u8 C_j = 0;
//u8 j=0;
//
//
//void CANopenProtocol()
//{
//   switch(State_Machine)
//   {
//        
//        case Initialisation: Boot_Up(ServoNodeID_0);
//                             Enter_Pre_Operational_State(ServoNodeID_0);
//                             Delay(10);
//                         //    break;                             
//       // case Pre_Operate   : 
//                             SDO_Process();                       //初始化伺服器节点以及设置相关参数                                                                               
//                             Start_Remote_Node(ServoNodeID_0);    //启动远程节点，进入operate模式
//                             Delay(10);
//                             break;
//        case Operate       : PDO_Process();                                                      
//                             if(C_j==0)
//                             {
//                                   C_j=1;
//                                   GPIO_ResetBits(GPIOF, GPIO_Pin_8);
//                             }
//                             //Delay(50);
//                             else
//                             {
//                                   C_j=0;
//                                   GPIO_SetBits(GPIOF, GPIO_Pin_8);
//                             }     
//                             break;
//        case Stop          : 
//                             break;           
//    }      
//      
//}




















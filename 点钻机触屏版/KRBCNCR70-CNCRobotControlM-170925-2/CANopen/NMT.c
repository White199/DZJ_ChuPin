/******************** COPYRIGHT 2013 KingRobot *********************************
* File Name          : NMT.c
* Author             : WuXiang
* Version            : V1.0.0
* Date               : 11/14/2013
* Description        : This file complete NMT protocol
********************************************************************************/
#include "CANopen.h"

CanTxMsg NMT_TxMessage;        //NMT发送的CAN数据

u8  NMT_TransmitMailbox;       //返回的发送邮箱状态
vu8 Flag_Get_Node_State=0;     //得到节点通信状态标识位

/***********************************************
 *              Send_NMT_Protocol              *
 ***********************************************/
/***************************************************************************
**  函数名：  Send_NMT_Protocol()
**	输入参数：无
**	输出参数：无
**	函数功能：发送NMT协议消息
**	备注：	  无
**  作者：    吴祥    
**  开发日期：2013/11/14
***************************************************************************/
/***********state tansition*********/
void Send_NMT_Protocol(u8 NMT_Node_ID , u8 NMT_cs)
{
  /* transmit */
  NMT_TxMessage.StdId   = 0x00;                  // COB_ID=0
  NMT_TxMessage.RTR     = CAN_RTR_DATA;          // 设定待传输消息帧类型：_DATA-数据帧；_REMOTE-远程帧
  NMT_TxMessage.IDE     = CAN_ID_STD;            // 设定消息标识符类型：_STD-标准标识符；_EXT-标准+扩展标识符
  NMT_TxMessage.DLC     = 2;                     // 确定传输消息的帧长度：0~0x8 
  NMT_TxMessage.Data[0] = NMT_cs;                // CMD specifier：命令字
  NMT_TxMessage.Data[1] = NMT_Node_ID;           // 伺服器ID
 
  NMT_TransmitMailbox=CAN_Transmit(&NMT_TxMessage);
  CANopen_k= 0;
  while((CAN_TransmitStatus(NMT_TransmitMailbox) != CANTXOK) && (CANopen_k!= 0xFF)) 
  {                                               //通过检查CANTXOK位来确认发送是否成功
      CANopen_k++;
  }
}

/***********get node state*********/
void Get_Node_State()
{
     u8 state=0;
   //  state = CAN_Recieve_Data[0]&0x7f ;         //得到状态值[6:0]   
     switch(state)
     {
        case 0  :  State_Machine = Initialisation; break;
        case 4  :  State_Machine = Stop;           break;
        case 5  :  State_Machine = Operate;        break;
        case 127:  State_Machine = Pre_Operate;    break;
        default :  break;       
     }
}

/*******Error_Control protocol*********/
/*    Boot-up    and   Node Gurding   */
void Send_Node_Guard( u8 Node_ID )
{
  NMT_TxMessage.StdId   = 0x700 + Node_ID ;      // COB_ID=0x700 + Node_ID
  NMT_TxMessage.RTR     = CAN_RTR_REMOTE;          // 设定待传输消息帧类型：_DATA-数据帧；_REMOTE-远程帧
  NMT_TxMessage.IDE     = CAN_ID_STD;            // 设定消息标识符类型：_STD-标准标识符；_EXT-标准+扩展标识符
 
  NMT_TransmitMailbox=CAN_Transmit(&NMT_TxMessage);
  CANopen_k = 0;
  while((CAN_TransmitStatus(NMT_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
  {                                              //通过检查CANTXOK位来确认发送是否成功
      CANopen_k++;
  }
} 

void Boot_Up( u8 Node_ID )
{
  NMT_TxMessage.StdId   = 0x700 + Node_ID ;      // COB_ID=0x700 + Node_ID
  NMT_TxMessage.RTR     = CAN_RTR_DATA;          // 设定待传输消息帧类型：_DATA-数据帧；_REMOTE-远程帧
  NMT_TxMessage.IDE     = CAN_ID_STD;            // 设定消息标识符类型：_STD-标准标识符；_EXT-标准+扩展标识符
  NMT_TxMessage.DLC     = 1;                     // 确定传输消息的帧长度：0~0x8 
  NMT_TxMessage.Data[0] = 0x00;                  // CMD specifier：命令字
 
  NMT_TransmitMailbox=CAN_Transmit(&NMT_TxMessage);
  CANopen_k = 0;
  while((CAN_TransmitStatus(NMT_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
  {                                              //通过检查CANTXOK位来确认发送是否成功
      CANopen_k++;
  }
}

/*******启动远程节点*********/
void Start_Remote_Node(u8 Node_ID)             //0x01
{
  Send_NMT_Protocol(Node_ID , START_REMOTE_NODE) ;
  State_Machine = Operate;
}

/*******停止远程节点*********/
void Stop_Remote_Node(u8 Node_ID)              //0x02
{
  Send_NMT_Protocol(Node_ID , STOP_REMOTE_NODE);
  State_Machine = Stop;
}

/*******进入预操作状态*********/
void Enter_Pre_Operational_State(u8 Node_ID)   //0x80 
{
   Send_NMT_Protocol(Node_ID , ENTER_PRE_OPERATIONAL_STATE);
   State_Machine = Pre_Operate;
}

/*******复位节点*********/
void Reset_Node(u8 Node_ID)                    //0x81
{
   Send_NMT_Protocol(Node_ID , RESET_NODE);
   State_Machine = Initialisation;  
}

/*******复位通信*********/
void Reset_Communication(u8 Node_ID)           //0x82
{                                                         
   Send_NMT_Protocol(Node_ID , RESET_COMMUNICATION);
   State_Machine = Initialisation;
}

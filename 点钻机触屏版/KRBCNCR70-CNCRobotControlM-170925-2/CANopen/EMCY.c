#include "CANopen.h"

CanTxMsg EMCY_TxMessage;
u8 EMCY_TransmitMailbox;
    
/***********************************************
 *              Send_NMT_Protocol              *
 ***********************************************/
void Process_EMCY()
{
  EMCY_TxMessage.StdId   = 0x80+0x01;             // COB_ID=0x80
  EMCY_TxMessage.RTR     = CAN_RTR_DATA;          // 设定待传输消息帧类型：_DATA-数据帧；_REMOTE-远程帧
  EMCY_TxMessage.IDE     = CAN_ID_STD;            // 设定消息标识符类型：_STD-标准标识符；_EXT-标准+扩展标识符
  EMCY_TxMessage.DLC     = 8;                     // 确定传输消息的帧长度：0~0x8 
  EMCY_TxMessage.Data[0] = 0x00;               
  EMCY_TxMessage.Data[1] = 0x00;               
  EMCY_TxMessage.Data[2] = 0x00;
  EMCY_TxMessage.Data[3] = 0x00;
  EMCY_TxMessage.Data[4] = 0x00;
  EMCY_TxMessage.Data[5] = 0x00;
  EMCY_TxMessage.Data[6] = 0x00;
  EMCY_TxMessage.Data[7] = 0x00;
  EMCY_TransmitMailbox=CAN_Transmit(&EMCY_TxMessage);
  CANopen_k = 0;
  while((CAN_TransmitStatus(EMCY_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
  {                                            
     CANopen_k++;
  }
}

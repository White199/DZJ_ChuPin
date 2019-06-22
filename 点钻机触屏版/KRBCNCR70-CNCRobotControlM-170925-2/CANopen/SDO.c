/******************** COPYRIGHT 2013 KingRobot *********************************
* File Name          : SDO.c
* Author             : WuXiang
* Version            : V1.0.0
* Date               : 11/14/2013
* Description        : This file complete SDO protocol 
********************************************************************************/
#include "CANopen.h"
#include "StatusControl.h"
#include "Parameter.h"

u8 SDO_TransmitMailbox;
 
CanTxMsg   SDO_TxMessage;                
SDO_Struct SDO_Message;                //SDO写信息



/***************************************************************************
**  函数名：  SDO_Process()
**	输入参数：无
**	输出参数：无
**	函数功能：发送SDO消息，设置伺服器参数，对PDO进行映射，设置伺服器位置模式
**	备注：	  无
**  作者：    吴祥    
**  开发日期：2013/11/14
***************************************************************************/
/***********************************************
 * SDO_Process: 在预操作状态下对伺服器进行初   *
 *              始化和相关参数设置             *
 ***********************************************/
void SDO_Process(u8 Servo_Node_ID)
{
    Change_Set(CHANGE_SET_IMMEDIATELY);                  //设置控制字6040h的第5位，CHANGE_SET_IMMEDIATELY
    
   /*******配置伺服器节点*****/  
    //Set_Guard_Time    (Servo_Node_ID);     //set guard time:2s 
	//--------------------------White_L  将电子齿轮比改为80------------------------------------------//
     Set_PUU           (Servo_Node_ID,0x0500,0x0001);       //设置电子齿轮比:1280
     RPDO_Mapping      (Servo_Node_ID);       //RPDO映射
     TPDO_Mapping      (Servo_Node_ID);       //TPDO映射
     //Set_Servo_PP_Mode (Servo_Node_ID);     //设置位置模式  
	 Set_Servo_Homing_Mode(Servo_Node_ID);    //设置回原点模式--引入变量回原点方式
}

/************************************************************
 * SDO_Process_PositionMode: 在预操作状态下对伺服器进行初   *
 * 2017.8.15                 始化和相关参数设置             *
 ************************************************************/
void SDO_Process_PositionMode(u8 Servo_Node_ID)
{
     Change_Set(CHANGE_SET_IMMEDIATELY);                    //设置控制字6040h的第5位，CHANGE_SET_IMMEDIATELY
	
     Set_PUU           (Servo_Node_ID,0x0500,0x0001); //设置电子齿轮比:1000
     RPDO_Mapping      (Servo_Node_ID);               //RPDO映射
     TPDO_Mapping      (Servo_Node_ID);               //TPDO映射  
	
     Set_Servo_PP_Mode (Servo_Node_ID);               //设置位置模式 
}

/**********************************************************
 * SDO_Process_HomingMode: 在预操作状态下对伺服器进行初   *
 * 2017.8.16               始化和相关参数设置             *
 **********************************************************/
void SDO_Process_HomingMode(u8 Servo_Node_ID)
{
	 Change_Set(CHANGE_SET_IMMEDIATELY);                    //设置控制字6040h的第5位，CHANGE_SET_IMMEDIATELY

     Set_PUU           (Servo_Node_ID,0x0500,0x0001); //设置电子齿轮比:1000
     RPDO_Mapping      (Servo_Node_ID);               //RPDO映射
     TPDO_Mapping      (Servo_Node_ID);               //TPDO映射  
	
     Set_Servo_Homing_Mode(Servo_Node_ID);            //设置回原点模式--引入变量回原点方式 
}

/***********************************************
 *      Pulse of user unit :设置电子齿轮比     *
 *      1280000*6093-02h/6093-01h              *
 *      6093-01h:1  6093-02h:1280              *
 *      PUU:1000                               *
 ***********************************************/
void Set_PUU(u8 Node_ID,u32 Numerator,u32 Feed_Constant)
{
   //Servo off
   SDO_Message.Node_ID   = Node_ID;
   SDO_Message.index     = 0x6040;
   SDO_Message.sub_index = 0x00;
   SDO_Message.len       = 2;
   SDO_Message.Data[0]   = Controlword;   //0x0000
   SDO_Message.Data[1]   = Controlword>>8;     
   SDO_Protocol(INITIATE_SDO_DOWNLOAD);
 //-- 
	Delay(10);
   //Set PUU
   SDO_Message.Node_ID   = Node_ID;
   SDO_Message.index     = 0x6093;
   SDO_Message.sub_index = 0x01;
   SDO_Message.len       = 4;
   SDO_Message.Data[0]   = Numerator;   //0x000500h:1280     
   SDO_Message.Data[1]   = Numerator>>8;
   SDO_Message.Data[2]   = Numerator>>16;
   SDO_Message.Data[3]   = Numerator>>24;
   SDO_Protocol(INITIATE_SDO_DOWNLOAD);
 //-- 
	Delay(10);

   SDO_Message.Node_ID   = Node_ID;
   SDO_Message.index     = 0x6093;
   SDO_Message.sub_index = 0x02;
   SDO_Message.len       = 4;
   SDO_Message.Data[0]   = Feed_Constant;        
   SDO_Message.Data[1]   = Feed_Constant>>8;
   SDO_Message.Data[2]   = Feed_Constant>>16;
   SDO_Message.Data[3]   = Feed_Constant>>24;
   SDO_Protocol(INITIATE_SDO_DOWNLOAD);
 //-- 
	Delay(10);
}

/***************************************************************************
**  函数名：  SDO_Protocol()
**	输入参数：无
**	输出参数：无
**	函数功能：SDO协议通信，完成伺服器的SDO上传和SDO下载
**	备注：	  无
**  作者：    吴祥    
**  开发日期：2013/11/14
***************************************************************************/
void SDO_Protocol(u8 Means)
{
    switch(Means){
          case INITIATE_SDO_DOWNLOAD: Initiate_SDO_Download(); break; //写入伺服器数据
          case INITIATE_SDO_UPLOAD  : Initiate_SDO_Upload();   break; //读取伺服器数据
          case ABORT_SDO_TRANSFER   : Abort_SDO_Transfer();    break;
          default : break;
    }
}

/***********************************************
 *      启动上传：Initiate_SDO_Upload          *
 *      读取伺服器数据                         *
 ***********************************************/
void Initiate_SDO_Upload()                        
{  
  SDO_TxMessage.StdId=SDO_Message.Node_ID+0x600;       // 伺服器ID号为0x01
  SDO_TxMessage.RTR=CAN_RTR_DATA;                      // 设定待传输消息帧类型：_DATA-数据帧；_REMOTE-远程帧
  SDO_TxMessage.IDE=CAN_ID_STD;                        // 设定消息标识符类型：_STD-标准标识符；_EXT-标准+扩展标识符
  SDO_TxMessage.DLC=SDO_Message.len+4;                 // 确定传输消息的帧长度：0~0x8 
  SDO_TxMessage.Data[0]=0x40;                          // CMD specifier：命令字
  SDO_TxMessage.Data[1]=SDO_Message.index&0x00ff;      // 对象索引低字节
  SDO_TxMessage.Data[2]=SDO_Message.index>>8;          // 对象索引高字节
  SDO_TxMessage.Data[3]=SDO_Message.sub_index;         // 对象子索引
  SDO_TxMessage.Data[4]=0x00;
  SDO_TxMessage.Data[5]=0x00;
  SDO_TxMessage.Data[6]=0x00;
  SDO_TxMessage.Data[7]=0x00;

  SDO_TransmitMailbox=CAN_Transmit(&SDO_TxMessage);
  CANopen_k = 0;
  while((CAN_TransmitStatus(SDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF))
  {
      CANopen_k++;
  }
}

/***********************************************
 *       启动下载：Initiate_SDO_Download       *
 *       写入伺服器数据                        *
 ***********************************************/
void Initiate_SDO_Download()                      
{
  SDO_TxMessage.StdId=SDO_Message.Node_ID+0x600;       // 伺服器ID号为0x01
  SDO_TxMessage.RTR=CAN_RTR_DATA;                      // 设定待传输消息帧类型：_DATA-数据帧；_REMOTE-远程帧
  SDO_TxMessage.IDE=CAN_ID_STD;                        // 设定消息标识符类型：_STD-标准标识符；_EXT-标准+扩展标识符
  SDO_TxMessage.DLC=SDO_Message.len+4;                 // 确定传输消息的帧长度：0~0x8 
  switch(SDO_Message.len)                              // CMD specifier：命令字
  {
     case 1: SDO_TxMessage.Data[0]=0x2F;break;         // 1字节：0X2F
     case 2: SDO_TxMessage.Data[0]=0x2B;break;         // 2字节：0X2B
     case 4: SDO_TxMessage.Data[0]=0x23;break;         // 4字节：0X23
     default:SDO_TxMessage.Data[0]=0x23;break; 
  }                         
  SDO_TxMessage.Data[1]=SDO_Message.index;             // 对象索引低字节
  SDO_TxMessage.Data[2]=SDO_Message.index>>8;          // 对象索引高字节
  SDO_TxMessage.Data[3]=SDO_Message.sub_index;         // 对象子索引
  SDO_TxMessage.Data[4]=SDO_Message.Data[0];
  SDO_TxMessage.Data[5]=SDO_Message.Data[1];
  SDO_TxMessage.Data[6]=SDO_Message.Data[2];
  SDO_TxMessage.Data[7]=SDO_Message.Data[3];

  SDO_TransmitMailbox=CAN_Transmit(&SDO_TxMessage);
  CANopen_k = 0;
  while((CAN_TransmitStatus(SDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF))
  {
      CANopen_k++;
  }
}

/***********************************************
 *       终止SDO通信：Abort_SDO_Transfer       *
 ***********************************************/
void Abort_SDO_Transfer()
{
  SDO_TxMessage.StdId=SDO_Message.Node_ID+0x600;       // 伺服器ID号为0x01
  SDO_TxMessage.RTR=CAN_RTR_DATA;                      // 设定待传输消息帧类型：_DATA-数据帧；_REMOTE-远程帧
  SDO_TxMessage.IDE=CAN_ID_STD;                        // 设定消息标识符类型：_STD-标准标识符；_EXT-标准+扩展标识符
  SDO_TxMessage.DLC=SDO_Message.len+4;                 // 确定传输消息的帧长度：0~0x8
  SDO_TxMessage.Data[0]=0x80;                          
  SDO_TxMessage.Data[1]=SDO_Message.index;             // 对象索引低字节
  SDO_TxMessage.Data[2]=SDO_Message.index>>8;          // 对象索引高字节
  SDO_TxMessage.Data[3]=SDO_Message.sub_index;         // 对象子索引
  SDO_TxMessage.Data[4]=SDO_Message.Data[0];           // 数据由abort code得到
  SDO_TxMessage.Data[5]=SDO_Message.Data[1];
  SDO_TxMessage.Data[6]=SDO_Message.Data[2];
  SDO_TxMessage.Data[7]=SDO_Message.Data[3];

  SDO_TransmitMailbox=CAN_Transmit(&SDO_TxMessage);
  CANopen_k = 0;
  while((CAN_TransmitStatus(SDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF))
  {
       CANopen_k++;
  }
}


/***************************************************************************
**  函数名：  Query_Servo_parameter()
**	输入参数：伺服器节点ID号、对象字典索引、子索引：Node_ID、index、sunindex
**	输出参数：无
**	函数功能：查询伺服器参数
**	备注：	  无
**  作者：    吴祥    
**  开发日期：2013/11/14
***************************************************************************/
void Query_Servo_parameter(u8 Node_ID,u16 index,u8 sub_index)
{
     SDO_Message.Node_ID = Node_ID;
     SDO_Message.index = index;
     SDO_Message.sub_index = sub_index;
     SDO_Message.len = 0;
	   Initiate_SDO_Upload();
}

/*************设置保护节点时间******************/
void Set_Guard_Time(u8 Node_ID)
{
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x100C;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 2;
    SDO_Message.Data[0]   = 0xe8;   //0x07d0:2000ms
    SDO_Message.Data[1]   = 0x03;       
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(10);
}

void Fault_Reset(u8 Node_ID)
{
    Controlword = Controlword | 0x0080;
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6040;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 2;
    SDO_Message.Data[0]   = Controlword; 
    SDO_Message.Data[1]   = Controlword>>8;      
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(10);
}

/***************************************************************************
**  函数名：  Set_Servo_PP_Mode()
**	输入参数：无
**	输出参数：无
**	函数功能：配置伺服器位置模式，并设置位置，速度，加减速初始值
**	备注：	  无
**  作者：    吴祥    
**  开发日期：2013/11/14
***************************************************************************/
/******************配置位置模式*************************              
 *  6060h: set to profile position mode                *
 *  607Ah: set to target position                      *
 *  6081h: set to profile velocity                     *
 *  6083h: set to plan acceleration slope              *
 *  6084h: set to plan deceleration slope              *
 *  6040h: set to servo on drive and make motor work   *
 *******************************************************/
void Set_Servo_PP_Mode(u8 Node_ID)
{
   /* 1. set to profile position mode  int8 */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6060;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x01;       
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	 //--	
   Delay(10);
   
   /* 2. set to target position  int32  default:0   */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x607A;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;
    SDO_Message.Data[1]   = 0x00;   //0xc350:50000
    SDO_Message.Data[2]   = 0x00;   //0x02faf080:50000000
    SDO_Message.Data[3]   = 0x00;       
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	 //--	
   Delay(10);

   /* 3. set to profile velocity  int32  default:10000  */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6081;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;   //0x03e8:1000
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x00;       
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	 //--	
   Delay(10);

   /************设置加减速大小*******************
    *     加减速单位为：ms                      *
    *     值为电机转速从0增加到3000rpm的时间    *
    *     故，当值越大，也就是时间越长          *
    *     则实际电机加速度就越小！              *
    *********************************************/
   /* 4. set to plan acceleration slope  u32  default:200  */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6083;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0xA0;   //0x00c8:200 0x0fa0:4000ms
    SDO_Message.Data[1]   = 0x86;
    SDO_Message.Data[2]   = 0x01;
    SDO_Message.Data[3]   = 0x00;      
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	 //--	
   Delay(10);

   /* 5. set to plan deceleration slope  u32  default:200     */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6084;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0xA0;   //0x00c8:200 0x0fa0:4000ms
    SDO_Message.Data[1]   = 0x86;
    SDO_Message.Data[2]   = 0x01;
    SDO_Message.Data[3]   = 0x00;      
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
   //--	
   Delay(10);
	
   /* 6. set to servo on drive and make motor work  u16  Default:0x0000  */		
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6040;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 2;
    SDO_Message.Data[0]   = Controlword;   //0x000F
    SDO_Message.Data[1]   = Controlword>>8;;      
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	  Delay(10);
	 
//    SDO_Message.Node_ID   = Node_ID;
//    SDO_Message.index     = 0x6040;
//    SDO_Message.sub_index = 0x00;
//    SDO_Message.len       = 2;
//    SDO_Message.Data[0]   = 0x1f;   //0x001F
//    SDO_Message.Data[1]   = 0x00;;      
//    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
//	 Delay(20);
	
   /* 7. followeing error window  u32  default:3840000   */
  /*  SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6065;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0xc8;   //0x3a9800:3840000
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x00;      
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
   */
   /* 8. position window    u32  default:100   */
   /* SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6067;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x64;   //0x0064:100
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x00;      
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    */

   /* 9. position window time   u16  default:0   */
  /*  SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6068;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 2;
    SDO_Message.Data[0]   = 0x50;   //0xc350:50000ms
    SDO_Message.Data[1]   = 0xc3;      
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    */
}

/***************************************************************************
**  函数名：  Set_Servo_Homing_Mode()
**	输入参数：无
**	输出参数：无
**	函数功能：配置伺服器回原点模式
**	备注：	  无
**  作者：    吴祥    
**  开发日期：2013/11/22
***************************************************************************/
void Set_Servo_Homing_Mode(u8 Node_ID)
{
	u32 Orign_Speed=0;
   /* 1. set to homing mode  int8 : 6     */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6060;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x06;       
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
    Delay(10);
   
   /* 2. set to homing offset  单位为脉冲数目int32  default:0   */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x607C;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x00;
    SDO_Message.Data[1]   = 0x00;   
    SDO_Message.Data[2]   = 0x00;   
    SDO_Message.Data[3]   = 0x00;       
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //---
	 Delay(10);
   /* 3. homing method (rang is 0-35)  int8  default:0  */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6098;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 1;
    SDO_Message.Data[0]   = 0x15;        //0x20:method 20 负方向回零
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //---
   Delay(10);
   /* 4. sub-1 speed during search for switch(1-2000)  u32  default:100  unit:rpm */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6099;
    SDO_Message.sub_index = 0x01;
    SDO_Message.len       = 4;
	switch(Node_ID-1)
	{
		case X_Axsis:
			Orign_Speed = JXS_Parameter.OriginSpeed[X_Axsis]*50;
			SDO_Message.Data[0]   = Orign_Speed;   //0x0064:500
			SDO_Message.Data[1]   = Orign_Speed>>8;
			SDO_Message.Data[2]   = 0x00;
			SDO_Message.Data[3]   = 0x00; 
		break;
		
		case Z_Axsis:
			Orign_Speed = JXS_Parameter.OriginSpeed[Z_Axsis]*50;
			SDO_Message.Data[0]   = Orign_Speed;   //0x0064:500
			SDO_Message.Data[1]   = Orign_Speed>>8;
			SDO_Message.Data[2]   = 0x00;
			SDO_Message.Data[3]   = 0x00; 
		break;
		
		case L_Axsis:
			Orign_Speed = JXS_Parameter.OriginSpeed[L_Axsis]*50;
			SDO_Message.Data[0]   = Orign_Speed;   //0x0064:500
			SDO_Message.Data[1]   = Orign_Speed>>8;
			SDO_Message.Data[2]   = 0x00;
			SDO_Message.Data[3]   = 0x00;
		break;

		case O_Axsis:
			Orign_Speed = JXS_Parameter.OriginSpeed[O_Axsis]*50;
			SDO_Message.Data[0]   = Orign_Speed;   //0x0064:500
			SDO_Message.Data[1]   = Orign_Speed>>8;
			SDO_Message.Data[2]   = 0x00;
			SDO_Message.Data[3]   = 0x00;
		break;

		case A_Axsis:
			Orign_Speed = JXS_Parameter.OriginSpeed[A_Axsis]*50;
			SDO_Message.Data[0]   = Orign_Speed;   //0x0064:500
			SDO_Message.Data[1]   = Orign_Speed>>8;
			SDO_Message.Data[2]   = 0x00;
			SDO_Message.Data[3]   = 0x00;
		break;

		case B_Axsis:
			Orign_Speed = JXS_Parameter.OriginSpeed[B_Axsis]*50;
			SDO_Message.Data[0]   = Orign_Speed;   //0x0064:500
			SDO_Message.Data[1]   = Orign_Speed>>8;
			SDO_Message.Data[2]   = 0x00;
			SDO_Message.Data[3]   = 0x00; 
		break;
			
		default:
			SDO_Message.Data[0]   = 0xF4;   //0x0064:500
			SDO_Message.Data[1]   = 0x01;
			SDO_Message.Data[2]   = 0x00;
			SDO_Message.Data[3]   = 0x00; 
		break;		
	}
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //---
   Delay(10);
   /* 5. sub-2 speed during search for zero(1-500)   u32  default:20     */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6099;
    SDO_Message.sub_index = 0x02;
    SDO_Message.len       = 4;
    SDO_Message.Data[0]   = 0x14;   //0x0015:21
    SDO_Message.Data[1]   = 0x00;
    SDO_Message.Data[2]   = 0x00;
    SDO_Message.Data[3]   = 0x00;      
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
  //---
   Delay(10);
//   /* 6.  homing acceleration    u32  default:100   unit:millisecond   */
//    SDO_Message.Node_ID   = Node_ID;
//    SDO_Message.index     = 0x609A;
//    SDO_Message.sub_index = 0x00;
//    SDO_Message.len       = 4;
//    SDO_Message.Data[0]   = 0x90;   //0x0064:100
//    SDO_Message.Data[1]   = 0x01;
//    SDO_Message.Data[2]   = 0x00;
//    SDO_Message.Data[3]   = 0x00;      
//    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
//    Delay(10);

   /* 7. set to servo on drive and make motor work  u16  Default:0x0000  */
    SDO_Message.Node_ID   = Node_ID;
    SDO_Message.index     = 0x6040;
    SDO_Message.sub_index = 0x00;
    SDO_Message.len       = 2;
    SDO_Message.Data[0]   = Controlword;   //0x000F
    SDO_Message.Data[1]   = Controlword>>8;;      
    SDO_Protocol(INITIATE_SDO_DOWNLOAD);
	Delay(10);

}

/***********************************************
 *   Polarity 位置极性                         *
 *   设置状态控制字607Eh 0/1                   *
 *   Polarity：0---Positive nultiply by 1      *
 ***********************************************/
void Change_Polarity_Positive(u8 Node_ID)
{  
   SDO_Message.Node_ID   = Node_ID;
   SDO_Message.index     = 0x607E;
   SDO_Message.sub_index = 0x00;
   SDO_Message.len       = 1;
   SDO_Message.Data[0]   = 0x00;   //     
   SDO_Protocol(INITIATE_SDO_DOWNLOAD);
}

/***********************************************
 *   Polarity 位置极性                         *
 *   设置状态控制字607Eh 0/1                   *
 *   Polarity：1---Positive nultiply by -1      *
 ***********************************************/
void Change_Polarity_Negtive(u8 Node_ID)
{  
   SDO_Message.Node_ID   = Node_ID;
   SDO_Message.index     = 0x607E;
   SDO_Message.sub_index = 0x00;
   SDO_Message.len       = 1;
   SDO_Message.Data[0]   = 0x80;   //    
   SDO_Protocol(INITIATE_SDO_DOWNLOAD);
}

/***********************************************
 *             *
 ***********************************************/
void Set_Halt_SDO(u8 Node_ID)
{
   Controlword = Controlword | 0x0100;
   SDO_Message.Node_ID   = Node_ID;
   SDO_Message.index     = 0x6040;
   SDO_Message.sub_index = 0x00;
   SDO_Message.len       = 2;
   SDO_Message.Data[0]   = Controlword;  
   SDO_Message.Data[1]   = Controlword>>8;     
   SDO_Protocol(INITIATE_SDO_DOWNLOAD);
}

/***********************************************
 *                             *
 ***********************************************/
void Reset_Halt_SDO(u8 Node_ID)
{
   Controlword = Controlword & 0xfeff;
   SDO_Message.Node_ID   = Node_ID;
   SDO_Message.index     = 0x6040;
   SDO_Message.sub_index = 0x00;
   SDO_Message.len       = 2;
   SDO_Message.Data[0]   = Controlword;  
   SDO_Message.Data[1]   = Controlword>>8;     
   SDO_Protocol(INITIATE_SDO_DOWNLOAD);  
}

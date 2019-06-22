/******************** COPYRIGHT 2013 KingRobot *********************************
* File Name          : PDO.c
* Author             : WuXiang
* Version            : V1.0.0
* Date               : 11/14/2013
* Description        : This file complete PDO protocol communication
********************************************************************************/
#include "CANopen.h"
#include "Manual.h"
#include "SpeedControl.h"
#include "StatusControl.h"

CanTxMsg PDO_TxMessage;     //PDO发送消息结构体变量

u8   PDO_TransmitMailbox;   //PDO发送邮箱
vs32 PDO_Position=0;        //发送位置值 
vu32 PDO_Velocity=0;        //发送速度值：0x000003e8                                               
vu8  Flag_Query_Status_Position=0; //是否发起运行状态和位置查询标识位
u8   Change_Set_Immediately = 0;   //是否立即更改设置
//---2017.8.9
u16 Actual_Status_1[6]={0};
u16 Actual_Status[6];
u16 Homing_attained_status[6]={0};       //---回零完成状态字
u16 Target_Rreached[6]={0};              //---2017.8.22目标达到状态字

u32 Pre_Position[6]={0}; //上一次位置0924
u32 Cur_Position[6]={0}; //当前位置
u8 Axsis_Stop_Flag[6]={0};
u16 STD_ID = 0 ; 
u8 Set_Halt_Flag[6]={0};

//-----------------------------------------------//
extern u8 Axis_Step;
//--------------------------------------------------//
/***************************************************************************
**  函数名：  PDO_Process()
**	输入参数：无
**	输出参数：无
**	函数功能：发送PDO消息，设置伺服器位置、速度、加速度、减速度值
**	备注：	  无
**  作者：    吴祥    
**  开发日期：2013/11/14
***************************************************************************/
/***********************************************
 *    Set yarget position: 607Ah(signed32)     *
 *    Profile velocity   : 6081h(unsigned32)   *
 *    PDO Mapping:1401->1601:300h+Node-ID      *
 ***********************************************/
void ServoStart_PDO(u8 Node_ID,u32 Target_Position, u32 Target_Velocity)
{      
	  Node_ID=Node_ID+1;
	  New_Set_Point_Reset(Node_ID);  //设置Control word的第4位New_Set_point为0,停止电机  ---(位置模式下 New_set_point)
	  Delay(10);
	/***    设置伺服器目标位置和速度值    ***/
      PDO_TxMessage.StdId=0x300+Node_ID;        
      PDO_TxMessage.RTR=CAN_RTR_DATA;   
      PDO_TxMessage.IDE=CAN_ID_STD;     
      PDO_TxMessage.DLC=8;                      
      PDO_TxMessage.Data[0]=Target_Position;        //[3:0]:tagget position                
      PDO_TxMessage.Data[1]=Target_Position>>8;    
      PDO_TxMessage.Data[2]=Target_Position>>16;    
      PDO_TxMessage.Data[3]=Target_Position>>24;    
      PDO_TxMessage.Data[4]=Target_Velocity;        //[7:4]:profile velocity
      PDO_TxMessage.Data[5]=Target_Velocity>>8;     //0x01f4:500
      PDO_TxMessage.Data[6]=Target_Velocity>>16;
      PDO_TxMessage.Data[7]=Target_Velocity>>24;
            
      PDO_TransmitMailbox=CAN_Transmit(&PDO_TxMessage);
      CANopen_k = 0;
      while((CAN_TransmitStatus(PDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFFFF)) 
      {
         CANopen_k++;
      }
          //--
		Delay(10);
			
//      /***   配置伺服器的加速度值和减速度值    ***/
//      PDO_TxMessage.StdId=0x400+Node_ID;        
//      PDO_TxMessage.RTR=CAN_RTR_DATA;   
//      PDO_TxMessage.IDE=CAN_ID_STD;     
//      PDO_TxMessage.DLC=8;                      
//      PDO_TxMessage.Data[0]=Target_Servo_Acceleration;        //[3:0]:profile acceleration                
//      PDO_TxMessage.Data[1]=Target_Servo_Acceleration>>8;    
//      PDO_TxMessage.Data[2]=Target_Servo_Acceleration>>16;    
//      PDO_TxMessage.Data[3]=Target_Servo_Acceleration>>24;    
//      PDO_TxMessage.Data[4]=Target_Servo_Deceleration;        //[7:4]:profile deceleration
//      PDO_TxMessage.Data[5]=Target_Servo_Deceleration>>8;     //0x01f4:500
//      PDO_TxMessage.Data[6]=Target_Servo_Deceleration>>16;
//      PDO_TxMessage.Data[7]=Target_Servo_Deceleration>>24;
//            
//      PDO_TransmitMailbox=CAN_Transmit(&PDO_TxMessage);
//      CANopen_k = 0;
//      while((CAN_TransmitStatus(PDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
//      {
//          CANopen_k++;
//      }              
        New_Set_Point_Set(Node_ID);    //设置Control word的第4位New_Set_point为1,启动电机 
             
}

//---发送函数
u8 Can_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=0x12;			// 标准标识符 
	TxMessage.ExtId=0x12;			// 设置扩展标示符 
	TxMessage.IDE=CAN_ID_STD; 	// 标准帧
	TxMessage.RTR=CAN_RTR_DATA;		// 数据帧
	TxMessage.DLC=len;				// 要发送的数据长度
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];			          
	mbox= CAN_Transmit(&TxMessage);   
	i=0; 
	while((CAN_TransmitStatus(mbox) != CANTXOK)&&(CANopen_k != 0xFF))
		 CANopen_k++;	//等待发送结束
	return 1;
}

u8 Can_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
	if( CAN_MessagePending(CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
	CAN_Receive(CAN_FIFO0, &RxMessage);//读取数据	
	for(i=0;i<8;i++)
	buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}

/***********************************************
 *   Profile Position Mode 位置模式            *
 *   设置状态控制字6040h第 位             *
 *                       *
 *   PDO Mapping  : 1400->1600:200h+Node_ID    *
 ***********************************************/
void ServoStop_PDO(u8 Node_ID)
{  
    Set_Halt(Node_ID);
	Set_Halt_Flag[Node_ID-1]=TRUE;//--0925
}

/***********************************************
 *   Profile Position Mode 位置模式            *
 *   设置状态控制字6040h第 位             *
 *                       *
 *   PDO Mapping  : 1400->1600:200h+Node_ID    *
 ***********************************************/
void ServoEmergencyStop_PDO(u8 Node_ID)
{  
    Change_Servo_State_Machine(Node_ID,QUICK_STOP);
}

/*********************************************************
 *      设置状态控制字6040h第5位:Change_Set_Immediately  *
 *       ：在修改位置参数之后是否直接修改伺服器          *
 *      0: SINGLE_SET_POINT                              *
 *      1: CHANGE_SET_IMMEDIATELY                        *
 *********************************************************/
void Change_Set(u8 Change_Set_Immediately)
{
   switch(Change_Set_Immediately)
   {
       case SINGLE_SET_POINT        : Controlword = Controlword & 0xffdf ; break;
       case CHANGE_SET_IMMEDIATELY  : Controlword = Controlword | 0x0020 ; break;
       default  : break;
   }
}

/***********************************************
 *   Profile Position Mode 位置模式            *
 *   设置状态控制字6040h第4位触发位            *
 *   New_Set_Point：0  失能                    *
 *   PDO Mapping  : 1400->1600:200h+Node_ID    *
 ***********************************************/
void New_Set_Point_Reset(u8 Node_ID)
{  
   Controlword = Controlword & 0xffef; 
   PDO_TxMessage.StdId=0x200+Node_ID;        
   PDO_TxMessage.RTR=CAN_RTR_DATA;   
   PDO_TxMessage.IDE=CAN_ID_STD;     
   PDO_TxMessage.DLC=2;                         
   PDO_TxMessage.Data[0]=Controlword;
   PDO_TxMessage.Data[1]=Controlword>>8;
        
   PDO_TransmitMailbox=CAN_Transmit(&PDO_TxMessage);
   CANopen_k = 0;
   while((CAN_TransmitStatus(PDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
   {
       CANopen_k++;
   }
}

/***********************************************
 *   Profile Position Mode 位置模式            *
 *   设置状态控制字6040h第4位触发位            *
 *   New_Set_Point：1  使能                    *
 *   PDO Mapping  : 1400->1600:200h+Node_ID    *
 ***********************************************/
void New_Set_Point_Set(u8 Node_ID)
{
      Controlword = Controlword | 0x0010;
      PDO_TxMessage.StdId=0x200+Node_ID;        
      PDO_TxMessage.RTR=CAN_RTR_DATA;   
      PDO_TxMessage.IDE=CAN_ID_STD;     
      PDO_TxMessage.DLC=2;                        
      PDO_TxMessage.Data[0]=Controlword;
      PDO_TxMessage.Data[1]=Controlword>>8;
    
      PDO_TransmitMailbox=CAN_Transmit(&PDO_TxMessage);
      CANopen_k = 0;
      while((CAN_TransmitStatus(PDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
      {
          CANopen_k++;
      }  
}

/***********************************************
 *   HOMING　MODE 回原点模式                   *
 *   设置状态控制字6040h第4位触发位            *
 *   Homing operation start：0  失能           *
 *   PDO Mapping  : 1400->1600:200h+Node_ID    *
 ***********************************************/
void Homing_Operation_Start_Reset(u8 Node_ID)
{  
   Controlword = Controlword & 0xffef; 
   PDO_TxMessage.StdId=0x200+Node_ID;        
   PDO_TxMessage.RTR=CAN_RTR_DATA;   
   PDO_TxMessage.IDE=CAN_ID_STD;     
   PDO_TxMessage.DLC=2;                         
   PDO_TxMessage.Data[0]=Controlword;
   PDO_TxMessage.Data[1]=Controlword>>8;
        
   PDO_TransmitMailbox=CAN_Transmit(&PDO_TxMessage);
   CANopen_k = 0;
   while((CAN_TransmitStatus(PDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF))//---由FF改成0F 
   {
        CANopen_k++;
   }
}

/***********************************************
 *   HOMING　MODE 回原点模式                   *
 *   设置状态控制字6040h第4位触发位            *
 *   Homing operation start：1  使能           *
 *   PDO Mapping  : 1400->1600:200h+Node_ID    *
 ***********************************************/
void Homing_Operation_Start_Set(u8 Node_ID)
{  
      Controlword = Controlword | 0x0010;
      PDO_TxMessage.StdId=0x200+Node_ID;        
      PDO_TxMessage.RTR=CAN_RTR_DATA;   
      PDO_TxMessage.IDE=CAN_ID_STD;     
      PDO_TxMessage.DLC=2;                        
      PDO_TxMessage.Data[0]=Controlword;
      PDO_TxMessage.Data[1]=Controlword>>8;
    
      PDO_TransmitMailbox=CAN_Transmit(&PDO_TxMessage);
      CANopen_k = 0;
      while((CAN_TransmitStatus(PDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF))//---同上 
      {
          CANopen_k++;
      }  
}

/***************************************************************************
**  函数名：  Home_Stop_Axis()
**	输入参数：无
**	输出参数：无
**	函数功能：停止运动轴
**	备注：	  6040h bit8 :halt	stop axis with homing acceleration
**  作者：    吴祥    
**  开发日期：2013/11/22
***************************************************************************/
void Home_Stop_Axis(u8 Node_ID)
{
	 Set_Halt(Node_ID);
}

/***************************************************************************
**  函数名：  	Set_Halt()
**	输入参数：	无
**	输出参数：	无
**	函数功能：	将控制字6040h的第8位置1
**	备注：		无
**  作者：    	吴祥    
**  开发日期：	2013/11/22
***************************************************************************/
void Set_Halt(u8 Node_ID)
{
	  Controlword = Controlword | 0x0100;
      PDO_TxMessage.StdId=0x200+Node_ID;
      PDO_TxMessage.RTR=CAN_RTR_DATA;   
      PDO_TxMessage.IDE=CAN_ID_STD;     
      PDO_TxMessage.DLC=2;				
      PDO_TxMessage.Data[0]=Controlword;
      PDO_TxMessage.Data[1]=Controlword>>8;
    
      PDO_TransmitMailbox=CAN_Transmit(&PDO_TxMessage);
      CANopen_k = 0;
      while((CAN_TransmitStatus(PDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
      {
          CANopen_k++;
      }			
}

/***************************************************************************
**  函数名：  Reset_Halt()
**	输入参数：无
**	输出参数：无
**	函数功能：将控制字6040h的第8位置0
**	备注：	  无
**  作者：    吴祥    
**  开发日期：2013/11/22
***************************************************************************/
void Reset_Halt(u8 Node_ID)
{
	  Node_ID=Node_ID+1;//--0926
	  Controlword = Controlword & 0xfeff;
      PDO_TxMessage.StdId=0x200+Node_ID;        
      PDO_TxMessage.RTR=CAN_RTR_DATA;   
      PDO_TxMessage.IDE=CAN_ID_STD;     
      PDO_TxMessage.DLC=2;                        
      PDO_TxMessage.Data[0]=Controlword;
      PDO_TxMessage.Data[1]=Controlword>>8;
    
      PDO_TransmitMailbox=CAN_Transmit(&PDO_TxMessage);
      CANopen_k = 0;
      while((CAN_TransmitStatus(PDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFFFF)) 
      {
         CANopen_k++;
      }    		
}
/***********************************************************
 *      Function:Query: status word,Position actual value  *
 *      status word           : 6041h                      *
 *      Position actual value : 6064h                      *
 ***********************************************************/
void Query_stWord_acPosition(u8 Node_ID)
{

      PDO_TxMessage.StdId=0x180+Node_ID;        
      PDO_TxMessage.RTR=CAN_RTR_REMOTE;                 //发送远程请求帧
      PDO_TxMessage.IDE=CAN_ID_STD;     
          
      PDO_TransmitMailbox=CAN_Transmit(&PDO_TxMessage);
      CANopen_k = 0;
      while((CAN_TransmitStatus(PDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
      {
          CANopen_k++;
      } 
}

/***********************************************
 *       State_Machine--状态机切换            *
 ***********************************************/
void Change_Servo_State_Machine(u8 Node_ID,u16 Cmd)
{
      Controlword = (Controlword&0Xfff0) | Cmd;
      PDO_TxMessage.StdId=0x200+Node_ID;   //此处为RPDO1     
      PDO_TxMessage.RTR=CAN_RTR_DATA;   //Remote Tranmission Request BIT 全称为远程发送请求位
      PDO_TxMessage.IDE=CAN_ID_STD;     
      PDO_TxMessage.DLC=2;               //数据帧长度代码DLC         
      PDO_TxMessage.Data[0]=Controlword;
      PDO_TxMessage.Data[1]=Controlword>>8;
    
      PDO_TransmitMailbox=CAN_Transmit(&PDO_TxMessage);
      CANopen_k = 0;
      while((CAN_TransmitStatus(PDO_TransmitMailbox) != CANTXOK) && (CANopen_k != 0xFF)) 
      {
          CANopen_k++;
      } 
}

/*************************************************************
 *      Function:判断是否到达目标位置、是否回0点             *
 *      status word bit 10   : Reached CAN_Recieve_Data[0:1] *
 *      Position actual value: 6064h   CAN_Recieve_Data[2:5] *
 *************************************************************/
//“0x30  0x1F  0x00  0x00”就代表了数据内容，
//在CAN协议内，帧数据是低位在前，高位在后的。
//所以上述数据直接读取后得出编码器送出1F30H，这就代表了编码器的当前位置值。
void Get_Status_Position()
{  
 switch(STD_ID)
	{
		case 0x181://对应节点1 总共181-1FF个节点 最大127个节点
			Real_Time_Position_1[X_Axsis]=CAN_Recieve_Data[5]<<24| CAN_Recieve_Data[4]<<16| CAN_Recieve_Data[3]<<8| CAN_Recieve_Data[2];
			Pulse_TotalCounter[X_Axsis]=1000000+Real_Time_Position_1[X_Axsis];					
			Actual_Status[X_Axsis]=(u16) ( ((u16)CAN_Recieve_Data[1]<<8)| ((u16)CAN_Recieve_Data[0]));
//	     Homing_attained_status[X_Axsis]=(Actual_Status[X_Axsis]>>12)&0x0001;//---为了获取第13位，回零完成标志位
//       Target_Rreached[X_Axsis]=(Actual_Status[X_Axsis]>>10)&0x0001;     //---2017.8.22
		   if(((Actual_Status[X_Axsis]>>12)&0x0001)&&0==Homing_attained_status[X_Axsis])
		   {
			    Homing_attained_status[X_Axsis]=1;
				Axis_Step++;
			}
			 
			if(((Actual_Status[X_Axsis]>>10)&0x0001)==1)
			{
			    Target_Rreached[X_Axsis]=1;
				Timer_Enable_X = DISABLE;
			 }			 
		
			 if(Axsis_Stop_Flag[X_Axsis]==0) //---0925 轴停止状态标志位
			 {
				 Cur_Position[X_Axsis]=Real_Time_Position_1[X_Axsis];
				 if(Cur_Position[X_Axsis]== Pre_Position[X_Axsis])
				 {
					Axsis_Stop_Flag[X_Axsis]=1;						 
				 }
				 else
				 {
					Axsis_Stop_Flag[X_Axsis]=0;
				 }
				 Pre_Position[X_Axsis]=Cur_Position[X_Axsis];
			 }
		
  		break;
		case 0x182:
			Real_Time_Position_1[Z_Axsis]=CAN_Recieve_Data[5]<<24| CAN_Recieve_Data[4]<<16| CAN_Recieve_Data[3]<<8| CAN_Recieve_Data[2];
			Pulse_TotalCounter[Z_Axsis]=1000000+Real_Time_Position_1[Z_Axsis];					
			Actual_Status[Z_Axsis]=(u16) ( ((u16)CAN_Recieve_Data[1]<<8)| ((u16)CAN_Recieve_Data[0]));
//       Homing_attained_status[Z_Axsis]=(Actual_Status[Z_Axsis]>>12)&0x0001;
//       Target_Rreached[Z_Axsis]=(Actual_Status[Z_Axsis]>>10)&0x0001;     //---2017.8.22
		   if(((Actual_Status[Z_Axsis]>>12)&0x0001)&&0==Homing_attained_status[Z_Axsis])
		   {
			    Homing_attained_status[Z_Axsis]=1;
				Axis_Step++;
			 }

		   if(((Actual_Status[Z_Axsis]>>10)&0x0001)==1)
			{
			    Target_Rreached[Z_Axsis]=1;
				Timer_Enable_Z = DISABLE;
			 }	
		
			 if(Axsis_Stop_Flag[Z_Axsis]==0)
			 {
				 Cur_Position[Z_Axsis]=Real_Time_Position_1[Z_Axsis];
				 if(Cur_Position[Z_Axsis]== Pre_Position[Z_Axsis])
				 {
					Axsis_Stop_Flag[Z_Axsis]=1;					 
				 }
				 else
				 {
					Axsis_Stop_Flag[Z_Axsis]=0;
				 }
				 Pre_Position[Z_Axsis]=Cur_Position[Z_Axsis];
			}
			 
		  break;
		case 0x183:
    	 Real_Time_Position_1[L_Axsis]=CAN_Recieve_Data[5]<<24| CAN_Recieve_Data[4]<<16| CAN_Recieve_Data[3]<<8| CAN_Recieve_Data[2];
		Pulse_TotalCounter[L_Axsis]=1000000+Real_Time_Position_1[L_Axsis];					
		Actual_Status[L_Axsis]=(u16) ( ((u16)CAN_Recieve_Data[1]<<8)| ((u16)CAN_Recieve_Data[0]));
//       Homing_attained_status[L_Axsis]=(Actual_Status[L_Axsis]>>12)&0x0001;
//       Target_Rreached[L_Axsis]=(Actual_Status[L_Axsis]>>10)&0x0001;
		   if(((Actual_Status[L_Axsis]>>12)&0x0001)&&0==Homing_attained_status[L_Axsis])
		   {
			    Homing_attained_status[L_Axsis]=1;
				Axis_Step++;
			 }
		
			if(((Actual_Status[L_Axsis]>>10)&0x0001)==1)
			{
			    Target_Rreached[L_Axsis]=1;
				Timer_Enable_L = DISABLE;
			 }		
		
			 if(Axsis_Stop_Flag[L_Axsis]==0)
			 {
				 Cur_Position[L_Axsis]=Real_Time_Position_1[L_Axsis];
				 if(Cur_Position[L_Axsis]== Pre_Position[L_Axsis])
				 {
					Axsis_Stop_Flag[L_Axsis]=1;						 
				 }
				 else
				 {
					Axsis_Stop_Flag[L_Axsis]=0;
				 }
				 Pre_Position[L_Axsis]=Cur_Position[L_Axsis];
			 }

			break;
		case 0x184:
			Real_Time_Position_1[O_Axsis]=CAN_Recieve_Data[5]<<24| CAN_Recieve_Data[4]<<16| CAN_Recieve_Data[3]<<8| CAN_Recieve_Data[2];
			Pulse_TotalCounter[O_Axsis]=1000000+Real_Time_Position_1[O_Axsis];					
			Actual_Status[O_Axsis]=(u16) ( ((u16)CAN_Recieve_Data[1]<<8)| ((u16)CAN_Recieve_Data[0]));
//		   Homing_attained_status[O_Axsis]=(Actual_Status[2]>>12)&0x0001;
//		   Target_Rreached[O_Axsis]=(Actual_Status[O_Axsis]>>10)&0x0001;
		   if(((Actual_Status[O_Axsis]>>12)&0x0001)&&0==Homing_attained_status[O_Axsis])
		   {
			    Homing_attained_status[O_Axsis]=1;
				Axis_Step++;
			 }
			 
			if(((Actual_Status[O_Axsis]>>10)&0x0001)==1)
			{
			    Target_Rreached[O_Axsis]=1;
				Timer_Enable_O = DISABLE;
			 }		
		
			 if(Axsis_Stop_Flag[O_Axsis]==0)
			 {
				 Cur_Position[O_Axsis]=Real_Time_Position_1[O_Axsis];
				 if(Cur_Position[O_Axsis]== Pre_Position[O_Axsis])
				 {
					Axsis_Stop_Flag[O_Axsis]=1;						 
				 }
				 else
				 {
					Axsis_Stop_Flag[O_Axsis]=0;
				 }
				 Pre_Position[O_Axsis]=Cur_Position[O_Axsis];
			 }

			break;
			 
		case 0x185:
			 Real_Time_Position_1[A_Axsis]=CAN_Recieve_Data[5]<<24| CAN_Recieve_Data[4]<<16| CAN_Recieve_Data[3]<<8| CAN_Recieve_Data[2];
			Pulse_TotalCounter[A_Axsis]=1000000+Real_Time_Position_1[A_Axsis];					
			Actual_Status[A_Axsis]=(u16) ( ((u16)CAN_Recieve_Data[1]<<8)| ((u16)CAN_Recieve_Data[0]));
//       Homing_attained_status[A_Axsis]=(Actual_Status[2]>>12)&0x0001;
//       Target_Rreached[A_Axsis]=(Actual_Status[A_Axsis]>>10)&0x0001;
		   if(((Actual_Status[A_Axsis]>>12)&0x0001)&&0==Homing_attained_status[A_Axsis])
		   {
			    Homing_attained_status[A_Axsis]=1;
				Axis_Step++;
			 }
			 
			if(((Actual_Status[A_Axsis]>>10)&0x0001)==1)
			{
			    Target_Rreached[A_Axsis]=1;
				Timer_Enable_A = DISABLE;
			 }		
		
			 if(Axsis_Stop_Flag[A_Axsis]==0)
			 {
				 Cur_Position[A_Axsis]=Real_Time_Position_1[A_Axsis];
				 if(Cur_Position[A_Axsis]== Pre_Position[A_Axsis])
				 {
					Axsis_Stop_Flag[A_Axsis]=1;						 
				 }
				 else
				 {
					Axsis_Stop_Flag[A_Axsis]=0;
				 }
				 Pre_Position[A_Axsis]=Cur_Position[A_Axsis];
			 }

			break;

		case 0x186:
			 Real_Time_Position_1[B_Axsis]=CAN_Recieve_Data[5]<<24| CAN_Recieve_Data[4]<<16| CAN_Recieve_Data[3]<<8| CAN_Recieve_Data[2];
			Pulse_TotalCounter[B_Axsis]=1000000+Real_Time_Position_1[B_Axsis];					
			Actual_Status[B_Axsis]=(u16) ( ((u16)CAN_Recieve_Data[1]<<8)| ((u16)CAN_Recieve_Data[0]));
//       Homing_attained_status[B_Axsis]=(Actual_Status[2]>>12)&0x0001;
//       Target_Rreached[B_Axsis]=(Actual_Status[B_Axsis]>>10)&0x0001;
		   if(((Actual_Status[B_Axsis]>>12)&0x0001)&&0==Homing_attained_status[B_Axsis])
		   {
			    Homing_attained_status[B_Axsis]=1;
				Axis_Step++;
			 }
			 
			if(((Actual_Status[B_Axsis]>>10)&0x0001)==1)
			{
			    Target_Rreached[B_Axsis]=1;
				Timer_Enable_B = DISABLE;
			 }		
		
			 if(Axsis_Stop_Flag[B_Axsis]==0)
			 {
				 Cur_Position[B_Axsis]=Real_Time_Position_1[B_Axsis];
				 if(Cur_Position[B_Axsis]== Pre_Position[B_Axsis])
				 {
					Axsis_Stop_Flag[B_Axsis]=1;						 
				 }
				 else
				 {
					Axsis_Stop_Flag[B_Axsis]=0;
				 }
				 Pre_Position[B_Axsis]=Cur_Position[B_Axsis];
			 }

			break;
  }
}

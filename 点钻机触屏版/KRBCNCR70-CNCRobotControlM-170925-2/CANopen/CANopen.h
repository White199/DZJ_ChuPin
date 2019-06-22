#include "stm32f10x_lib.h"
#include "Delay.h"
#include "PDO.h"
#include "SDO.h"
#include "NMT.h"
#include "Sync.h"
#include "EMCY.h"
#include "SpeedControl_CAN.h" 

#define  CANOPEN_MODE 
//#define PULSE_MODE 
//#define NMT              
//#define SDO              
//#define PDO              
//#define SYNC        
#define SERVO_NODE_ID_00 0     
#define SERVO_NODE_ID_01 1     
#define SERVO_NODE_ID_02 2
#define SERVO_NODE_ID_03 3
#define SERVO_NODE_ID_04 4
#define SERVO_NODE_ID_05 5  
#define SERVO_NODE_ID_06 6  
#define SINGLE_SET_POINT       (u8)0
#define CHANGE_SET_IMMEDIATELY (u8)1
#define PROFILE_POSITION_MODE  1
#define HOMING_MODE            6

#define  SHUTDOWN           0X0006
#define  SWITCH_ON          0X0007
#define  ENABLE_OPERATION   0X000F 
#define  DISABLE_OPERATION  0X0005 
#define  QUICK_STOP         0X0002
#define  DISABLE_VOLTAGE    0X0000

/***********************************
 *            variable             *
 ***********************************/
typedef enum { Initialisation=0, Pre_Operate=127, Operate=5, Stop=4 } State_Machine_state ;
typedef enum { NMT=0, SDO=1, PDO=2, SYNC=3 } Function ; 

typedef struct  //OD,对象字典结构体
{
  u16 index;
  u8  subindex;
  u8  ctr;
  u16 length;
}DICT_OBJECT ;

typedef struct   //PDO message
{
   u32 PDO_COB_ID;
   u8  len;
   u8  PDO_data[8];
}PDO_Struct;

typedef struct   //SDO-8字节数据格式
{
  u8 SDO_Scs;    //命令字
  u8 data[7];
}SDO_Mes;

typedef struct   //SDO message
{
  u8  Node_ID;
  u16 index;
  u8  sub_index;
  u8  len;      //传输数据的长度，+4变为总得can报文长度
  u8  Data[8];
}SDO_Struct;


extern State_Machine_state  State_Machine;    //状态机
extern Function             Function_Code;    //功能码
extern SDO_Struct           SDO_Message;      //SDO信息帧
extern u8   ServoNodeID_0 ;                 //节点ID号
extern u8   ServoNodeID_1 ;                 //节点1的ID号
extern u8   ServoNodeID_2 ;                 //节点2的ID号
extern u8   ServoNodeID_3 ;                 //节点3的ID号
extern u8   ServoNodeID_4 ;                 //节点4的ID号
extern u8   ServoNodeID_5 ;                 //节点5的ID号
extern u8   ServoNodeID_6 ;                 //节点6的ID号
extern vu8  Servo_Node_ID ;                 //伺服器节点ID号变量
extern vu16 COB_ID ;                        //交流对象ID
extern vu16 Command_Specifier;              //cs:命令符
extern u32  CANopen_k ;
extern vu8  CAN_Recieve_Data[8];
extern vu8  Send_Node_guard;                  //Node_guard信息发送标示位
//extern u8   Servo_On;                         //伺服器Servo on
extern vs32 PDO_Position;                     //发送给伺服器的位置值
extern vu32 PDO_Velocity;                     //发送给伺服器的速度值
extern u8   Flag_Reached_Point;               //是否到达给定位置标志位
extern u8   Reached;                          //状态字6041h的第10位Reached标志位
extern u8   Set_Point_Acknowledge;            //轨迹发生器是否设定速度值
extern u8   Change_Velocity;
extern u8   Get_Position_zero;                //是否达到位置0
extern vu8  Flag_Get_Node_State;              //得到节点通信状态标识位
extern vu8  Flag_Query_Status_Position;       //是否发起运行状态和位置查询标识位
extern u8   Flag_Chang_Operation_Mode  ;
extern u8   Change_Set_Immediately;           //判断在得到新位置值的时候是否速度降为0再改变
extern vu16 Controlword;                      //状态字


extern vu32 Target_Servo_Position ;           //伺服电机运动的目标位置值
extern vu32 Target_Servo_Velocity ;           //伺服电机运动的目标速度值
extern vu32 Target_Servo_Acceleration ;      //伺服电机运动的加速度值
extern vu32 Target_Servo_Deceleration ;      //伺服电机运动的加速度值
/***********************************
 *          Machine State          *
 ***********************************/
void Pre_Operational(void);
void Operational(void);
void Stopped(void);
void RPDO_Mapping(u8);
void TPDO_Mapping(u8);
void PDO_Mapping_Deleted(u8);
void PDO_Mapping_test(u8);
void Not_Valid_PDO(u8);
void CANopen_Init(void);
void Chang_Operation_Mode(u8,u8)	;
//---
void PP_Mode_Init(void);   
void Homing_Mode_Init(void);
/***********************************
 *          User  Fuction          *
 ***********************************/
State_Machine_state  Get_state(void);
Function      Get_Fuction_Code(vu16); //COB_ID

